#include <FS.h> //this needs to be first, or it all crashes and burns...
#include <LittleFS.h>

#include <ESP8266WiFi.h> //https://github.com/esp8266/Arduino

#include "tcs_protocol.h"

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager

#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson

#include <PubSubClient.h>

#include <time.h>

#define VERSION "1.0.6"

void publish_tcs_command(char *topic, CommandData data, bool direction);
void mqtt_output(char *format, ...);

const char* ntpServer = "pool.ntp.org";
char tz[48] = "CET-1CEST,M3.5.0,M10.5.0/3"; // POSIX TZ string, configurable via config.json "tz"

// Debug output can be switched on/off from the web GUI (http://<device>/) and is persisted in config.json
bool debug_enabled = true;
#define DBG(...) do { if (debug_enabled) Serial.printf(__VA_ARGS__); } while (0)

ESP8266WebServer web(80);

WiFiClient net;
PubSubClient client(net);

//JsonDocument intercomUsers;
DynamicJsonDocument intercomUsers(8192);

//define your default values here, if there are different values in config.json, they are overwritten.
char mqtt_server[40] = "192.168.10.243";
char mqtt_port[6] = "7953";
char mqtt_topic[64] = "ucl/by-mqtt-client/intercom";
char control4_command_topic[64] = "ucl/by-unid/zw-D80822E0-0084/ep0/OnOff/Commands/Toggle";
char control4_notify_topic[80] = "ucl/by-unid/zw-D80822E0-0084/ep0/IASZone/Attributes/ZoneStatus/Reported";
char tcs_command_topic[64] = "ucl/by-mqtt-client/intercom/command";
char mqtt_client[32] = "anovakovic";
char mqtt_user[32] = "";
char mqtt_pass[32] = "";

uint32_t my_serial_number = 0x8c0b9;
uint32_t last_bus_address = 3;      // address of the door station involved in the latest door-related frame
uint32_t last_bus_address_ms = 0;   // millis() of the last update
uint32_t bus_address_changes = 0;
volatile uint32_t isr_overruns = 0; // frames dropped because the previous one was not consumed yet

// Copy a JSON string value into dst safely; keeps the current value if the key is missing/not a string
static void load_str(JsonDocument &json, const char *key, char *dst, size_t size) {
  const char *v = json[key] | (const char*)nullptr;
  if (v) strlcpy(dst, v, size);
}

// Read a whole file into a NUL-terminated buffer
static std::unique_ptr<char[]> read_file(const char *path) {
  File f = LittleFS.open(path, "r");
  if (!f) return nullptr;
  size_t size = f.size();
  std::unique_ptr<char[]> buf(new char[size + 1]);
  size_t n = f.readBytes(buf.get(), size);
  buf[n] = 0;
  f.close();
  return buf;
}

//flag for saving data
bool shouldSaveConfig = false;

#define inputPin    14  // D5 - GPIO14
#define outputPin   12  // D6 - GPIO15
#define restorePin  0   // D0
#define startBit    6
#define oneBit      4
#define zeroBit     2

volatile uint32_t CMD = 0;
volatile uint8_t lengthCMD = 0;
volatile bool cmdReady = false;

// Transmit queue: frames are bit-banged by a hardware timer so the main loop never blocks
#define TX_QUEUE_SIZE 4
#define TX_MAX_STEPS  40
#define TX_GAP_MS     20
struct TxFrame { uint32_t cmd; bool is_long; };
TxFrame tx_queue[TX_QUEUE_SIZE];
uint8_t tx_q_head = 0;
uint8_t tx_q_count = 0;
uint32_t tx_dropped = 0;
volatile uint16_t tx_steps[TX_MAX_STEPS]; // ms between edges
volatile uint8_t tx_n = 0;
volatile uint8_t tx_i = 0;
volatile bool tx_busy = false;
volatile bool tx_level = false;

void get_tcsuser_data_by_addr(uint32_t u32_addr, char *_name, size_t name_size) {
uint32_t _addr = 0;

  //Serial.println("================================================================\n");
  //Serial.println("TONY:  Let's try to find " + String(u32_addr));

  for (JsonObject elem : intercomUsers.as<JsonArray>()) {
    const char* name = elem["name"];
    const char* addr = elem["tcs_addr"];

    if (!name || !addr) continue;
    sscanf(addr, "%x", &_addr);

    if (u32_addr == _addr) {
      strlcpy(_name, name, name_size);
    }
  }
}

// Short STOP_TALKING frame for the given bus address
uint32_t stop_talking(uint32_t address) {
  uint32_t command = buildCommand(COMMAND_TYPE_STOP_TALKING, address).command;

  DBG("Stop talking %08x\n", (unsigned)command);

  return command;
}

// Never blocks: if NTP has not synced yet the time reads as 1970
struct tm current_time(char *mqtt_timestamp) {
  time_t now = time(nullptr);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);

  strftime(mqtt_timestamp, 12, "%H:%M:%S", &timeinfo);

  return timeinfo;
}

void ICACHE_RAM_ATTR analyzeCMD()
{
    if (tx_busy) return; // ignore our own edges
    static uint32_t curCMD;
    static uint32_t usLast;
    static byte curCRC;
    static byte calCRC;
    static byte curLength;
    static byte cmdIntReady;
    static byte curPos;
    uint32_t usNow = micros();
    uint32_t timeInUS = usNow - usLast;
    usLast = usNow;
    byte curBit = 4;
    if (timeInUS >= 1000 && timeInUS <= 2999)
    {
        curBit = 0;
    }
    else if (timeInUS >= 3000 && timeInUS <= 4999)
    {
        curBit = 1;
    }
    else if (timeInUS >= 5000 && timeInUS <= 6999)
    {
        curBit = 2;
    }
    else if (timeInUS >= 7000 && timeInUS <= 24000)
    {
        curBit = 3;
        curPos = 0;
    }

    if (curPos == 0)
    {
        if (curBit == 2)
        {
            curPos++;
        }
        curCMD = 0;
        curCRC = 0;
        calCRC = 1;
        curLength = 0;
    }
    else if (curBit == 0 || curBit == 1)
    {
        if (curPos == 1)
        {
            curLength = curBit;
            curPos++;
        }
        else if (curPos >= 2 && curPos <= 17)
        {
            if (curBit)
                bitSet(curCMD, (curLength ? 33 : 17) - curPos);
            calCRC ^= curBit;
            curPos++;
        }
        else if (curPos == 18)
        {
            if (curLength)
            {
                if (curBit)
                    bitSet(curCMD, 33 - curPos);
                calCRC ^= curBit;
                curPos++;
            }
            else
            {
                curCRC = curBit;
                cmdIntReady = 1;
            }
        }
        else if (curPos >= 19 && curPos <= 33)
        {
            if (curBit)
                bitSet(curCMD, 33 - curPos);
            calCRC ^= curBit;
            curPos++;
        }
        else if (curPos == 34)
        {
            curCRC = curBit;
            cmdIntReady = 1;
        }
    }
    else
    {
        curPos = 0;
    }
    if (cmdIntReady)
    {
        cmdIntReady = 0;
        if (curCRC == calCRC)
        {
            if (cmdReady) isr_overruns++;
            cmdReady = 1;
            lengthCMD = curLength;
            CMD = curCMD;
        }
        curCMD = 0;
        curPos = 0;
    }
}

void ICACHE_RAM_ATTR tx_timer_isr() {
  tx_i++;
  if (tx_i >= tx_n) {
    GPOC = (1 << outputPin);
    tx_busy = false;
    return;
  }
  tx_level = !tx_level;
  if (tx_level) GPOS = (1 << outputPin); else GPOC = (1 << outputPin);
  timer1_write((uint32_t)tx_steps[tx_i] * 5000); // 5 MHz timer: 5000 ticks per ms
}

// Starts transmitting one frame; same waveform as before: start, length bit, 16/32 data bits, parity
static void tx_start(uint32_t cmd, bool is_long) {
  int length = is_long ? 32 : 16;
  byte checksm = 1;
  uint8_t n = 0;

  tx_steps[n++] = startBit;
  tx_steps[n++] = is_long ? oneBit : zeroBit;
  for (int i = length; i > 0; i--) {
    int bit = bitRead(cmd, i - 1);
    tx_steps[n++] = bit ? oneBit : zeroBit;
    checksm ^= bit;
  }
  tx_steps[n++] = checksm ? oneBit : zeroBit;

  tx_n = n;
  tx_i = 0;
  tx_level = true;
  tx_busy = true;
  GPOS = (1 << outputPin);
  timer1_attachInterrupt(tx_timer_isr);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  timer1_write((uint32_t)tx_steps[0] * 5000);
}

bool queue_command(uint32_t cmd, bool is_long) {
  if (tx_q_count >= TX_QUEUE_SIZE) {
    tx_dropped++;
    DBG("TX queue full, dropping %08x\n", (unsigned)cmd);
    return false;
  }
  tx_queue[(tx_q_head + tx_q_count) % TX_QUEUE_SIZE] = { cmd, is_long };
  tx_q_count++;
  return true;
}

// Called from loop(): starts the next queued frame once the bus is idle
void tx_service() {
  static bool started = false;
  static uint32_t end_ms = 0;

  if (started && !tx_busy) {
    started = false;
    end_ms = millis();
  }
  if (!started && tx_q_count > 0 && millis() - end_ms >= TX_GAP_MS) {
    TxFrame f = tx_queue[tx_q_head];
    tx_q_head = (tx_q_head + 1) % TX_QUEUE_SIZE;
    tx_q_count--;
    DBG("TX %08x (%s)\n", (unsigned)f.cmd, f.is_long ? "long" : "short");
    started = true;
    tx_start(f.cmd, f.is_long);
  }
}

// Pulses the Control4 IAS zone: set, then clear
void control4_notify(bool alarm1, bool alarm2, const CommandData &d) {
  char msg[200];
  const char *fmt = "{\"value\":{\"ACMainsFault\":false,\"Alarm1\":%s,\"Alarm2\":%s,\"BatteryDefect\":false,\"BatteryLow\":false,\"RestoreNotify\":false,\"SupervisionNotify\":false,\"Tamper\":false,\"Test\":false,\"Trouble\":false}}";

  snprintf(msg, sizeof(msg), fmt, alarm1 ? "true" : "false", alarm2 ? "true" : "false");
  client.publish(control4_notify_topic, msg);
  snprintf(msg, sizeof(msg), fmt, "false", "false");
  client.publish(control4_notify_topic, msg);

  mqtt_output((char*)"\"type\": \"control4_notify\", \"alarm\": \"%s\", \"address\": \"%02x\", \"serial_number\": \"%05x\"",
              alarm1 ? "alarm1" : "alarm2", (unsigned)d.address, (unsigned)d.serial_number);
}

void mqtt_output(char *format, ...) {
    char mqtt_tmp[160];
    char mqtt_msg[256];
    char mqtt_time[12];
    char mqtt_timestamp[20];
    struct tm timeinfo;
    va_list args;

    va_start (args, format);
    vsnprintf(mqtt_tmp, sizeof(mqtt_tmp), format, args);
    va_end (args);

    timeinfo=current_time((char*)&mqtt_time);
    strftime(mqtt_timestamp, 11, "%s", &timeinfo);

    snprintf(mqtt_msg, sizeof(mqtt_msg), "{ \"ts\": \"%s\", \"time\": \"%s\", %s }", mqtt_timestamp, mqtt_time, mqtt_tmp);
    client.publish(mqtt_topic, mqtt_msg);
}

// Retained "online"/"offline" availability, with offline as the last will
static void topic_with_suffix(char *out, size_t size, const char *suffix) {
  snprintf(out, size, "%s/%s", mqtt_topic, suffix);
}

// Single non-blocking attempt; the caller rate-limits retries
bool connectToMqtt() {
  char mcli[64];
  char avail[96];
  // Stable client ID so the broker replaces the old session instead of piling up new ones
  snprintf(mcli, sizeof(mcli), "%s-%06x", mqtt_client, (unsigned)ESP.getChipId());
  topic_with_suffix(avail, sizeof(avail), "availability");

  DBG("connecting %s to MQTT...\n", mcli);
  if (!client.connect(mcli, mqtt_user, mqtt_pass, avail, 0, true, "offline")) {
    DBG("MQTT connect failed, rc=%d\n", client.state());
    return false;
  }

  client.subscribe(tcs_command_topic);
  client.subscribe(control4_command_topic);
  client.publish(avail, "online", true);
  DBG("MQTT connected, subscribed\n");
  mqtt_output((char*)"\"type\": \"sniffer_online\", \"version\": \"" VERSION "\"");
  return true;
}

// Periodic health report on <mqtt_topic>/status
void publish_status() {
  char topic[96];
  char msg[256];
  topic_with_suffix(topic, sizeof(topic), "status");
  snprintf(msg, sizeof(msg),
           "{ \"version\": \"" VERSION "\", \"uptime\": %lu, \"rssi\": %d, \"free_heap\": %u, \"rx_overruns\": %lu, "
           "\"tx_dropped\": %lu, \"tx_queued\": %u, \"door_address\": \"%02x\", \"door_address_changes\": %lu }",
           (unsigned long)(millis() / 1000), WiFi.RSSI(), (unsigned)ESP.getFreeHeap(), (unsigned long)isr_overruns,
           (unsigned long)tx_dropped, (unsigned)tx_q_count, (unsigned)last_bus_address, (unsigned long)bus_address_changes);
  client.publish(topic, msg);
}

void save_config() {
  DynamicJsonDocument json(1024);
  json["mqtt_server"] = mqtt_server;
  json["mqtt_port"] = mqtt_port;
  json["mqtt_topic"] = mqtt_topic;
  json["mqtt_client"] = mqtt_client;
  json["mqtt_user"] = mqtt_user;
  json["mqtt_pass"] = mqtt_pass;
  json["control4_command_topic"] = control4_command_topic;
  json["control4_notify_topic"] = control4_notify_topic;
  json["tcs_command_topic"] = tcs_command_topic;
  json["tz"] = tz;
  json["debug"] = debug_enabled;

  File configFile = LittleFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("failed to open config file for writing");
    return;
  }
  serializeJson(json, configFile);
  configFile.close();
}

static String html_escape(const char *v) {
  String o;
  for (; *v; v++) {
    switch (*v) {
      case '&': o += "&amp;"; break;
      case '<': o += "&lt;"; break;
      case '>': o += "&gt;"; break;
      case '"': o += "&quot;"; break;
      case '\'': o += "&#39;"; break;
      default: o += *v;
    }
  }
  return o;
}

void handle_root() {
  String h = "<html><body><h3>TCS Intercom " VERSION "</h3>";
  h += "<p>MQTT: " + String(client.connected() ? "connected" : "disconnected") + "</p>";
  h += "<p>Door address: 0x" + String(last_bus_address, HEX) + " (changes: " + String(bus_address_changes) +
       ", updated " + String((millis() - last_bus_address_ms) / 1000) + " s ago)</p>";
  h += "<p>Dropped RX frames: " + String(isr_overruns) + ", dropped TX frames: " + String(tx_dropped) + "</p>";
  h += "<form method='POST' action='/debug'><p>Debug logging: <b>" + String(debug_enabled ? "ON" : "OFF") + "</b> ";
  h += "<button name='on' value='" + String(debug_enabled ? 0 : 1) + "'>Turn " + String(debug_enabled ? "off" : "on") + "</button></p></form>";
  h += "<p><a href='/config'>Settings</a></p>";
  h += "</body></html>";
  web.send(200, "text/html", h);
}

void handle_debug() {
  debug_enabled = web.arg("on") == "1";
  save_config();
  web.sendHeader("Location", "/");
  web.send(303);
}

static void config_field(String &h, const char *label, const char *name, const char *value, bool password = false) {
  h += "<p>" + String(label) + "<br><input name='" + name + "' size='60' type='" + (password ? "password" : "text") +
       "' value='" + (password ? "" : html_escape(value)) + "'></p>";
}

void handle_config_get() {
  String h = "<html><body><h3>TCS Intercom settings</h3><form method='POST' action='/config'>";
  config_field(h, "MQTT server", "mqtt_server", mqtt_server);
  config_field(h, "MQTT port", "mqtt_port", mqtt_port);
  config_field(h, "MQTT topic", "mqtt_topic", mqtt_topic);
  config_field(h, "MQTT client name", "mqtt_client", mqtt_client);
  config_field(h, "MQTT user", "mqtt_user", mqtt_user);
  config_field(h, "MQTT password (blank = keep)", "mqtt_pass", "", true);
  config_field(h, "TCS command topic", "tcs_command_topic", tcs_command_topic);
  config_field(h, "Control4 command topic", "control4_command_topic", control4_command_topic);
  config_field(h, "Control4 notify topic", "control4_notify_topic", control4_notify_topic);
  config_field(h, "Timezone (POSIX TZ)", "tz", tz);
  h += "<p><label><input type='checkbox' name='debug' value='1'" + String(debug_enabled ? " checked" : "") + "> Debug logging</label></p>";
  h += "<button>Save and reboot</button></form><p><a href='/'>Back</a></p></body></html>";
  web.send(200, "text/html", h);
}

uint32_t restart_at = 0;

static void set_field(const char *name, char *dst, size_t size) {
  if (web.hasArg(name)) strlcpy(dst, web.arg(name).c_str(), size);
}

void handle_config_post() {
  set_field("mqtt_server", mqtt_server, sizeof(mqtt_server));
  set_field("mqtt_port", mqtt_port, sizeof(mqtt_port));
  set_field("mqtt_topic", mqtt_topic, sizeof(mqtt_topic));
  set_field("mqtt_client", mqtt_client, sizeof(mqtt_client));
  set_field("mqtt_user", mqtt_user, sizeof(mqtt_user));
  if (web.arg("mqtt_pass").length() > 0) set_field("mqtt_pass", mqtt_pass, sizeof(mqtt_pass));
  set_field("tcs_command_topic", tcs_command_topic, sizeof(tcs_command_topic));
  set_field("control4_command_topic", control4_command_topic, sizeof(control4_command_topic));
  set_field("control4_notify_topic", control4_notify_topic, sizeof(control4_notify_topic));
  if (web.arg("tz").length() > 0) set_field("tz", tz, sizeof(tz));
  debug_enabled = web.arg("debug") == "1";
  save_config();
  web.send(200, "text/html", "<html><body>Saved, rebooting... <a href='/'>Back</a></body></html>");
  restart_at = millis() + 1500;
}

// Door-related frames carry the address of the door station currently in use; follow it as it changes
void track_door_address(const CommandData &d) {
  switch (d.type) {
    case COMMAND_TYPE_DOOR_CALL:
    case COMMAND_TYPE_START_TALKING_DOOR_CALL:
    case COMMAND_TYPE_STOP_TALKING_DOOR_CALL:
    case COMMAND_TYPE_OPEN_DOOR:
    case COMMAND_TYPE_OPEN_DOOR_LONG:
    case COMMAND_TYPE_DOOR_OPENED:
    case COMMAND_TYPE_DOOR_CLOSED:
    case COMMAND_TYPE_END_OF_RINGTONE:
    case COMMAND_TYPE_END_OF_DOOR_READINESS:
    case COMMAND_TYPE_INITIALIZE_DOOR_STATION:
      break;
    default:
      return;
  }
  last_bus_address_ms = millis();
  if (d.address != last_bus_address) {
    DBG("Door address changed 0x%02x -> 0x%02x\n", (unsigned)last_bus_address, (unsigned)d.address);
    last_bus_address = d.address;
    bus_address_changes++;
  }
}

//callback notifying us of the need to save config
void saveConfigCallback() {
    Serial.println("Should save config");
    shouldSaveConfig = true;
}

// Accepts a hex string ("1a") or a JSON number
static uint32_t hex_field(JsonDocument &doc, const char *key) {
  uint32_t v = 0;
  JsonVariant f = doc[key];
  if (f.is<const char*>()) {
    sscanf(f.as<const char*>(), "%x", &v);
  } else if (f.is<unsigned long>()) {
    v = f.as<unsigned long>();
  }
  return v;
}

void callback(char *topic, byte *payload, unsigned int length) {
  JsonDocument doc;

  DBG("Received on %s: %.*s\n", topic, (int)length, (const char*)payload);

  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) {
    DBG("JSON error: #%d %s\n", (int)error.code(), error.c_str());
    return;
  }

  if (String(topic) == String(tcs_command_topic)) {
    uint32_t type = COMMAND_TYPE_UNKNOWN;
    if (doc["type"].is<const char*>())
      type = string_to_command_type(doc["type"].as<const char*>());

    if (type == COMMAND_TYPE_UNKNOWN) {
      DBG("Unknown tcs_command\n");
      return;
    }

    // address/payload/serial_number are hex strings (or numbers); a missing address uses the tracked door address
    uint32_t address = doc["address"].isNull() ? last_bus_address : hex_field(doc, "address");
    CommandData command_data = buildCommand(type, address, hex_field(doc, "payload"), hex_field(doc, "serial_number"));
    if (command_data.command > 0) {
      publish_tcs_command(mqtt_topic, command_data, true);
      queue_command(command_data.command, command_data.is_long);
    }
  } else if (String(topic) == String(control4_command_topic)) {
    CommandData command_data = buildCommand(COMMAND_TYPE_OPEN_DOOR, last_bus_address, 0, 0);
    DBG("Control4 open door, address 0x%02x\n", (unsigned)last_bus_address);
    if (command_data.command > 0) {
      publish_tcs_command(mqtt_topic, command_data, true);
      queue_command(command_data.command, command_data.is_long);
    }
  }
}

void setup()
{
    Serial.begin(115200);
    // LittleFS.format();
    
    pinMode(restorePin, INPUT_PULLUP);
    pinMode(inputPin,   INPUT);
    pinMode(outputPin,  OUTPUT);

    digitalWrite(outputPin, LOW);
    
    attachInterrupt(digitalPinToInterrupt(inputPin), analyzeCMD, CHANGE);
    //read configuration from FS json
    Serial.println("mounting FS...");

    if (LittleFS.begin()) {
        Serial.println("Read config file...");
        if (LittleFS.exists("/config.json")) {
            //file exists, reading and loading
            Serial.println("reading config file");
            std::unique_ptr<char[]> buf = read_file("/config.json");
            if (buf) {
                Serial.println("opened config file");
                DynamicJsonDocument json(1024);
                DeserializationError error = deserializeJson(json, buf.get());
                if (!error) {
                    Serial.println("parsed json");

                    load_str(json, "mqtt_server", mqtt_server, sizeof(mqtt_server));
                    load_str(json, "mqtt_port", mqtt_port, sizeof(mqtt_port));
                    load_str(json, "mqtt_topic", mqtt_topic, sizeof(mqtt_topic));
                    load_str(json, "mqtt_client", mqtt_client, sizeof(mqtt_client));
                    load_str(json, "mqtt_user", mqtt_user, sizeof(mqtt_user));
                    load_str(json, "mqtt_pass", mqtt_pass, sizeof(mqtt_pass));
                    load_str(json, "control4_command_topic", control4_command_topic, sizeof(control4_command_topic));
                    load_str(json, "control4_notify_topic", control4_notify_topic, sizeof(control4_notify_topic));
                    load_str(json, "tcs_command_topic", tcs_command_topic, sizeof(tcs_command_topic));
                    load_str(json, "tz", tz, sizeof(tz));
                    debug_enabled = json["debug"] | debug_enabled;
                } else {
                    Serial.println("failed to load json config");
                }
            }
        } else {
          Serial.println("File config.json does not exist!");
        }

        Serial.println("Read configured users...");
        if (LittleFS.exists("/tcs_users.json")) {
            //file exists, reading and loading
            Serial.println("reading users file");
            std::unique_ptr<char[]> buf = read_file("/tcs_users.json");
            if (buf) {
                Serial.println("opened users file");
                DeserializationError error = deserializeJson(intercomUsers, buf.get());
                if (!error) {
                    Serial.println("parsed json");
                } else {
                    Serial.printf("failed to load users: %s\n", error.c_str());
                }
            }
        } else {
          Serial.println("File tcs_users.json does not exist!");
        }


    }
    else
    {
        Serial.println("failed to mount FS");
    }
    //end read

    // The extra parameters to be configured (can be either global or just in the setup)
    // After connecting, parameter.getValue() will get you the configured value
    // id/name placeholder/prompt default length
    WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, sizeof(mqtt_server) - 1);
    WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, sizeof(mqtt_port) - 1);
    WiFiManagerParameter custom_mqtt_topic("topic", "mqtt topic", mqtt_topic, sizeof(mqtt_topic) - 1);
    WiFiManagerParameter custom_mqtt_client("client", "mqtt client", mqtt_client, sizeof(mqtt_client) - 1);
    WiFiManagerParameter custom_mqtt_user("user", "mqtt user", mqtt_user, sizeof(mqtt_user) - 1);
    WiFiManagerParameter custom_mqtt_pass("pass", "mqtt pass", mqtt_pass, sizeof(mqtt_pass) - 1);

    //WiFiManager
    //Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wifiManager;

    //set config save notify callback
    wifiManager.setSaveConfigCallback(saveConfigCallback);

    //set static ip
    // wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

    //add all your parameters here
    wifiManager.addParameter(&custom_mqtt_server);
    wifiManager.addParameter(&custom_mqtt_port);
    wifiManager.addParameter(&custom_mqtt_topic);
    wifiManager.addParameter(&custom_mqtt_client);
    wifiManager.addParameter(&custom_mqtt_user);
    wifiManager.addParameter(&custom_mqtt_pass);

    // Check the FLASH button
    delay(800);
    int restore=0;
    for (int i=0; i<1000; i++) {
    
      if (digitalRead(restorePin))
        restore++;
    }
    Serial.println("\n\nOutput: " + String(restore) + "\n\n");
    if (restore==0) {
      Serial.println("Restoring parameters, restore=" + String(restore));
      //reset settings - for testing
      wifiManager.resetSettings();
    }

    //set minimu quality of signal so it ignores AP's under that quality
    //defaults to 8%
    //wifiManager.setMinimumSignalQuality();

    //sets timeout until configuration portal gets turned off
    //useful to make it all retry or go to sleep
    //in seconds
    //wifiManager.setTimeout(120);

    //fetches ssid and pass and tries to connect
    //if it does not connect it starts an access point with the specified name
    //here  "AutoConnectAP"
    //and goes into a blocking loop awaiting configuration
    if (!wifiManager.autoConnect("TCS  ", "intercom"))
    {
        Serial.println("failed to connect and hit timeout");
        delay(3000);
        //reset and try again, or maybe put it to deep sleep
        ESP.reset();
        delay(5000);
    }

    //if you get here you have connected to the WiFi
    Serial.println("connected...yeey :)");

    //read updated parameters
    strlcpy(mqtt_server, custom_mqtt_server.getValue(), sizeof(mqtt_server));
    strlcpy(mqtt_port, custom_mqtt_port.getValue(), sizeof(mqtt_port));
    strlcpy(mqtt_topic, custom_mqtt_topic.getValue(), sizeof(mqtt_topic));
    strlcpy(mqtt_client, custom_mqtt_client.getValue(), sizeof(mqtt_client));
    strlcpy(mqtt_user, custom_mqtt_user.getValue(), sizeof(mqtt_user));
    strlcpy(mqtt_pass, custom_mqtt_pass.getValue(), sizeof(mqtt_pass));

    //save the custom parameters to FS
    if (shouldSaveConfig)
    {
        Serial.println("saving config");
        save_config();
    }

    configTime(tz, ntpServer); // non-blocking; time syncs in the background

    web.on("/", HTTP_GET, handle_root);
    web.on("/debug", HTTP_POST, handle_debug);
    web.on("/config", HTTP_GET, handle_config_get);
    web.on("/config", HTTP_POST, handle_config_post);
    web.begin();

    Serial.println("local ip");
    Serial.println(WiFi.localIP());

    Serial.print("MQTT Server ");
    Serial.println(mqtt_server);
    Serial.print("MQTT port ");
    Serial.println(atoi(mqtt_port));

    client.setServer(mqtt_server, atoi(mqtt_port));
    client.setBufferSize(512);
    client.setCallback(callback);

    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
        {
            type = "sketch";
        }
        else
        { // U_FS
            type = "filesystem";
        }

        // NOTE: if updating FS this would be the place to unmount FS using FS.end()
        Serial.println("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
        {
            Serial.println("Auth Failed");
        }
        else if (error == OTA_BEGIN_ERROR)
        {
            Serial.println("Begin Failed");
        }
        else if (error == OTA_CONNECT_ERROR)
        {
            Serial.println("Connect Failed");
        }
        else if (error == OTA_RECEIVE_ERROR)
        {
            Serial.println("Receive Failed");
        }
        else if (error == OTA_END_ERROR)
        {
            Serial.println("End Failed");
        }
    });
    ArduinoOTA.begin();
}

// Appends formatted text to buf, truncating instead of overflowing
static void append(char *buf, size_t size, const char *fmt, ...) {
  size_t len = strlen(buf);
  if (len >= size - 1) return;
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf + len, size - len, fmt, args);
  va_end(args);
}

void publish_tcs_command(char *topic, CommandData data, bool direction) {
  char mqtt_time[12];
  char msg[320];

  current_time(mqtt_time);
  snprintf(msg, sizeof(msg), "{ \"ts\": \"%lu\", \"time\": \"%s\", \"direction\": \"%s\"",
           (unsigned long)time(nullptr), mqtt_time, direction ? "output" : "input");

  if (data.command) append(msg, sizeof(msg), ", \"command\": \"%08x\"", (unsigned)data.command);
  if (data.type) append(msg, sizeof(msg), ", \"type\": \"%s\"", command_type_to_string(data.type));
  if (data.serial_number) append(msg, sizeof(msg), ", \"serial_number\": \"%08x\"", (unsigned)data.serial_number);
  append(msg, sizeof(msg), ", \"address\": \"%02x\", \"payload\": \"%x\" }", data.address, (unsigned)data.payload);

  client.publish(topic, msg);
}

void loop() {
    static uint32_t last_wifi_try = 0;
    static uint32_t last_mqtt_try = 0;
    static uint32_t last_status = 0;
    uint32_t now = millis();

    // Never block: the bus must keep being sniffed while WiFi/MQTT recover
    if (WiFi.status() != WL_CONNECTED) {
      if (now - last_wifi_try > 10000) {
        last_wifi_try = now;
        DBG("WiFi down, reconnecting\n");
        WiFi.reconnect();
      }
    } else if (!client.connected()) {
      if (now - last_mqtt_try > 5000) {
        last_mqtt_try = now;
        connectToMqtt();
      }
    } else {
      client.loop();
      if (now - last_status > 60000) {
        last_status = now;
        publish_status();
      }
    }
    ArduinoOTA.handle();
    web.handleClient();
    tx_service();
    if (restart_at && (int32_t)(now - restart_at) >= 0) ESP.restart();

    // Take a consistent snapshot of the frame; the ISR may overwrite it at any time
    noInterrupts();
    bool ready = cmdReady;
    uint32_t cmd = CMD;
    uint8_t len = lengthCMD;
    cmdReady = false;
    interrupts();

    if (ready) {
      CommandData cmd_data = parseCommand(cmd, len);
      track_door_address(cmd_data);

      DBG("RX %s %s addr=0x%02x\n", cmd_data.command_hex.c_str(), command_type_to_string(cmd_data.type), cmd_data.address);
      publish_tcs_command(mqtt_topic, cmd_data, false);

      if (cmd_data.serial_number == my_serial_number) {
        if (cmd_data.type == COMMAND_TYPE_INTERNAL_CALL)
          control4_notify(true, false, cmd_data);
        else if (cmd_data.type == COMMAND_TYPE_DOOR_CALL)
          control4_notify(false, true, cmd_data);
      }
    }
}
