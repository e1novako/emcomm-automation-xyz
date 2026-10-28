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
#include <algorithm>
#include <limits.h>
#include <EEPROM.h>
#include <libb64/cdecode.h>

#define VERSION "1.1.2"

void publish_tcs_command(char *topic, CommandData data, bool direction);
void mqtt_output(char *format, ...);

char ntp_server[48] = "pool.ntp.org";
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
char wifi_ssid[33] = "";           // empty = use WiFiManager / stored credentials
char admin_pass[64] = "";          // WiFi password; empty = use the WiFi password stored in the device. Also the GUI and OTA password

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
volatile uint32_t last_edge_us = 0; // last bus edge seen by the RX ISR (carrier sense)
uint32_t tx_watchdog_trips = 0;
uint32_t min_free_heap = 0xFFFFFFFF;
uint32_t tx_forced = 0;
#define TX_IDLE_US     10000  // bus must be quiet this long before we send
#define TX_MAX_WAIT_MS 300    // then send anyway
#define TX_WATCHDOG_MS 500    // longest legal frame is ~200 ms
volatile bool tx_level = false;

// Looks up a user name by TCS serial number (tcs_addr is hex in tcs_users.json)
bool get_tcsuser_name(uint32_t serial, char *name_out, size_t size, int *flat = nullptr) {
  for (JsonObject elem : intercomUsers.as<JsonArray>()) {
    const char *name = elem["name"];
    const char *addr = elem["tcs_addr"];
    if (!name || !addr) continue;
    if ((uint32_t)strtoul(addr, nullptr, 16) == serial) {
      strlcpy(name_out, name, size);
      if (flat) *flat = elem["flat_no"] | 0;
      return true;
    }
  }
  return false;
}

JsonObject find_user(uint32_t serial) {
  for (JsonObject elem : intercomUsers.as<JsonArray>()) {
    const char *addr = elem["tcs_addr"];
    if (addr && (uint32_t)strtoul(addr, nullptr, 16) == serial) return elem;
  }
  return JsonObject();
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
    last_edge_us = usNow;
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
static void count_unlock();

void tx_service() {
  static bool started = false;
  static uint32_t end_ms = 0;
  static uint32_t start_ms = 0;
  static uint32_t wait_since = 0;

  if (started && tx_busy && millis() - start_ms > TX_WATCHDOG_MS) {
    timer1_disable();
    GPOC = (1 << outputPin);
    tx_busy = false;
    tx_watchdog_trips++;
    DBG("TX watchdog: stuck frame aborted\n");
  }
  if (started && !tx_busy) {
    started = false;
    end_ms = millis();
    last_edge_us = micros();
  }
  if (started || tx_q_count == 0 || millis() - end_ms < TX_GAP_MS) {
    wait_since = 0;
    return;
  }

  // Carrier sense: let a frame in progress from another device finish first
  noInterrupts();
  uint32_t quiet_us = micros() - last_edge_us;
  interrupts();
  if (quiet_us < TX_IDLE_US) {
    if (!wait_since) wait_since = millis() | 1;
    if (millis() - wait_since < TX_MAX_WAIT_MS) return;
    tx_forced++;
    DBG("TX: bus busy for %d ms, sending anyway\n", TX_MAX_WAIT_MS);
  }
  wait_since = 0;

  TxFrame f = tx_queue[tx_q_head];
  tx_q_head = (tx_q_head + 1) % TX_QUEUE_SIZE;
  tx_q_count--;
  DBG("TX %08x (%s)\n", (unsigned)f.cmd, f.is_long ? "long" : "short");
  uint32_t sent = parseCommand(f.cmd, f.is_long).type;
  if (sent == COMMAND_TYPE_OPEN_DOOR || sent == COMMAND_TYPE_OPEN_DOOR_LONG) count_unlock();
  started = true;
  start_ms = millis();
  tx_start(f.cmd, f.is_long);
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

// ---- Configuration in NVRAM (EEPROM emulation: its own flash sector, survives firmware and FS updates) ----
#define NV_SIZE  2048
#define NV_MAGIC 0x31534354UL // "TCS1"
#define NV_HDR   12           // magic(4) len(2) reserved(2) crc(4)

static uint32_t crc32_buf(const uint8_t *d, size_t n) {
  uint32_t c = 0xFFFFFFFF;
  for (size_t i = 0; i < n; i++) {
    c ^= d[i];
    for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320 & -(c & 1));
  }
  return ~c;
}

static bool nv_read(String &out) {
  uint32_t magic, crc;
  uint16_t len;
  EEPROM.get(0, magic);
  EEPROM.get(4, len);
  EEPROM.get(8, crc);
  if (magic != NV_MAGIC || len == 0 || len > NV_SIZE - NV_HDR) return false;

  std::unique_ptr<uint8_t[]> b(new uint8_t[len]);
  for (uint16_t i = 0; i < len; i++) b[i] = EEPROM.read(NV_HDR + i);
  if (crc32_buf(b.get(), len) != crc) return false;

  out = "";
  out.concat((const char*)b.get(), len);
  return true;
}

// Write to a temp file then rename, so a power loss never leaves a half-written file
static bool write_file_atomic(const char *path, const String &data) {
  String tmp = String(path) + ".tmp";
  File f = LittleFS.open(tmp, "w");
  if (!f) return false;
  size_t n = f.write((const uint8_t*)data.c_str(), data.length());
  f.close();
  if (n != data.length()) { LittleFS.remove(tmp); return false; }
  return LittleFS.rename(tmp, path);
}

// ---- Ring log (last 10 rings) and automatic learning of new serial numbers ----
#define RING_LOG_SIZE 10
#define RING_DEDUPE_S 5 // a ring repeats its frame; count it once
struct RingEntry { time_t ts; uint32_t serial; uint8_t type; uint32_t uptime_s; };
static RingEntry ring_log[RING_LOG_SIZE];
static uint8_t ring_head = 0; // next slot to write
static uint8_t ring_count = 0;

#define MAX_USERS 40
#define MAX_USER_ROWS MAX_USERS

// Door-unlock counter, persisted in /stats.json so it survives reboot/power loss
static uint32_t door_unlocks = 0;
static uint32_t last_unlock_ms = 0;

static void save_stats() {
  write_file_atomic("/stats.json", String("{\"unlocks\":") + door_unlocks + "}");
}

static void load_stats() {
  std::unique_ptr<char[]> buf = read_file("/stats.json");
  if (!buf) return;
  JsonDocument doc;
  if (!deserializeJson(doc, buf.get())) door_unlocks = doc["unlocks"] | 0u;
}

// An unlock command may be repeated on the bus; count it once per 2 s
static void count_unlock() {
  if (last_unlock_ms && millis() - last_unlock_ms < 2000) return;
  last_unlock_ms = millis() | 1;
  door_unlocks++;
  save_stats();
}

static bool save_users_doc() {
  String out;
  serializeJson(intercomUsers, out);
  return out.length() <= 7000 && write_file_atomic("/tcs_users.json", out);
}

static void ring_type_name(uint8_t t, char *out, size_t n) { strlcpy(out, t == COMMAND_TYPE_DOOR_CALL ? "door" : "internal", n); }

// Rings are kept across reboots in /rings.json (oldest first); only written when a new ring is logged
static void rings_to_json(JsonDocument &doc) {
  JsonArray arr = doc.to<JsonArray>();
  for (int i = ring_count - 1; i >= 0; i--) {
    const RingEntry &e = ring_log[(ring_head + RING_LOG_SIZE - 1 - i) % RING_LOG_SIZE];
    char serial[10], type[10], name[40] = "";
    snprintf(serial, sizeof(serial), "%06x", (unsigned)e.serial);
    ring_type_name(e.type, type, sizeof(type));
    get_tcsuser_name(e.serial, name, sizeof(name));
    JsonObject o = arr.add<JsonObject>();
    o["ts"] = (uint32_t)e.ts;
    o["type"] = type;
    o["serial"] = serial;
    o["name"] = name;
  }
}

static void save_rings() {
  JsonDocument doc;
  rings_to_json(doc);
  String out;
  serializeJson(doc, out);
  write_file_atomic("/rings.json", out);
}

static void load_rings() {
  std::unique_ptr<char[]> buf = read_file("/rings.json");
  if (!buf) return;
  JsonDocument doc;
  if (deserializeJson(doc, buf.get())) return;
  for (JsonObject o : doc.as<JsonArray>()) {
    const char *serial = o["serial"], *type = o["type"];
    if (!serial || !type) continue;
    ring_log[ring_head] = { (time_t)(o["ts"] | 0u), (uint32_t)strtoul(serial, nullptr, 16),
                            (uint8_t)(strcmp(type, "door") == 0 ? COMMAND_TYPE_DOOR_CALL : COMMAND_TYPE_INTERNAL_CALL), 0xFFFFFFF0u };
    ring_head = (ring_head + 1) % RING_LOG_SIZE;
    if (ring_count < RING_LOG_SIZE) ring_count++;
  }
}

static bool add_unknown_user(uint32_t serial) {
  if (!find_user(serial).isNull()) return false;
  JsonArray arr = intercomUsers.as<JsonArray>();
  if (arr.isNull()) arr = intercomUsers.to<JsonArray>();
  if (arr.size() >= MAX_USERS) { DBG("user list full, not adding %06x\n", (unsigned)serial); return false; }
  char addr[10];
  snprintf(addr, sizeof(addr), "%06x", (unsigned)serial);
  JsonObject o = arr.add<JsonObject>();
  o["tcs_addr"] = addr;
  o["name"] = "n/a";
  o["flat_no"] = 0;
  if (intercomUsers.overflowed()) { arr.remove(arr.size() - 1); DBG("users buffer full\n"); return false; }
  DBG("new user %06x added\n", (unsigned)serial);
  return true;
}

static void record_ring(uint32_t serial, uint8_t type) {
  time_t now = time(nullptr);
  uint32_t up = millis() / 1000;
  if (ring_count) {
    RingEntry &l = ring_log[(ring_head + RING_LOG_SIZE - 1) % RING_LOG_SIZE];
    if (l.serial == serial && l.type == type && up - l.uptime_s < RING_DEDUPE_S) { l.uptime_s = up; return; }
  }
  ring_log[ring_head] = { now, serial, type, up };
  ring_head = (ring_head + 1) % RING_LOG_SIZE;
  if (ring_count < RING_LOG_SIZE) ring_count++;
  bool users_changed = add_unknown_user(serial);
  if (type == COMMAND_TYPE_DOOR_CALL) {
    // per-user door ring counter, kept in tcs_users.json so it survives reboot/power loss
    JsonObject u = find_user(serial);
    if (!u.isNull()) {
      u["rings"] = (u["rings"] | 0u) + 1;
      u["last_ring"] = (uint32_t)now;
      users_changed = !intercomUsers.overflowed();
    }
  }
  if (users_changed) save_users_doc();
  save_rings();
}

static bool nv_write(const String &json) {
  size_t len = json.length();
  if (len == 0 || len > NV_SIZE - NV_HDR) return false;

  uint32_t magic = NV_MAGIC;
  uint32_t crc = crc32_buf((const uint8_t*)json.c_str(), len);
  uint16_t l16 = len;
  EEPROM.put(0, magic);
  EEPROM.put(4, l16);
  EEPROM.put(8, crc);
  for (size_t i = 0; i < len; i++) EEPROM.write(NV_HDR + i, json[i]);
  return EEPROM.commit();
}

static void config_to_json(JsonDocument &json) {
  char serial[12];
  snprintf(serial, sizeof(serial), "%x", (unsigned)my_serial_number);

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
  json["ntp_server"] = ntp_server;
  json["debug"] = debug_enabled;
  json["serial_number"] = serial;
  json["wifi_ssid"] = wifi_ssid;
  json["password"] = admin_pass;
}

// Applies any keys present; missing or invalid ones keep their current value
// The one password: used to join WiFi and to protect the web GUI and OTA
static String effective_pass() {
  return admin_pass[0] ? String(admin_pass) : WiFi.psk();
}

static void apply_config(JsonDocument &json) {
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
  load_str(json, "ntp_server", ntp_server, sizeof(ntp_server));
  load_str(json, "wifi_ssid", wifi_ssid, sizeof(wifi_ssid));
  if (json["debug"].is<bool>()) debug_enabled = json["debug"];

  const char *sn = json["serial_number"] | (const char*)nullptr;
  if (sn) sscanf(sn, "%x", &my_serial_number);

  // WPA2 needs 8..63 characters; empty means "use the stored WiFi password"
  const char *pw = json["password"] | (const char*)nullptr;
  if (pw && (!pw[0] || (strlen(pw) >= 8 && strlen(pw) <= 63))) strlcpy(admin_pass, pw, sizeof(admin_pass));
}

void save_config() {
  JsonDocument json;
  config_to_json(json);
  String out;
  serializeJson(json, out);
  // The flash file copy is written first (atomic); an interrupted NVRAM write is recovered from it at boot
  if (!write_file_atomic("/config.json", out)) Serial.println("failed to write config.json copy");
  if (!nv_write(out)) Serial.println("failed to write config to NVRAM");
}

// NVRAM first; if it is empty or corrupt, recover from the /config.json flash copy (also the legacy
// location) and rewrite NVRAM
static void load_config() {
  String s;
  if (nv_read(s)) {
    JsonDocument json;
    if (!deserializeJson(json, s)) {
      apply_config(json);
      Serial.println("config loaded from NVRAM");
      return;
    }
  }

  std::unique_ptr<char[]> buf = read_file("/config.json");
  if (buf) {
    JsonDocument json;
    if (!deserializeJson(json, buf.get())) {
      apply_config(json);
      Serial.println("NVRAM empty/invalid, config recovered from config.json");
      String out;
      serializeJson(json, out);
      nv_write(out);
      return;
    }
  }
  Serial.println("no stored config, using defaults");
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

// HTTP basic auth: any user name, only the password is checked
static bool auth() {
  String pass = effective_pass();
  if (pass.length() == 0) return true; // open network and no password set

  String hdr = web.header("Authorization");
  if (hdr.startsWith("Basic ")) {
    String b64 = hdr.substring(6);
    char decoded[160];
    if (b64.length() < 200) {
      int n = base64_decode_chars(b64.c_str(), b64.length(), decoded);
      decoded[n] = 0;
      const char *colon = strchr(decoded, ':');
      if (colon && pass == (colon + 1)) return true;
    }
  }
  web.requestAuthentication();
  return false;
}

// ---- Web GUI: pages share one layout with a pill-button menu on top ----
static const char CSS[] =
  "<style>body{font-family:sans-serif;width:1000px;max-width:100%;box-sizing:border-box;margin:16px auto;padding:0 8px;text-align:center}"
  "h3{text-align:center;font-size:32px;margin:8px 0 16px}"
  "nav{margin-bottom:16px;text-align:center}"
  "table{margin:0 auto}table.c{width:100%;table-layout:fixed;border-collapse:collapse}table.c th,table.c td{border:1px solid #ccc;padding:6px;text-align:center}table.c th{background:#e3e6ea}input{text-align:center}"
  "nav a,button{display:inline-block;padding:8px 18px;margin:2px;border:0;border-radius:20px;background:#e3e6ea;color:#222;text-decoration:none;font-size:14px;cursor:pointer}"
  "nav a.on,button{background:#1a73e8;color:#fff}"
  "input[type=text],input[type=password]{width:420px;max-width:95%;padding:6px;border:1px solid #bbb;border-radius:8px}"
  "</style>";

static String uptime_str() {
  uint32_t t = millis() / 1000;
  char b[32];
  snprintf(b, sizeof(b), "%ud %02uh %02um %02us", (unsigned)(t / 86400), (unsigned)(t / 3600 % 24), (unsigned)(t / 60 % 60), (unsigned)(t % 60));
  return b;
}

static String page_begin(const char *active) {
  static const char *items[][2] = { {"/", "Home"}, {"/status", "Access"}, {"/rings", "Statistics"}, {"/users", "Users"}, {"/wifi", "WiFi"}, {"/mqtt", "MQTT"}, {"/time", "Time"}, {"/debug", "Debug"}, {"/backup", "Backup/Restore"} };
  String h = "<html><head><meta name='viewport' content='width=device-width,initial-scale=1'><title>TCS Intercom</title>";
  // Only the status page auto-refreshes; form pages must not, it would discard what is being typed
  if (strcmp(active, "/") == 0 || strcmp(active, "/status") == 0) h += "<meta http-equiv='refresh' content='10'>";
  h += CSS;
  h += "</head><body><h3>TCS Intercom " VERSION "</h3><p style='text-align:center;margin:-8px 0 16px;color:#555'>Uptime: " + uptime_str() + "</p><nav>";
  if (strcmp(active, "/") == 0) {
    h += "<a href='/status'>Config</a></nav>";
    return h;
  }
  for (auto &it : items) {
    h += "<a href='" + String(it[0]) + "'" + (strcmp(it[0], active) == 0 ? " class='on'" : "") + ">" + it[1] + "</a>";
  }
  h += "</nav>";
  return h;
}

static void send_page(const char *active, const String &body) {
  web.send(200, "text/html", page_begin(active) + body + "</body></html>");
}

static void config_field(String &h, const char *label, const char *name, const char *value, bool password = false) {
  h += "<p>" + String(label) + "<br><input name='" + name + "' type='" + (password ? "password" : "text") +
       "' value='" + (password ? "" : html_escape(value)) + "'></p>";
}

uint32_t restart_at = 0;

static void set_field(const char *name, char *dst, size_t size) {
  if (web.hasArg(name)) strlcpy(dst, web.arg(name).c_str(), size);
}

static void save_and_reboot() {
  save_config();
  send_page("", "<p>Saved, rebooting...</p><p><a href='/'>Back to status</a></p>");
  restart_at = millis() + 1500;
}

static void fmt_ts(time_t t, char *out, size_t n) {
  if (t > 1600000000) {
    struct tm tm;
    localtime_r(&t, &tm);
    strftime(out, n, "%Y-%m-%d %H:%M:%S", &tm);
  } else {
    strlcpy(out, t ? "(no time)" : "-", n);
  }
}

void handle_rings_page() {
  if (!auth()) return;
  String h = "<h4>Door rings per device</h4><table class='c'><tr><th style='width:15%'>Flat No.</th><th style='width:40%'>Surname / Name</th><th style='width:15%'>Ring No.</th><th style='width:30%'>Last ring</th></tr>";
  // most rings first; ties by flat number, users without a flat (0) last
  JsonObject sorted[MAX_USERS];
  int cnt = 0;
  for (JsonObject u : intercomUsers.as<JsonArray>()) if (cnt < MAX_USERS) sorted[cnt++] = u;
  std::stable_sort(sorted, sorted + cnt, [](JsonObject a, JsonObject b) {
    uint32_t ra = a["rings"] | 0u, rb = b["rings"] | 0u;
    if (ra != rb) return ra > rb;
    int fa = a["flat_no"] | 0, fb = b["flat_no"] | 0;
    if (fa <= 0) fa = INT_MAX;
    if (fb <= 0) fb = INT_MAX;
    return fa < fb;
  });
  for (int i = 0; i < cnt; i++) {
    JsonObject u = sorted[i];
    char ts[24];
    fmt_ts((time_t)(u["last_ring"] | 0u), ts, sizeof(ts));
    int flat = u["flat_no"] | 0;
    String who = html_escape(u["name"] | "");
    if (who.length() == 0 || who == "n/a") who = u["tcs_addr"] | "";
    h += "<tr><td>" + (flat > 0 ? String(flat) : String("-")) + "</td><td>" + who + "</td><td>" +
         String(u["rings"] | 0u) + "</td><td>" + ts + "</td></tr>";
  }
  h += "</table>";
  send_page("/rings", h);
}

void handle_status() {
  if (!auth()) return;
  String h = "<h4 style='text-align:center'>Last rings</h4><table class='c' cellpadding='4'><tr><th style='width:16.66%'>Date/Time</th><th style='width:16.66%'>Type</th><th style='width:16.66%'>Serial</th><th style='width:50%'>Surname / Name</th></tr>";
  if (!ring_count) h += "<tr><td colspan='4'>none yet</td></tr>";
  for (int i = 0; i < ring_count; i++) {
    const RingEntry &e = ring_log[(ring_head + RING_LOG_SIZE - 1 - i) % RING_LOG_SIZE];
    char ts[24] = "(no time)", name[40] = "";
    if (e.ts > 1600000000) {
      struct tm t;
      localtime_r(&e.ts, &t);
      strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &t);
    }
    int flat = 0;
    get_tcsuser_name(e.serial, name, sizeof(name), &flat);
    String who = html_escape(name);
    if (flat > 0) who = String(flat) + ". " + who;
    h += "<tr><td>" + String(ts) + "</td><td>" + (e.type == COMMAND_TYPE_DOOR_CALL ? "Door" : "Internal") +
         "</td><td>" + String(e.serial, HEX) + "</td><td>" + who + "</td></tr>";
  }
  h += "</table>";
  send_page("/status", h);
}

// Public (no password); the unlock action itself still requires it
void handle_home() {
  String h = "<form method='POST' action='/unlock' style='margin-top:200px'><button style='width:33%;min-width:300px;padding:40px 0;font-size:40px;border-radius:60px'>Unlock door</button></form>";
  h += "<p style='margin-top:32px'>Door unlocked: <b>" + String(door_unlocks) + "</b></p>";
  send_page("/", h);
}

void handle_unlock() {
  if (!auth()) return;
  CommandData d = buildCommand(COMMAND_TYPE_OPEN_DOOR, last_bus_address, 0, 0);
  DBG("Web unlock, address 0x%02x\n", (unsigned)last_bus_address);
  if (d.command > 0) {
    publish_tcs_command(mqtt_topic, d, true);
    queue_command(d.command, d.is_long);
  }
  web.sendHeader("Location", "/");
  web.send(303);
}

void handle_debug_page() {
  if (!auth()) return;
  String h = "<p>Reset reason: " + html_escape(ESP.getResetReason().c_str()) + "</p>";
  h += "<p>WiFi: " + html_escape(WiFi.SSID().c_str()) + " (" + WiFi.localIP().toString() + ", " + String(WiFi.RSSI()) + " dBm)</p>";
  h += "<p>MQTT: " + String(client.connected() ? "connected" : "disconnected") + "</p>";
  h += "<p>Door address: 0x" + String(last_bus_address, HEX) + " (changes: " + String(bus_address_changes) +
       ", updated " + String((millis() - last_bus_address_ms) / 1000) + " s ago)</p>";
  h += "<p>Dropped RX frames: " + String(isr_overruns) + ", dropped TX frames: " + String(tx_dropped) +
       ", TX sent into busy bus: " + String(tx_forced) + ", TX watchdog trips: " + String(tx_watchdog_trips) + "</p>";
  h += "<form method='POST' action='/debug'><p>Debug logging: <b>" + String(debug_enabled ? "ON" : "OFF") + "</b> ";
  h += "<button name='on' value='" + String(debug_enabled ? 0 : 1) + "'>Turn " + String(debug_enabled ? "off" : "on") + "</button></p></form>";
  h += "<h4>Memory</h4><table cellpadding='4'>";
  auto row = [&](const char *k, String v) { h += "<tr><td>" + String(k) + "</td><td>" + v + "</td></tr>"; };
  uint32_t heap = ESP.getFreeHeap();
  row("Free heap", String(heap) + " bytes (lowest since boot: " + String(min_free_heap) + ")");
  row("Largest free block", String(ESP.getMaxFreeBlockSize()) + " bytes");
  row("Heap fragmentation", String(ESP.getHeapFragmentation()) + " %");
  row("Free stack (cont)", String(ESP.getFreeContStack()) + " bytes");
  row("Sketch", String(ESP.getSketchSize()) + " used, " + String(ESP.getFreeSketchSpace()) + " free for OTA");
  row("Flash chip", String(ESP.getFlashChipRealSize()) + " bytes");
  FSInfo fs;
  if (LittleFS.info(fs)) row("LittleFS", String(fs.usedBytes) + " / " + String(fs.totalBytes) + " bytes used");
  row("Users buffer", String(intercomUsers.memoryUsage()) + " / 8192 bytes");
  h += "</table>";
  send_page("/debug", h);
}

void handle_debug() {
  if (!auth()) return;
  debug_enabled = web.arg("on") == "1";
  save_config();
  web.sendHeader("Location", "/debug");
  web.send(303);
}

void handle_wifi_get() {
  if (!auth()) return;
  String h = "<form method='POST' action='/wifi'>";
  config_field(h, "WiFi SSID (blank = WiFiManager / stored network)", "wifi_ssid", wifi_ssid);
  config_field(h, "WiFi password, also the GUI and OTA password, 8-63 chars (blank = keep)", "password", "", true);
  h += "<p><label><input type='checkbox' name='use_wifi_pass' value='1'> Use the WiFi password already stored in the device</label></p>";
  h += "<button>Save and reboot</button></form>";
  send_page("/wifi", h);
}

void handle_wifi_post() {
  if (!auth()) return;

  String pw = web.arg("password");
  if (pw.length() > 0 && (pw.length() < 8 || pw.length() > 63)) {
    web.send(400, "text/plain", "Password must be 8-63 characters");
    return;
  }
  if (web.arg("wifi_ssid").length() > 32) {
    web.send(400, "text/plain", "SSID must be at most 32 characters");
    return;
  }
  bool use_stored = web.arg("use_wifi_pass") == "1";
  if (web.arg("wifi_ssid").length() > 0 && web.arg("wifi_ssid") != WiFi.SSID() && pw.length() == 0 && (use_stored || admin_pass[0] == 0)) {
    web.send(400, "text/plain", "Enter the WiFi password for the new SSID");
    return;
  }

  set_field("wifi_ssid", wifi_ssid, sizeof(wifi_ssid));
  if (pw.length() > 0) strlcpy(admin_pass, pw.c_str(), sizeof(admin_pass));
  else if (use_stored) admin_pass[0] = 0;
  save_and_reboot();
}

void handle_mqtt_get() {
  if (!auth()) return;
  char serial[12];
  snprintf(serial, sizeof(serial), "%x", (unsigned)my_serial_number);

  String h = "<form method='POST' action='/mqtt'>";
  config_field(h, "MQTT server", "mqtt_server", mqtt_server);
  config_field(h, "MQTT port", "mqtt_port", mqtt_port);
  config_field(h, "MQTT topic", "mqtt_topic", mqtt_topic);
  config_field(h, "MQTT client name", "mqtt_client", mqtt_client);
  config_field(h, "MQTT user", "mqtt_user", mqtt_user);
  config_field(h, "MQTT password (blank = keep)", "mqtt_pass", "", true);
  config_field(h, "TCS command topic", "tcs_command_topic", tcs_command_topic);
  config_field(h, "Control4 command topic", "control4_command_topic", control4_command_topic);
  config_field(h, "Control4 notify topic", "control4_notify_topic", control4_notify_topic);
  config_field(h, "This device's TCS serial number (hex)", "serial_number", serial);
  h += "<button>Save and reboot</button></form>";
  send_page("/mqtt", h);
}

void handle_mqtt_post() {
  if (!auth()) return;
  set_field("mqtt_server", mqtt_server, sizeof(mqtt_server));
  set_field("mqtt_port", mqtt_port, sizeof(mqtt_port));
  set_field("mqtt_topic", mqtt_topic, sizeof(mqtt_topic));
  set_field("mqtt_client", mqtt_client, sizeof(mqtt_client));
  set_field("mqtt_user", mqtt_user, sizeof(mqtt_user));
  if (web.arg("mqtt_pass").length() > 0) set_field("mqtt_pass", mqtt_pass, sizeof(mqtt_pass));
  set_field("tcs_command_topic", tcs_command_topic, sizeof(tcs_command_topic));
  set_field("control4_command_topic", control4_command_topic, sizeof(control4_command_topic));
  set_field("control4_notify_topic", control4_notify_topic, sizeof(control4_notify_topic));
  if (web.arg("serial_number").length() > 0) sscanf(web.arg("serial_number").c_str(), "%x", &my_serial_number);
  save_and_reboot();
}

void handle_time_get() {
  if (!auth()) return;
  time_t now = time(nullptr);
  char buf[24] = "not synced yet";
  if (now > 1000000000) {
    struct tm t;
    localtime_r(&now, &t);
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &t);
  }
  String h = "<p>Current time: <b>" + String(buf) + "</b></p><form method='POST' action='/time'>";
  config_field(h, "NTP server", "ntp_server", ntp_server);
  config_field(h, "Timezone (POSIX TZ, e.g. CET-1CEST,M3.5.0,M10.5.0/3)", "tz", tz);
  h += "<button>Save and reboot</button></form>";
  send_page("/time", h);
}

void handle_time_post() {
  if (!auth()) return;
  if (web.arg("ntp_server").length() > 0) set_field("ntp_server", ntp_server, sizeof(ntp_server));
  if (web.arg("tz").length() > 0) set_field("tz", tz, sizeof(tz));
  save_and_reboot();
}

void handle_backup_page() {
  if (!auth()) return;
  String h = "<p><a href='/backup/download'><button type='button'>Download backup</button></a> (contains passwords)</p>";
  h += "<form method='POST' action='/restore' enctype='multipart/form-data'><p><input type='file' name='file'> <button>Restore and reboot</button></p></form>";
  h += "<h4>Users</h4><p><a href='/users/download'><button type='button'>Export users</button></a></p>";
  h += "<form method='POST' action='/users/restore' enctype='multipart/form-data'><p><input type='file' name='file'> <button>Import users</button></p></form>";
  h += "<h4>Last rings</h4><p><a href='/rings/download'><button type='button'>Export last rings</button></a></p>";
  h += "<h4>Device</h4><form method='POST' action='/reboot'><p><button>Reboot only</button></p></form>";
  send_page("/backup", h);
}

void handle_users_download() {
  if (!auth()) return;
  String out;
  serializeJsonPretty(intercomUsers, out);
  web.sendHeader("Content-Disposition", "attachment; filename=tcs-users.json");
  web.send(200, "application/json", out);
}

void handle_rings_download() {
  if (!auth()) return;
  JsonDocument doc;
  rings_to_json(doc);
  String out;
  serializeJsonPretty(doc, out);
  web.sendHeader("Content-Disposition", "attachment; filename=tcs-rings.json");
  web.send(200, "application/json", out);
}

static String users_buf;
static bool users_overflow = false;

void handle_users_upload() {
  HTTPUpload &u = web.upload();
  if (u.status == UPLOAD_FILE_START) {
    users_buf = "";
    users_overflow = false;
  } else if (u.status == UPLOAD_FILE_WRITE) {
    if (users_buf.length() + u.currentSize > 8000) users_overflow = true;
    else users_buf.concat((const char*)u.buf, u.currentSize);
  }
}

void handle_users_import() {
  if (!auth()) { users_buf = ""; return; }
  JsonDocument in;
  DeserializationError err = users_overflow ? DeserializationError::NoMemory : deserializeJson(in, users_buf);
  users_buf = "";
  if (err || !in.is<JsonArray>()) {
    web.send(400, "text/plain", "Invalid users file (JSON array expected)");
    return;
  }
  JsonDocument nd;
  JsonArray arr = nd.to<JsonArray>();
  for (JsonObject o : in.as<JsonArray>()) {
    const char *addr = o["tcs_addr"];
    if (!addr || !*addr) continue;
    char *end;
    strtoul(addr, &end, 16);
    if (*end || strlen(addr) > 8 || arr.size() >= MAX_USERS) {
      web.send(400, "text/plain", String("Invalid or too many users (max ") + MAX_USERS + ")");
      return;
    }
    JsonObject n = arr.add<JsonObject>();
    n["tcs_addr"] = addr;
    n["name"] = o["name"] | "n/a";
    n["flat_no"] = o["flat_no"] | 0;
    n["rings"] = o["rings"] | 0u;
    n["last_ring"] = o["last_ring"] | 0u;
  }
  String out;
  serializeJson(nd, out);
  if (out.length() > 7000 || !write_file_atomic("/tcs_users.json", out)) {
    web.send(500, "text/plain", "Could not save users");
    return;
  }
  intercomUsers.clear();
  deserializeJson(intercomUsers, out);
  send_page("/backup", "<p>Imported " + String(arr.size()) + " users.</p><p><a href='/users'>Users</a></p>");
}

void handle_reboot() {
  if (!auth()) return;
  send_page("", "<p>Rebooting...</p><p><a href='/'>Back to status</a></p>");
  restart_at = millis() + 1500;
}

void handle_backup_download() {
  if (!auth()) return;
  JsonDocument json;
  config_to_json(json);
  String out;
  serializeJsonPretty(json, out);
  web.sendHeader("Content-Disposition", "attachment; filename=tcs-config.json");
  web.send(200, "application/json", out);
}

// Restore uploads are buffered (max 4 KB) and only applied if the final handler authenticates
static String restore_buf;
static bool restore_overflow = false;

void handle_restore_upload() {
  HTTPUpload &u = web.upload();
  if (u.status == UPLOAD_FILE_START) {
    restore_buf = "";
    restore_overflow = false;
  } else if (u.status == UPLOAD_FILE_WRITE) {
    if (restore_buf.length() + u.currentSize > 4096) restore_overflow = true;
    else restore_buf.concat((const char*)u.buf, u.currentSize);
  }
}

void handle_restore_done() {
  if (!auth()) { restore_buf = ""; return; }

  JsonDocument json;
  DeserializationError err = restore_overflow ? DeserializationError::NoMemory : deserializeJson(json, restore_buf);
  restore_buf = "";
  if (err) {
    web.send(400, "text/plain", String("Invalid backup: ") + err.c_str());
    return;
  }
  apply_config(json);
  save_and_reboot();
}

// ---- Users (serial number -> name), stored in LittleFS /tcs_users.json ----

void handle_users_get() {
  if (!auth()) return;
  String h = "<form method='POST' action='/users'><table><tr><th>No.</th><th>Serial No.</th><th>Flat No.</th><th>Surname / Name</th></tr>";
  int n = 0;
  for (JsonObject u : intercomUsers.as<JsonArray>()) {
    if (n >= MAX_USER_ROWS) break;
    h += "<tr><td>" + String(n + 1) + "</td><td><input name='a" + String(n) + "' size='8' value='" + html_escape(u["tcs_addr"] | "") + "'></td>";
    h += "<td><input name='f" + String(n) + "' size='4' value='" + String(u["flat_no"] | 0) + "'></td>";
    h += "<td><input name='n" + String(n) + "' value='" + html_escape(u["name"] | "") + "'></td></tr>";
    n++;
  }
  for (int i = 0; i < 2 && n < MAX_USER_ROWS; i++, n++) {
    h += "<tr><td>" + String(n + 1) + "</td><td><input name='a" + String(n) + "' size='8'></td><td><input name='f" + String(n) + "' size='4'></td><td><input name='n" + String(n) + "'></td></tr>";
  }
  h += "</table><p>Clear the serial to delete a row; use the blank rows to add users (max " + String(MAX_USER_ROWS) + ").</p>";
  h += "<input type='hidden' name='rows' value='" + String(n) + "'><button>Save</button></form>";
  send_page("/users", h);
}

void handle_users_post() {
  if (!auth()) return;
  int rows = min((int)web.arg("rows").toInt(), MAX_USER_ROWS);
  JsonDocument nd;
  JsonArray arr = nd.to<JsonArray>();
  for (int i = 0; i < rows; i++) {
    String a = web.arg("a" + String(i));
    String nm = web.arg("n" + String(i));
    a.trim();
    nm.trim();
    if (a.length() == 0) continue;
    char *end;
    strtoul(a.c_str(), &end, 16);
    if (*end || a.length() > 8) {
      web.send(400, "text/plain", "Invalid serial (hex expected): " + a);
      return;
    }
    JsonObject o = arr.add<JsonObject>();
    o["tcs_addr"] = a;
    o["name"] = nm;
    o["flat_no"] = (int)web.arg("f" + String(i)).toInt();
    JsonObject old = find_user(strtoul(a.c_str(), nullptr, 16));
    if (!old.isNull()) {
      o["rings"] = old["rings"] | 0u;
      o["last_ring"] = old["last_ring"] | 0u;
    }
  }
  String out;
  serializeJson(nd, out);
  if (out.length() > 7000 || !write_file_atomic("/tcs_users.json", out)) {
    web.send(500, "text/plain", "Could not save users (too many or flash error)");
    return;
  }
  intercomUsers.clear();
  deserializeJson(intercomUsers, out);
  send_page("/users", "<p>Saved " + String(arr.size()) + " users.</p><p><a href='/users'>Back</a></p>");
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
    // Config lives in NVRAM; LittleFS only holds tcs_users.json (and the legacy config.json, migrated on first boot)
    EEPROM.begin(NV_SIZE);
    Serial.println("mounting FS...");
    bool fs_ok = LittleFS.begin();
    load_config();

    if (fs_ok) {
        Serial.println("Read configured users...");
        if (LittleFS.exists("/tcs_users.json")) {
            std::unique_ptr<char[]> buf = read_file("/tcs_users.json");
            if (buf) {
                DeserializationError error = deserializeJson(intercomUsers, buf.get());
                if (!error) {
                    Serial.println("parsed users json");
                } else {
                    Serial.printf("failed to load users: %s\n", error.c_str());
                }
            }
        } else {
          Serial.println("File tcs_users.json does not exist!");
        }
        load_rings();
        load_stats();
    }
    else
    {
        Serial.println("failed to mount FS");
    }

    // Join the configured network directly; WiFiManager below is the fallback
    if (wifi_ssid[0]) {
        Serial.printf("joining %s\n", wifi_ssid);
        WiFi.mode(WIFI_STA);
        WiFi.begin(wifi_ssid, effective_pass().c_str());
        for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) delay(500);
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
    char ap_name[16];
    snprintf(ap_name, sizeof(ap_name), "TCS-%06x", (unsigned)ESP.getChipId());
    String ap_pass = effective_pass();
    if (ap_pass.length() < 8) ap_pass = "intercom"; // fallback AP only; WPA2 needs 8+ chars
    if (WiFi.status() != WL_CONNECTED && !wifiManager.autoConnect(ap_name, ap_pass.c_str()))
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

    configTime(tz, ntp_server); // non-blocking; time syncs in the background

    web.on("/", HTTP_GET, handle_home);
    web.on("/status", HTTP_GET, handle_status);
    web.on("/unlock", HTTP_POST, handle_unlock);
    web.on("/debug", HTTP_GET, handle_debug_page);
    web.on("/debug", HTTP_POST, handle_debug);
    web.on("/wifi", HTTP_GET, handle_wifi_get);
    web.on("/wifi", HTTP_POST, handle_wifi_post);
    web.on("/mqtt", HTTP_GET, handle_mqtt_get);
    web.on("/mqtt", HTTP_POST, handle_mqtt_post);
    web.on("/time", HTTP_GET, handle_time_get);
    web.on("/time", HTTP_POST, handle_time_post);
    web.on("/rings", HTTP_GET, handle_rings_page);
    web.on("/users", HTTP_GET, handle_users_get);
    web.on("/users", HTTP_POST, handle_users_post);
    web.on("/backup", HTTP_GET, handle_backup_page);
    web.on("/reboot", HTTP_POST, handle_reboot);
    web.on("/users/download", HTTP_GET, handle_users_download);
    web.on("/users/restore", HTTP_POST, handle_users_import, handle_users_upload);
    web.on("/rings/download", HTTP_GET, handle_rings_download);
    web.on("/backup/download", HTTP_GET, handle_backup_download);
    web.on("/restore", HTTP_POST, handle_restore_done, handle_restore_upload);
    web.collectHeaders("Authorization");
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
    String ota_pass = effective_pass();
    if (ota_pass.length() > 0) ArduinoOTA.setPassword(ota_pass.c_str());
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
  char msg[400];
  char name[40];

  current_time(mqtt_time);
  snprintf(msg, sizeof(msg), "{ \"ts\": \"%lu\", \"time\": \"%s\", \"direction\": \"%s\"",
           (unsigned long)time(nullptr), mqtt_time, direction ? "output" : "input");

  if (data.command) append(msg, sizeof(msg), ", \"command\": \"%08x\"", (unsigned)data.command);
  if (data.type) append(msg, sizeof(msg), ", \"type\": \"%s\"", command_type_to_string(data.type));
  if (data.serial_number) append(msg, sizeof(msg), ", \"serial_number\": \"%08x\"", (unsigned)data.serial_number);
  if (data.serial_number && get_tcsuser_name(data.serial_number, name, sizeof(name))) {
    // names come from user input: drop characters that would break the JSON
    for (char *c = name; *c; c++) if (*c == '"' || *c == '\\' || (unsigned char)*c < 0x20) *c = ' ';
    append(msg, sizeof(msg), ", \"name\": \"%s\"", name);
  }
  append(msg, sizeof(msg), ", \"address\": \"%02x\", \"payload\": \"%x\" }", data.address, (unsigned)data.payload);

  client.publish(topic, msg);
}

void loop() {
    static uint32_t last_wifi_try = 0;
    static uint32_t last_mqtt_try = 0;
    static uint32_t last_status = 0;
    uint32_t now = millis();
    uint32_t heap_now = ESP.getFreeHeap();
    if (heap_now < min_free_heap) min_free_heap = heap_now;

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
      if (cmd_data.type == COMMAND_TYPE_OPEN_DOOR || cmd_data.type == COMMAND_TYPE_OPEN_DOOR_LONG) count_unlock();
      if ((cmd_data.type == COMMAND_TYPE_DOOR_CALL || cmd_data.type == COMMAND_TYPE_INTERNAL_CALL) && cmd_data.serial_number)
        record_ring(cmd_data.serial_number, cmd_data.type);

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
