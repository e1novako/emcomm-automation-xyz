#include <FS.h> //this needs to be first, or it all crashes and burns...
#include <LittleFS.h>

#include <ESP8266WiFi.h> //https://github.com/esp8266/Arduino

#include "tcs_protocol.h"

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager

#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson

#include <PubSubClient.h>

#include <time.h>

#define VERSION "1.0.2"

#define TCS_RING        0x00
#define TCS_ANSWER      0x80
#define TCS_ONHOOK      0x30000000
#define TCS_ONERING     0x7F

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 3600;

int mqtt_connection_status = 0;

WiFiClient net;
PubSubClient client(net);

//JsonDocument intercomUsers;
DynamicJsonDocument intercomUsers(4096);

//define your default values here, if there are different values in config.json, they are overwritten.
char mqtt_server[40] = "192.168.10.243";
char mqtt_port[6] = "7953";
char mqtt_topic[64] = "ucl/by-mqtt-client/intercom";
char control4_command_topic[64] = "ucl/by-unid/zw-D80822E0-0084/ep0/OnOff/Commands/Toggle";
char control4_notify_topic[80] = "ucl/by-unid/zw-D80822E0-0084/ep0/IASZone/Attributes/ZoneStatus/Reported";
char tcs_command_topic[64] = "ucl/by-mqtt-client/intercom/command";
char mqtt_client[16] = "anovakovic";
char mqtt_user[16] = "";
char mqtt_pass[16] = "";

uint32_t my_serial_number = 0x8c0b9;
uint32_t last_bus_address = 3;

//flag for saving data
bool shouldSaveConfig = false;

#define inputPin    14  // D5 - GPIO14
#define outputPin   12  // D6 - GPIO15
#define restorePin  0   // D0
#define startBit    6
#define oneBit      4
#define zeroBit     2

uint32_t nCMD = 0;
volatile uint32_t CMD = 0;
volatile uint32_t _CMD = -1;
volatile uint8_t lengthCMD = 0;
volatile bool cmdReady;

void get_tcsuser_data_by_addr(uint32_t u32_addr, char *_name) {
uint32_t _addr = 0;

  //Serial.println("================================================================\n");
  //Serial.println("TONY:  Let's try to find " + String(u32_addr));

  for (JsonObject elem : intercomUsers.as<JsonArray>()) {
    const char* name = elem["name"];
    const char* addr = elem["tcs_addr"];
    const char* flat = elem["flat_no"];

    sscanf(addr, "%x", &_addr);

    if (u32_addr == _addr) {
      strcpy(_name, name);
    }
  }
}

uint32_t stop_talking(uint32_t address) {
uint32_t command = 0;
//  case COMMAND_TYPE_STOP_TALKING:

  command |= (3 << 12); // 3
  command &= ~(1 << 7); // 00
  command |= (address & 0x3F); // 0

  Serial.println("Stop talking " + String(command));

  return command;
}

struct tm current_time(char *mqtt_timestamp) {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
  }

  strftime(mqtt_timestamp, 12, "%H:%M:%S", &timeinfo);
  mqtt_timestamp[8]=0;

  return timeinfo;
}

void ICACHE_RAM_ATTR analyzeCMD()
{
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
            cmdReady = 1;
            lengthCMD = curLength;
            CMD = curCMD;
        }
        curCMD = 0;
        curPos = 0;
    }
}

void send_command(uint32_t tcs_protocol_command) {
  int length = 16;
  byte checksm = 1;
  byte firstBit = 0;

  if (tcs_protocol_command > 0xFFFF) {
    length = 32;
    firstBit = 1;
  }
  char helper[30];
  //sprintf(helper, "Sending %s command: %08x", (length==16) ? "short" : "long", tcs_protocol_command);
  //Serial.println(helper);

  // Disable interrupt during command send
  detachInterrupt(digitalPinToInterrupt(inputPin));

  digitalWrite(outputPin, HIGH);
  delay(startBit);
  digitalWrite(outputPin, !digitalRead(outputPin));
  delay(firstBit ? oneBit : zeroBit);
  int curBit = 0;
  for (byte i = length; i > 0; i--) {
    curBit = bitRead(tcs_protocol_command, i - 1);
    digitalWrite(outputPin, !digitalRead(outputPin));
    delay(curBit ? oneBit : zeroBit);
    checksm ^= curBit;
  }
  digitalWrite(outputPin, !digitalRead(outputPin));
  delay(checksm ? oneBit : zeroBit);
  digitalWrite(outputPin, LOW);

  // Enable interrupts after comand was sent
  attachInterrupt(digitalPinToInterrupt(inputPin), analyzeCMD, CHANGE);
}

void control4_notify(CommandData  data) {
  client.publish(control4_notify_topic, "{\"value\":{\"ACMainsFault\":false,\"Alarm1\":true,\"Alarm2\":false,\"BatteryDefect\":false,\"BatteryLow\":false,\"RestoreNotify\":false,\"SupervisionNotify\":false,\"Tamper\":false,\"Test\":false,\"Trouble\":false}}");
  client.publish(control4_notify_topic, "{\"value\":{\"ACMainsFault\":false,\"Alarm1\":false,\"Alarm2\":false,\"BatteryDefect\":false,\"BatteryLow\":false,\"RestoreNotify\":false,\"SupervisionNotify\":false,\"Tamper\":false,\"Test\":false,\"Trouble\":false}}");
} 

void mqtt_output(char *format, ...) {
    char mqtt_tmp[100];
    char mqtt_msg[100];
    char mqtt_time[10];
    char mqtt_timestamp[20];
    struct tm timeinfo;
    va_list args;

    va_start (args, format);
    vsprintf(mqtt_tmp, format, args);
    va_end (args);

    timeinfo=current_time((char*)&mqtt_time);
    strftime(mqtt_timestamp, 11, "%s", &timeinfo);

    sprintf(mqtt_msg, "{ \"ts\": \"%s\", \"time\": \"%s\", %s }", mqtt_timestamp, mqtt_time, mqtt_tmp);
    client.publish(mqtt_topic, mqtt_msg);
}

void connectToMqtt() {
  char mcli[32];
  sprintf(mcli, "@%s-%d", mqtt_client, random(100,999));
  Serial.print("\nconnecting ");
  Serial.print(mcli);
  Serial.print(" to MQTT...");
  while (!client.connect(mcli, mqtt_user, mqtt_pass)) {
      Serial.print(".");
      delay(5000);
  }

  // Set AutoReconnect !!!
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  mqtt_connection_status=1;
  mqtt_output((char*)"\"type\": \"sniffer_online\", \"version\": \"" VERSION "\"");
}

void connectToWifi() {
    Serial.print("checking wifi...");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(1000);
    }

    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    while (time(nullptr) < 1000) {
        delay(100); // Adjust the delay time as per your requirements
    }

    connectToMqtt();

    Serial.println("\nconnected!");
}

void sendMessage() {
    client.publish(mqtt_topic, "triggered");
}

//callback notifying us of the need to save config
void saveConfigCallback() {
    Serial.println("Should save config");
    shouldSaveConfig = true;
}

void callback(char *topic, byte *payload, unsigned int length) {
  uint32_t tcs_command = 0;
  JsonDocument doc;
  char *message;
 
  message=(char*)malloc(length+1);
  strncpy(message, (char*)payload, length);
  message[length]=0;
  Serial.println("Received: " + String(message));

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, message);
  if (error) {
    Serial.print(F("JSON error: #"));
    Serial.print(error.code());
    Serial.print(" ");
    Serial.println(error.c_str());
    return;
  } else {
    if (String(topic) == String(tcs_command_topic)) {
      //const char* command = doc["command"];
      uint32_t type = 0;
      if (doc["type"])
        type=string_to_command_type(doc["type"]);
      uint32_t address = 0;
      if (doc["address"]) { 
        //address = (uint32_t)atoi(doc["address"]);
        sscanf(doc["address"], "%x", &address);
      }
      uint32_t payload = 0;
      if (doc["payload"]) {
        payload = (uint32_t)atoi(doc["payload"]);
        sscanf(doc["payload"], "%x", &payload);
      }
      uint32_t serial_number = 0;
      if (doc["serial_number"]) {
        //serial_number = (uint32_t)atoi(doc["serial_number"]);
        sscanf(doc["serial_number"], "%x", &serial_number);
      }

      //Serial.println("Type: " + String(type));

      if (type!=COMMAND_TYPE_UNKNOWN) {
        CommandData command_data = buildCommand(type, address, payload, serial_number);
        if(command_data.command > 0) {
          publish_tcs_command(mqtt_topic, command_data, true);
          send_command(command_data.command);
        }
      } else {
        Serial.println("Unknown tcs_command %s " + String(message));
      }
    } else if (String(topic) == String(control4_command_topic)) {
      uint32_t type = COMMAND_TYPE_OPEN_DOOR;
 
      CommandData command_data = buildCommand(type, last_bus_address, 0, 0);
      if(command_data.command > 0) {
        publish_tcs_command(mqtt_topic, command_data, true);
        send_command(command_data.command);
      }
    }
  }

  free(message);
}

void printHEX(uint32_t DATA)
{
    uint8_t numChars = lengthCMD ? 8 : 4;
    uint32_t mask = 0x0000000F;
    mask = mask << 4 * (numChars - 1);
    for (uint32_t i = numChars; i > 0; --i)
    {
        Serial.print(((DATA & mask) >> (i - 1) * 4), HEX);
        mask = mask >> 4;
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
            File configFile = LittleFS.open("/config.json", "r");
            if (configFile) {
                Serial.println("opened config file");
                size_t size = configFile.size();
                // Allocate a buffer to store contents of the file.
                std::unique_ptr<char[]> buf(new char[size]);

                configFile.readBytes(buf.get(), size);
                DynamicJsonDocument json(1024);
                DeserializationError error = deserializeJson(json, buf.get());
                serializeJson(json, Serial);
                if (!error) {
                    Serial.println("\nparsed json");

                    strcpy(mqtt_server, json["mqtt_server"]);
                    strcpy(mqtt_port, json["mqtt_port"]);
                    strcpy(mqtt_topic, json["mqtt_topic"]);
                    strcpy(mqtt_client, json["mqtt_client"]);
                    strcpy(mqtt_user, json["mqtt_user"]);
                    strcpy(mqtt_pass, json["mqtt_pass"]);
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
            File configFile = LittleFS.open("/tcs_users.json", "r");
            if (configFile) {
                Serial.println("opened users file");
                size_t size = configFile.size();
                // Allocate a buffer to store contents of the file.
                std::unique_ptr<char[]> buf(new char[size]);

                configFile.readBytes(buf.get(), size);
                DynamicJsonDocument json(4096);
                DeserializationError error = deserializeJson(intercomUsers, buf.get());
                serializeJson(intercomUsers, Serial);
                if (!error) {
                    Serial.println("\nparsed json");
                } else {
                    Serial.println("failed to load json config");
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
    WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
    WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 5);
    WiFiManagerParameter custom_mqtt_topic("topic", "mqtt topic", mqtt_topic, 32);
    WiFiManagerParameter custom_mqtt_client("client", "mqtt client", mqtt_client, 32);
    WiFiManagerParameter custom_mqtt_user("user", "mqtt user", mqtt_user, 32);
    WiFiManagerParameter custom_mqtt_pass("pass", "mqtt pass", mqtt_pass, 32);

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
    strcpy(mqtt_server, custom_mqtt_server.getValue());
    strcpy(mqtt_port, custom_mqtt_port.getValue());
    strcpy(mqtt_topic, custom_mqtt_topic.getValue());
    strcpy(mqtt_client, custom_mqtt_client.getValue());
    strcpy(mqtt_user, custom_mqtt_user.getValue());
    strcpy(mqtt_pass, custom_mqtt_pass.getValue());

    //save the custom parameters to FS
    if (shouldSaveConfig)
    {
        Serial.println("saving config");
        DynamicJsonDocument json(1024);
        json["mqtt_server"] = mqtt_server;
        json["mqtt_port"] = mqtt_port;
        json["mqtt_topic"] = mqtt_topic;
        json["mqtt_client"] = mqtt_client;
        json["mqtt_user"] = mqtt_user;
        json["mqtt_pass"] = mqtt_pass;

        File configFile = LittleFS.open("/config.json", "w");
        if (!configFile)
        {
            Serial.println("failed to open config file for writing");
        }

        serializeJson(json, Serial);
        serializeJson(json, configFile);
        configFile.close();
        //end save
    }

    Serial.println("local ip");
    Serial.println(WiFi.localIP());

    Serial.println("get ntp time");

    Serial.print("MQTT Server ");
    Serial.println(mqtt_server);
    Serial.print("MQTT port ");
    Serial.println(atoi(mqtt_port));

    client.setServer(mqtt_server, atoi(mqtt_port));
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

    connectToWifi();
}

void publish_tcs_command(char *topic, CommandData data, bool direction) {
struct tm timeinfo;
char mqtt_time[20];
char mqtt_timestamp[12];
char format[350];
char tcs_tmp[50];

timeinfo=current_time((char*)&mqtt_time);
strftime(mqtt_timestamp, 11, "%s", &timeinfo);

sprintf(format, "{ \"ts\": \"%s\", \"time\": \"%s\"", mqtt_timestamp, mqtt_time);

strcat(format, ", \"direction\": ");
if (direction)
  strcat(format, "\"output\"");
else
  strcat(format, "\"input\"");

if (data.command) {
  strcat(format, ", \"command\": ");
  sprintf(tcs_tmp, "\"%08x\"", data.command);
  strcat(format, tcs_tmp);
}

if (data.type) {
  strcat(format, ", \"type\": ");
  sprintf(tcs_tmp, "\"%s\"", command_type_to_string(data.type));
  strcat(format, tcs_tmp);
}

if (data.serial_number) {
  strcat(format, ", \"serial_number\": ");
  sprintf(tcs_tmp, "\"%08x\"", data.serial_number);
  strcat(format, tcs_tmp);

/*
  char *s_tmp; 
  s_tmp = (char*)calloc(32 * sizeof(char), 0);
  get_tcsuser_data_by_addr(data.serial_number, s_tmp);
  if (s_tmp[0] != 0) {
    strcat(format, ", \"flat\": ");
    strcat(format, s_tmp);
  }
  free(s_tmp);
*/
}

if (data.address) {
  strcat(format, ", \"address\": ");
  sprintf(tcs_tmp, "\"%02x\"", data.address);
  strcat(format, tcs_tmp);
}

if (data.payload > 0) {
  strcat(format, ", \"payload\": ");
  sprintf(tcs_tmp, "\"%x\"", data.payload);
  strcat(format, tcs_tmp);
} 

strcat(format, " }");
client.publish(topic, format);

}


void loop() {
    if (WiFi.status() != WL_CONNECTED) {
      connectToWifi();
      mqtt_connection_status = 0;
    }
    if (!client.connected()) { 
      connectToMqtt();
    }
    if (mqtt_connection_status == 1) {
      mqtt_connection_status = 0;
      Serial.println("subscribe to topic....");
      client.subscribe(tcs_command_topic);
      client.subscribe(control4_command_topic);
    }
    client.loop();
    ArduinoOTA.handle();
    if (cmdReady) {
      cmdReady = 0;

      CommandData cmd_data = parseCommand(CMD);
      if (cmd_data.serial_number>0)
        last_bus_address = cmd_data.address;

      publish_tcs_command(mqtt_topic, cmd_data, false);

      if ( cmd_data.serial_number == my_serial_number &&  cmd_data.type == COMMAND_TYPE_INTERNAL_CALL) {
        control4_notify(cmd_data);
      }
    }
}
