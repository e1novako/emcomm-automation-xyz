#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <LittleFS.h>
extern "C" {
#include "user_interface.h"
}
#include "Config.h"
#include "Debug.h"
#include "MqttClient.h"
#include "Reservations.h"
#include "WifiManager.h"

namespace vibrant {

const char CONFIG_PATH[] = "/vibrant_config.json";
const char IMPORT_CONFIG_PATH[] = "/vibrant_config_upload.json";
const char DEFAULT_STA_SSID[] = "Z-Wave Automation";
const char DEFAULT_STA_PASSWORD[] = "Fiber714Cvet";
const char DEFAULT_AP_PASSWORD[] = "Fiber714Cvet";
const char DEFAULT_DEVICE_MANUFACTURER[] = "Control4";
const char DEFAULT_DEVICE_MODEL[] = "Vibrant";
const uint8_t DEFAULT_NUM_OUTPUTS = 8;
const float MIN_WIFI_POWER = 5.0f;
const float MAX_WIFI_POWER = 20.5f;
const unsigned long OUTPUT_BOOT_ACTIVATION_DELAY_MS = 1000UL;
const uint16_t DEFAULT_MQTT_PORT = 1883;
constexpr uint8_t DEFAULT_D0_D7_COUNT = 8;
const PinMapping OUTPUT_PIN_MAPPINGS[] = {
    {16, "D0 (GPIO16)"},
    {5, "D1 (GPIO5)"},
    {4, "D2 (GPIO4)"},
    {0, "D3 (GPIO0) - boot/FLASH pin"},
    {2, "D4 (GPIO2)"},
    {14, "D5 (GPIO14)"},
    {12, "D6 (GPIO12)"},
    {13, "D7 (GPIO13)"},
    {15, "D8 (GPIO15) - must be LOW at boot"},
    {3, "RX (GPIO3) - disables serial receive"},
    {1, "TX (GPIO1) - disables serial logging"}};
const size_t OUTPUT_PIN_MAPPING_COUNT =
    sizeof(OUTPUT_PIN_MAPPINGS) / sizeof(OUTPUT_PIN_MAPPINGS[0]);
static_assert(
    DEFAULT_D0_D7_COUNT <= OUTPUT_PIN_MAPPING_COUNT,
    "DEFAULT_D0_D7_COUNT exceeds available OUTPUT_PIN_MAPPINGS entries.");
DeviceConfig cfg;
const unsigned long WIFI_RECONNECT_INTERVAL_MS = 15000UL,
                    WIFI_CONNECT_LOG_INTERVAL_MS = 5000UL,
                    WIFI_RECOVERY_WINDOW_MS = 180000UL;
const uint8_t MAX_WIFI_RECOVERY_ATTEMPTS = 12;
const unsigned long MQTT_RECONNECT_INTERVAL_MS = 10000UL;
const uint16_t MQTT_PACKET_BUFFER_SIZE = 2048, MQTT_PAYLOAD_LOG_MAX_LEN = 120;
bool parseMac(const String &mac, uint8_t out[6]) {
  if (mac.length() != 17)
    return false;
  for (uint8_t i = 0; i < 6; ++i) {
    char hi = mac[i * 3];
    char lo = mac[i * 3 + 1];
    if (i < 5 && mac[i * 3 + 2] != ':')
      return false;
    auto hex = [](char c) -> int {
      if (c >= '0' && c <= '9')
        return c - '0';
      if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
      if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
      return -1;
    };
    int h = hex(hi);
    int l = hex(lo);
    if (h < 0 || l < 0)
      return false;
    out[i] = static_cast<uint8_t>((h << 4) | l);
  }
  return true;
}

bool applyConfiguredMac() {
  uint8_t mac[6] = {0};
  if (!parseMac(cfg.mac, mac)) {
    logWarning(
        F("Configured MAC address is invalid; continuing with hardware MAC."));
    return false;
  }

  bool stationOk = wifi_set_macaddr(STATION_IF, mac);
  bool apOk = wifi_set_macaddr(SOFTAP_IF, mac);
  if (!stationOk || !apOk) {
    logWarning(F("Failed to apply configured MAC address to one or more "
                 "interfaces; continuing with hardware MAC."));
    return false;
  }

  logStatus(String(F("Applied MAC address: ")) + cfg.mac);
  return true;
}

void setFactoryDefaults() {
  cfg.mac = WiFi.softAPmacAddress();
  cfg.useCustomMac = false;
  cfg.hostname = defaultHostnameFromMac(cfg.mac);
  cfg.staSsid = DEFAULT_STA_SSID;
  cfg.staPassword = DEFAULT_STA_PASSWORD;
  cfg.apPassword = DEFAULT_AP_PASSWORD;
  cfg.wifiPower = 20.5f;
  cfg.numOutputs = DEFAULT_NUM_OUTPUTS;

  for (uint8_t i = 0; i < MAX_DEVICES; ++i) {
    cfg.devices[i].manufacturer = DEFAULT_DEVICE_MANUFACTURER;
    cfg.devices[i].model = DEFAULT_DEVICE_MODEL;
    cfg.devices[i].name = String(F("Output ")) + String(i + 1);
    // Map first DEFAULT_D0_D7_COUNT outputs to D0-D7 by default; rest
    // unassigned. Compile-time static_assert above guarantees
    // DEFAULT_D0_D7_COUNT <= OUTPUT_PIN_MAPPING_COUNT.
    cfg.devices[i].pin =
        (i < DEFAULT_D0_D7_COUNT) ? OUTPUT_PIN_MAPPINGS[i].gpio : -1;
    cfg.devices[i].state = false;
    cfg.devices[i].reserved = false;
    cfg.devices[i].reservedOwner = "";
  }

  cfg.mqttEnabled = false;
  cfg.mqttHost = "";
  cfg.mqttPort = DEFAULT_MQTT_PORT;
  cfg.mqttUser = "";
  cfg.mqttPassword = "";
  cfg.arduinoOtaEnabled = true;
  cfg.debugSerial = false;
  cfg.stickserverRespondEnabled = true;
  cfg.stickserverQueryEnabled = false;
  cfg.stickserverPassiveDiscoveryEnabled = false;
  cfg.restoreOutputStateOnBoot = false;
  cfg.restoreReservationsOnBoot = false;
  clearOutputReservations();

  logStatus(F("Factory defaults loaded."));
}

bool saveConfig() {
  JsonDocument doc;
  doc["mac"] = cfg.mac;
  doc["useCustomMac"] = cfg.useCustomMac;
  doc["hostname"] = cfg.hostname;
  doc["staSsid"] = cfg.staSsid;
  doc["staPassword"] = cfg.staPassword;
  doc["apPassword"] = cfg.apPassword;
  doc["wifiPower"] = cfg.wifiPower;
  doc["numOutputs"] = cfg.numOutputs;
  doc["mqttEnabled"] = cfg.mqttEnabled;
  doc["mqttHost"] = cfg.mqttHost;
  doc["mqttPort"] = cfg.mqttPort;
  doc["mqttUser"] = cfg.mqttUser;
  doc["mqttPassword"] = cfg.mqttPassword;
  doc["arduinoOtaEnabled"] = cfg.arduinoOtaEnabled;
  doc["debugSerial"] = cfg.debugSerial;
  doc["stickserverRespondEnabled"] = cfg.stickserverRespondEnabled;
  doc["stickserverQueryEnabled"] = cfg.stickserverQueryEnabled;
  doc["stickserverPassiveDiscoveryEnabled"] =
      cfg.stickserverPassiveDiscoveryEnabled;
  doc["restoreOutputStateOnBoot"] = cfg.restoreOutputStateOnBoot;
  doc["restoreReservationsOnBoot"] = cfg.restoreReservationsOnBoot;

  JsonArray devices = doc["devices"].to<JsonArray>();
  for (uint8_t i = 0; i < MAX_DEVICES; ++i) {
    JsonObject d = devices.add<JsonObject>();
    d["manufacturer"] = cfg.devices[i].manufacturer;
    d["model"] = cfg.devices[i].model;
    d["name"] = cfg.devices[i].name;
    d["pin"] = cfg.devices[i].pin;
    d["state"] = cfg.devices[i].state;
    d["reserved"] = cfg.devices[i].reserved;
    d["reservedOwner"] = cfg.devices[i].reservedOwner;
  }

  File file = LittleFS.open(CONFIG_PATH, "w");
  if (!file) {
    logError(F("Failed to open config file for writing."));
    return false;
  }

  if (serializeJsonPretty(doc, file) == 0) {
    file.close();
    logError(F("Failed to serialize config JSON."));
    return false;
  }

  file.close();
  logStatus(F("Configuration saved to LittleFS."));
  return true;
}

bool loadConfig() {
  logStatus(F("Loading configuration from LittleFS..."));
  if (!LittleFS.exists(CONFIG_PATH)) {
    logStatus(F("Configuration file not found; generating factory defaults."));
    setFactoryDefaults();
    return saveConfig();
  }

  File file = LittleFS.open(CONFIG_PATH, "r");
  if (!file) {
    logError(
        F("Unable to open configuration file; restoring factory defaults."));
    setFactoryDefaults();
    return saveConfig();
  }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, file);
  file.close();
  if (err) {
    logError(String(F("Configuration JSON parse failed: ")) + err.c_str());
    setFactoryDefaults();
    return saveConfig();
  }

  cfg.mac = doc["mac"] | WiFi.softAPmacAddress();
  cfg.useCustomMac = doc["useCustomMac"] | false;
  cfg.hostname = doc["hostname"] | defaultHostnameFromMac(cfg.mac);

  String legacySsid = doc["ssid"] | String(DEFAULT_STA_SSID);
  String legacyPassword = doc["password"] | String(DEFAULT_STA_PASSWORD);
  cfg.staSsid = doc["staSsid"] | legacySsid;
  cfg.staPassword = doc["staPassword"] | legacyPassword;
  cfg.apPassword = doc["apPassword"] | legacyPassword;
  cfg.wifiPower = doc["wifiPower"] | 20.5f;
  // Backward-compat: existing saved configs without numOutputs default to
  // MAX_DEVICES so no previously configured outputs are hidden unexpectedly.
  cfg.numOutputs = static_cast<uint8_t>(doc["numOutputs"] |
                                        static_cast<uint8_t>(MAX_DEVICES));
  if (cfg.numOutputs < 1 || cfg.numOutputs > MAX_DEVICES)
    cfg.numOutputs = MAX_DEVICES;
  cfg.restoreOutputStateOnBoot = doc["restoreOutputStateOnBoot"] | false;
  cfg.restoreReservationsOnBoot = doc["restoreReservationsOnBoot"] | false;

  JsonArray devices = doc["devices"].as<JsonArray>();
  for (uint8_t i = 0; i < MAX_DEVICES; ++i) {
    if (i < devices.size()) {
      JsonObject d = devices[i];
      cfg.devices[i].manufacturer =
          d["manufacturer"] | String(DEFAULT_DEVICE_MANUFACTURER);
      cfg.devices[i].model = d["model"] | String(DEFAULT_DEVICE_MODEL);
      String manufacturer = cfg.devices[i].manufacturer;
      String model = cfg.devices[i].model;
      manufacturer.trim();
      model.trim();
      if (manufacturer.isEmpty())
        cfg.devices[i].manufacturer = DEFAULT_DEVICE_MANUFACTURER;
      if (model.isEmpty())
        cfg.devices[i].model = DEFAULT_DEVICE_MODEL;
      cfg.devices[i].name = d["name"] | String(F("Output ")) + String(i + 1);
      cfg.devices[i].pin = static_cast<int8_t>(d["pin"] | -1);
      // Only restore the last saved ON/OFF state when
      // restoreOutputStateOnBoot is enabled; otherwise always boot OFF.
      cfg.devices[i].state =
          cfg.restoreOutputStateOnBoot ? (bool)(d["state"] | false) : false;
      cfg.devices[i].reserved = d["reserved"] | false;
      cfg.devices[i].reservedOwner = d["reservedOwner"] | String("");
    } else {
      cfg.devices[i].manufacturer = DEFAULT_DEVICE_MANUFACTURER;
      cfg.devices[i].model = DEFAULT_DEVICE_MODEL;
      cfg.devices[i].name = String(F("Output ")) + String(i + 1);
      cfg.devices[i].pin = -1;
      cfg.devices[i].state = false;
      cfg.devices[i].reserved = false;
      cfg.devices[i].reservedOwner = "";
    }
  }
  if (cfg.restoreReservationsOnBoot) {
    for (uint8_t i = 0; i < MAX_DEVICES; ++i) {
      outputReservations[i].reserved = cfg.devices[i].reserved;
      outputReservations[i].owner = cfg.devices[i].reservedOwner;
    }
  } else {
    clearOutputReservations();
  }

  if (cfg.staSsid.isEmpty())
    cfg.staSsid = DEFAULT_STA_SSID;
  if (cfg.staPassword.isEmpty())
    cfg.staPassword = DEFAULT_STA_PASSWORD;
  if (cfg.apPassword.isEmpty())
    cfg.apPassword = DEFAULT_AP_PASSWORD;
  if (cfg.hostname.isEmpty())
    cfg.hostname = defaultHostnameFromMac(cfg.mac);

  cfg.mqttEnabled = doc["mqttEnabled"] | false;
  cfg.mqttHost = doc["mqttHost"] | String("");
  cfg.mqttPort = static_cast<uint16_t>(
      doc["mqttPort"] | static_cast<uint16_t>(DEFAULT_MQTT_PORT));
  cfg.mqttUser = doc["mqttUser"] | String("");
  cfg.mqttPassword = doc["mqttPassword"] | String("");
  if (cfg.mqttPort == 0)
    cfg.mqttPort = DEFAULT_MQTT_PORT;
  cfg.arduinoOtaEnabled = doc["arduinoOtaEnabled"] | true;
  cfg.debugSerial = doc["debugSerial"] | false;
  // Back-compat: configs saved before the single stickserverDiscoveryEnabled
  // flag was split default the two new discovery flags to its old value
  // (and keep responding enabled, matching the new default) when the new
  // keys aren't present yet.
  bool legacyDiscoveryEnabled = doc["stickserverDiscoveryEnabled"] | false;
  cfg.stickserverRespondEnabled = doc["stickserverRespondEnabled"] | true;
  cfg.stickserverQueryEnabled =
      doc["stickserverQueryEnabled"] | legacyDiscoveryEnabled;
  cfg.stickserverPassiveDiscoveryEnabled =
      doc["stickserverPassiveDiscoveryEnabled"] | legacyDiscoveryEnabled;

  logStatus(F("Configuration loaded successfully."));
  return true;
}

void performFactoryResetAndRestart(const String &reason) {
  logWarning(reason);
  setFactoryDefaults();
  if (!saveConfig()) {
    restartDevice(
        F("Failed to persist factory defaults during requested reset."));
  }
  delay(500);
  ESP.restart();
}

void maybePersistOutputState() {
  if (!cfg.restoreOutputStateOnBoot)
    return;
  saveConfig();
}

void maybePersistReservations() {
  if (!cfg.restoreReservationsOnBoot)
    return;
  for (uint8_t i = 0; i < MAX_DEVICES; ++i) {
    cfg.devices[i].reserved = outputReservations[i].reserved;
    cfg.devices[i].reservedOwner = outputReservations[i].owner;
  }
  saveConfig();
}

} // namespace vibrant
