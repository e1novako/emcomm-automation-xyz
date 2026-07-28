#include <Arduino.h>
#include <LittleFS.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Updater.h>

extern "C" {
#include "user_interface.h"
}

// ---------------------------------------------------------------------------
// DIAGNOSTICS FLAG — set to 1 to re-enable the custom MAC override, 0 to skip
// it for testing.  Remove this block (and the #if guards below) once the
// Wi-Fi connection issue has been diagnosed.
// ---------------------------------------------------------------------------
#define WIFI_DIAG_APPLY_CUSTOM_MAC 0

namespace {

// Forward declarations (defined below)
bool handleLoadAction(uint8_t idx, const String& cmd);
void cancelAction();

constexpr const char* CONFIG_PATH = "/vibrant_config.json";
constexpr const char* IMPORT_CONFIG_PATH = "/vibrant_config_upload.json";
constexpr const char* DEFAULT_STA_SSID = "Z-Wave Automation";
constexpr const char* DEFAULT_STA_PASSWORD = "Fiber714Cvet";
constexpr const char* DEFAULT_AP_PASSWORD = "Fiber714Cvet";
constexpr const char* SOFTWARE_VERSION = "1.1.8";
constexpr uint8_t MAX_DEVICES = 16;
constexpr uint8_t DEFAULT_NUM_OUTPUTS = 8;
constexpr float MIN_WIFI_POWER = 5.0f;
constexpr float MAX_WIFI_POWER = 20.5f;
constexpr unsigned long OUTPUT_BOOT_ACTIVATION_DELAY_MS = 1000UL;
constexpr unsigned long WIFI_RECONNECT_INTERVAL_MS = 15000UL;
constexpr unsigned long WIFI_CONNECT_LOG_INTERVAL_MS = 5000UL;
constexpr uint8_t FLASH_BUTTON_PIN = 0;
constexpr unsigned long FLASH_BOOT_DETECTION_WINDOW_MS = 750UL;
constexpr unsigned long FLASH_BOOT_SAMPLE_INTERVAL_MS = 10UL;
constexpr uint8_t FLASH_BOOT_REQUIRED_LOW_PERCENT = 30;
constexpr unsigned long FLASH_BOOT_MIN_SAMPLES = 4UL;
constexpr unsigned long WIFI_RECOVERY_WINDOW_MS = 180000UL;
constexpr uint8_t MAX_WIFI_RECOVERY_ATTEMPTS = 12;
constexpr uint16_t DEFAULT_MQTT_PORT = 1883;
constexpr unsigned long MQTT_RECONNECT_INTERVAL_MS = 10000UL;
constexpr uint16_t MQTT_PACKET_BUFFER_SIZE = 2048;
constexpr uint16_t MQTT_PAYLOAD_LOG_MAX_LEN = 120;
constexpr const char* STICKSERVER_ROOT_TOPIC = "s1/c4/stickserver/v1";
constexpr uint8_t STICKSERVER_PROTOCOL_VERSION = 1;
constexpr const char* STICKSERVER_OUTPUT_TYPE = "vibrant-output";
// Background load-action timing
constexpr uint8_t LEAVE_MESH_CYCLES = 5;
constexpr uint8_t REBOOT_SEQUENCE_CYCLES = 1;
// Number of D0-D7 entries at the start of OUTPUT_PIN_MAPPINGS used for default assignment
constexpr uint8_t DEFAULT_D0_D7_COUNT = 8;
// Generic sequence timing (used by reboot)
constexpr unsigned long ACTION_CYCLE_OFF_MS = 5000UL;
constexpr unsigned long ACTION_CYCLE_ON_MS = 1000UL;
constexpr unsigned long ACTION_FINAL_WAIT_MS = 5000UL;
constexpr unsigned long ACTION_TRIGGER_OFF_MS = 2000UL;
constexpr unsigned long ACTION_TRIGGER_ON_MS = 1000UL;
// Leave-mesh load action timing
constexpr unsigned long LEAVE_MESH_PREP_ON_MS = 4000UL;
constexpr unsigned long LEAVE_MESH_CYCLE_OFF_MS = 5000UL;
constexpr unsigned long LEAVE_MESH_CYCLE_ON_MS = 1000UL;
constexpr unsigned long LEAVE_MESH_FINAL_WAIT_MS = 5000UL;
constexpr unsigned long LEAVE_MESH_TRIGGER_OFF_MS = 5000UL;
constexpr unsigned long LEAVE_MESH_TRIGGER_ON_MS = 1000UL;
// Dedicated factory-reset load action timing
constexpr uint8_t FACTORY_RESET_CYCLES = 13;
constexpr unsigned long FACTORY_RESET_PREP_ON_MS = 4000UL;
constexpr unsigned long FACTORY_RESET_CYCLE_OFF_MS = 2000UL;
constexpr unsigned long FACTORY_RESET_CYCLE_ON_MS = 1000UL;
constexpr unsigned long FACTORY_RESET_FINAL_WAIT_MS = 5000UL;
constexpr unsigned long FACTORY_RESET_TRIGGER_OFF_MS = 2000UL;
constexpr unsigned long FACTORY_RESET_TRIGGER_ON_MS = 1000UL;

constexpr unsigned long ceilDiv(unsigned long numerator, unsigned long denominator) {
  if (denominator == 0) return 0;
  return (numerator + denominator - 1UL) / denominator;
}

constexpr unsigned long ceilPercentOf(unsigned long value, unsigned long percent) {
  return static_cast<unsigned long>(
      (static_cast<unsigned long long>(value) * percent + 99ULL) / 100ULL);
}

constexpr unsigned long FLASH_BOOT_SAMPLE_COUNT =
    ceilDiv(FLASH_BOOT_DETECTION_WINDOW_MS, FLASH_BOOT_SAMPLE_INTERVAL_MS);
constexpr unsigned long FLASH_BOOT_REQUIRED_LOW_SAMPLES =
    ceilPercentOf(FLASH_BOOT_SAMPLE_COUNT, FLASH_BOOT_REQUIRED_LOW_PERCENT);

static_assert(FLASH_BOOT_SAMPLE_INTERVAL_MS > 0, "FLASH boot sample interval must be greater than zero.");
static_assert(FLASH_BOOT_SAMPLE_COUNT >= FLASH_BOOT_MIN_SAMPLES,
              "FLASH boot detection window must collect the minimum number of samples.");

struct DeviceEntry {
  String model;
  String name;
  int8_t pin;
  bool state;
};

struct DeviceConfig {
  String mac;
  String hostname;
  String staSsid;
  String staPassword;
  String apPassword;
  float wifiPower;
  uint8_t numOutputs;
  DeviceEntry devices[MAX_DEVICES];
  // MQTT
  bool mqttEnabled;
  String mqttHost;
  uint16_t mqttPort;
  String mqttUser;
  String mqttPassword;
  bool arduinoOtaEnabled;
  // Debug
  bool debugSerial;
};

// Background load-action state machine
enum ActionPhase : uint8_t {
  APHASE_NONE = 0,
  APHASE_CYCLE_OFF,
  APHASE_CYCLE_ON,
  APHASE_FINAL_WAIT,
  APHASE_TRIGGER_OFF,
  APHASE_TRIGGER_ON,
  // Dedicated leave-mesh phases
  APHASE_LEAVE_MESH_PREP_ON,
  APHASE_LEAVE_MESH_CYCLE_OFF,
  APHASE_LEAVE_MESH_CYCLE_ON,
  APHASE_LEAVE_MESH_FINAL_WAIT,
  APHASE_LEAVE_MESH_TRIGGER_OFF,
  APHASE_LEAVE_MESH_TRIGGER_ON,
  // Dedicated factory-reset phases
  APHASE_FACTORY_RESET_PREP_ON,
  APHASE_FACTORY_RESET_CYCLE_OFF,
  APHASE_FACTORY_RESET_CYCLE_ON,
  APHASE_FACTORY_RESET_FINAL_WAIT,
  APHASE_FACTORY_RESET_TRIGGER_OFF,
  APHASE_FACTORY_RESET_TRIGGER_ON
};

struct ActiveAction {
  ActionPhase phase;
  uint8_t deviceIdx;
  uint8_t cyclesRemaining;
  unsigned long phaseStartMs;
  bool allOutputs;  // when true, action iterates all managed outputs sequentially
};

struct OutputReservation {
  bool reserved;
  String owner;
};

struct PinMapping {
  int8_t gpio;
  const char* label;
};

const PinMapping OUTPUT_PIN_MAPPINGS[] = {
    {16, "D0 (GPIO16)"},
    {5, "D1 (GPIO5)"},
    {4, "D2 (GPIO4)"},
    {0, "D3 (GPIO0)"},
    {2, "D4 (GPIO2)"},
    {14, "D5 (GPIO14)"},
    {12, "D6 (GPIO12)"},
    {13, "D7 (GPIO13)"},
    {15, "D8 (GPIO15)"},
    {3, "RX (GPIO3)"},
    {1, "TX (GPIO1)"},
};

constexpr size_t OUTPUT_PIN_MAPPING_COUNT = sizeof(OUTPUT_PIN_MAPPINGS) / sizeof(OUTPUT_PIN_MAPPINGS[0]);
static_assert(DEFAULT_D0_D7_COUNT <= OUTPUT_PIN_MAPPING_COUNT,
              "DEFAULT_D0_D7_COUNT exceeds available OUTPUT_PIN_MAPPINGS entries.");

DeviceConfig cfg;
ESP8266WebServer server(80);
File importFile;
bool importFailed = false;
bool otaUpdateFailed = false;
String otaUpdateError;
wl_status_t lastWifiStatus = WL_IDLE_STATUS;
unsigned long lastWifiReconnectAttemptMs = 0;
unsigned long lastWifiConnectLogMs = 0;
unsigned long wifiDisconnectSinceMs = 0;
uint8_t wifiRecoveryAttempts = 0;
bool outputsActivated = false;
bool outputActivationDeferredLogged = false;
unsigned long bootStartMillis = 0;
// MQTT
WiFiClient mqttWifiClient;
PubSubClient mqttClient(mqttWifiClient);
unsigned long lastMqttConnectAttemptMs = 0;
// Background load-action state
ActiveAction bgAction = {APHASE_NONE, 0, 0, 0UL, false};
OutputReservation outputReservations[MAX_DEVICES];

String htmlEscape(const String& value) {
  String out;
  out.reserve(value.length() + 16);
  for (size_t i = 0; i < value.length(); ++i) {
    char c = value[i];
    if (c == '&') out += F("&amp;");
    else if (c == '<') out += F("&lt;");
    else if (c == '>') out += F("&gt;");
    else if (c == '"') out += F("&quot;");
    else if (c == '\'') out += F("&#39;");
    else out += c;
  }
  return out;
}

const PinMapping* findPinMapping(int pin) {
  for (size_t i = 0; i < OUTPUT_PIN_MAPPING_COUNT; ++i) {
    if (OUTPUT_PIN_MAPPINGS[i].gpio == pin) {
      return &OUTPUT_PIN_MAPPINGS[i];
    }
  }
  return nullptr;
}

const char* wifiStatusToString(wl_status_t status) {
  switch (status) {
    case WL_CONNECTED:
      return "CONNECTED";
    case WL_NO_SSID_AVAIL:
      return "NO_SSID_AVAIL";
    case WL_CONNECT_FAILED:
      return "CONNECT_FAILED";
    case WL_WRONG_PASSWORD:
      return "WRONG_PASSWORD";
    case WL_IDLE_STATUS:
      return "IDLE";
    case WL_DISCONNECTED:
      return "DISCONNECTED";
    case WL_CONNECTION_LOST:
      return "CONNECTION_LOST";
    case WL_SCAN_COMPLETED:
      return "SCAN_COMPLETED";
    default:
      return "UNKNOWN";
  }
}

void restartDevice(const String& reason) {
  Serial.println();
  Serial.println(F("[FATAL] Unrecoverable condition encountered."));
  Serial.print(F("[FATAL] Reason: "));
  Serial.println(reason);
  Serial.println(F("[FATAL] Restarting device in 2 seconds..."));
  delay(2000);
  ESP.restart();
}

void logStatus(const String& message) {
  Serial.print(F("[INFO] "));
  Serial.println(message);
}

void logWarning(const String& message) {
  Serial.print(F("[WARN] "));
  Serial.println(message);
}


void logError(const String& message) {
  Serial.print(F("[ERROR] "));
  Serial.println(message);
}

String macLastThreeOctets(const String& mac) {
  int i1 = mac.indexOf(':');
  if (i1 < 0) return "000000";
  int i2 = mac.indexOf(':', i1 + 1);
  if (i2 < 0) return "000000";
  int i3 = mac.indexOf(':', i2 + 1);
  if (i3 < 0) return "000000";
  String tail = mac.substring(i3 + 1);
  tail.replace(":", "");
  tail.toUpperCase();
  return tail;
}

String defaultHostnameFromMac(const String& mac) {
  return String(F("C4-VIBRANT-")) + macLastThreeOctets(mac);
}

String defaultSoftApSsidFromMac(const String& mac) {
  return String(F("C4-VIBRANT-")) + macLastThreeOctets(mac);
}

void clearOutputReservations() {
  for (uint8_t i = 0; i < MAX_DEVICES; ++i) {
    outputReservations[i].reserved = false;
    outputReservations[i].owner = "";
  }
}

bool parseMac(const String& mac, uint8_t out[6]) {
  if (mac.length() != 17) return false;
  for (uint8_t i = 0; i < 6; ++i) {
    char hi = mac[i * 3];
    char lo = mac[i * 3 + 1];
    if (i < 5 && mac[i * 3 + 2] != ':') return false;
    auto hex = [](char c) -> int {
      if (c >= '0' && c <= '9') return c - '0';
      if (c >= 'A' && c <= 'F') return c - 'A' + 10;
      if (c >= 'a' && c <= 'f') return c - 'a' + 10;
      return -1;
    };
    int h = hex(hi);
    int l = hex(lo);
    if (h < 0 || l < 0) return false;
    out[i] = static_cast<uint8_t>((h << 4) | l);
  }
  return true;
}

bool applyConfiguredMac() {
  uint8_t mac[6] = {0};
  if (!parseMac(cfg.mac, mac)) {
    logWarning(F("Configured MAC address is invalid; continuing with hardware MAC."));
    return false;
  }

  bool stationOk = wifi_set_macaddr(STATION_IF, mac);
  bool apOk = wifi_set_macaddr(SOFTAP_IF, mac);
  if (!stationOk || !apOk) {
    logWarning(F("Failed to apply configured MAC address to one or more interfaces; continuing with hardware MAC."));
    return false;
  }

  logStatus(String(F("Applied MAC address: ")) + cfg.mac);
  return true;
}

void setFactoryDefaults() {
  cfg.mac = WiFi.softAPmacAddress();
  cfg.hostname = defaultHostnameFromMac(cfg.mac);
  cfg.staSsid = DEFAULT_STA_SSID;
  cfg.staPassword = DEFAULT_STA_PASSWORD;
  cfg.apPassword = DEFAULT_AP_PASSWORD;
  cfg.wifiPower = 20.5f;
  cfg.numOutputs = DEFAULT_NUM_OUTPUTS;

  // Default GPIO pins for D0-D7 in NodeMCU v3 order
  static const int8_t DEFAULT_OUTPUT_PINS[] = {16, 5, 4, 0, 2, 14, 12, 13};
  static const uint8_t DEFAULT_OUTPUT_PIN_COUNT =
      sizeof(DEFAULT_OUTPUT_PINS) / sizeof(DEFAULT_OUTPUT_PINS[0]);

  for (uint8_t i = 0; i < MAX_DEVICES; ++i) {
    cfg.devices[i].model = String(F("Model ")) + String(i + 1);
    cfg.devices[i].name = String(F("Output ")) + String(i + 1);
    // Map first DEFAULT_D0_D7_COUNT outputs to D0-D7 by default; rest unassigned.
    // Compile-time static_assert above guarantees DEFAULT_D0_D7_COUNT <= OUTPUT_PIN_MAPPING_COUNT.
    cfg.devices[i].pin = (i < DEFAULT_D0_D7_COUNT) ? OUTPUT_PIN_MAPPINGS[i].gpio : -1;
    cfg.devices[i].state = false;
    /*
    cfg.devices[i].pin = (i < DEFAULT_OUTPUT_PIN_COUNT) ? DEFAULT_OUTPUT_PINS[i] : -1;
    cfg.devices[i].state = false;
    */
  }

  cfg.mqttEnabled = false;
  cfg.mqttHost = "";
  cfg.mqttPort = DEFAULT_MQTT_PORT;
  cfg.mqttUser = "";
  cfg.mqttPassword = "";
  cfg.arduinoOtaEnabled = false;
  cfg.debugSerial = false;
  clearOutputReservations();

  logStatus(F("Factory defaults loaded."));
}

bool saveConfig() {
  JsonDocument doc;
  doc["mac"] = cfg.mac;
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

  JsonObject mqtt = doc["mqtt"].to<JsonObject>();
  mqtt["enabled"] = cfg.mqttEnabled;
  mqtt["host"] = cfg.mqttHost;
  mqtt["port"] = cfg.mqttPort;
  mqtt["user"] = cfg.mqttUser;
  mqtt["password"] = cfg.mqttPassword;

  JsonArray devices = doc["devices"].to<JsonArray>();
  for (uint8_t i = 0; i < MAX_DEVICES; ++i) {
    JsonObject d = devices.add<JsonObject>();
    d["model"] = cfg.devices[i].model;
    d["name"] = cfg.devices[i].name;
    d["pin"] = cfg.devices[i].pin;
    d["state"] = cfg.devices[i].state;
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
    logError(F("Unable to open configuration file; restoring factory defaults."));
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
  cfg.hostname = doc["hostname"] | defaultHostnameFromMac(cfg.mac);

  String legacySsid = doc["ssid"] | String(DEFAULT_STA_SSID);
  String legacyPassword = doc["password"] | String(DEFAULT_STA_PASSWORD);
  cfg.staSsid = doc["staSsid"] | legacySsid;
  cfg.staPassword = doc["staPassword"] | legacyPassword;
  cfg.apPassword = doc["apPassword"] | legacyPassword;
  cfg.wifiPower = doc["wifiPower"] | 20.5f;
  // Backward-compat: existing saved configs without numOutputs default to MAX_DEVICES
  // so no previously configured outputs are hidden unexpectedly.
  cfg.numOutputs = static_cast<uint8_t>(doc["numOutputs"] | static_cast<uint8_t>(MAX_DEVICES));
  if (cfg.numOutputs < 1 || cfg.numOutputs > MAX_DEVICES) cfg.numOutputs = MAX_DEVICES;

  JsonArray devices = doc["devices"].as<JsonArray>();
  for (uint8_t i = 0; i < MAX_DEVICES; ++i) {
    if (i < devices.size()) {
      JsonObject d = devices[i];
      cfg.devices[i].model = d["model"] | String(F("Model ")) + String(i + 1);
      cfg.devices[i].name = d["name"] | String(F("Output ")) + String(i + 1);
      cfg.devices[i].pin = static_cast<int8_t>(d["pin"] | -1);
      cfg.devices[i].state = false;  // Always boot OFF; do not restore runtime ON state
    } else {
      cfg.devices[i].model = String(F("Model ")) + String(i + 1);
      cfg.devices[i].name = String(F("Output ")) + String(i + 1);
      cfg.devices[i].pin = -1;
      cfg.devices[i].state = false;
    }
  }
  clearOutputReservations();

  if (cfg.staSsid.isEmpty()) cfg.staSsid = DEFAULT_STA_SSID;
  if (cfg.staPassword.isEmpty()) cfg.staPassword = DEFAULT_STA_PASSWORD;
  if (cfg.apPassword.isEmpty()) cfg.apPassword = DEFAULT_AP_PASSWORD;
  if (cfg.hostname.isEmpty()) cfg.hostname = defaultHostnameFromMac(cfg.mac);

  cfg.mqttEnabled = doc["mqttEnabled"] | false;
  cfg.mqttHost = doc["mqttHost"] | String("");
  cfg.mqttPort = static_cast<uint16_t>(doc["mqttPort"] | static_cast<uint16_t>(DEFAULT_MQTT_PORT));
  cfg.mqttUser = doc["mqttUser"] | String("");
  cfg.mqttPassword = doc["mqttPassword"] | String("");
  if (cfg.mqttPort == 0) cfg.mqttPort = DEFAULT_MQTT_PORT;
  cfg.arduinoOtaEnabled = doc["arduinoOtaEnabled"] | false;
  cfg.debugSerial = doc["debugSerial"] | false;

  logStatus(F("Configuration loaded successfully."));
  return true;
}

String pinLabel(int8_t pin) {
  const PinMapping* mapping = findPinMapping(pin);
  return mapping ? String(mapping->label) : String(F("none"));
}

bool isValidOutputPin(int8_t pin) {
  return findPinMapping(pin) != nullptr;
}

void logDeviceSummary() {
  Serial.println(F("[INFO] Configured outputs:"));
  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    Serial.print(F("  ["));
    Serial.print(i + 1);
    Serial.print(F("] Model='"));
    Serial.print(cfg.devices[i].model);
    Serial.print(F("' Name='"));
    Serial.print(cfg.devices[i].name);
    Serial.print(F("' Pin="));
    Serial.print(pinLabel(cfg.devices[i].pin));
    Serial.print(F(" State="));
    Serial.println(cfg.devices[i].state ? F("ON") : F("OFF"));
  }
}

void applyOutputsNow() {
  logStatus(F("Applying output states..."));
  for (uint8_t i = 0; i < MAX_DEVICES; ++i) {
    int8_t pin = cfg.devices[i].pin;
    if (!isValidOutputPin(pin)) {
      continue;
    }
    pinMode(pin, OUTPUT);
    // Inverted output logic: logical ON -> LOW, logical OFF -> HIGH (active-low).
    digitalWrite(pin, cfg.devices[i].state ? LOW : HIGH);

  }
  outputsActivated = true;
  outputActivationDeferredLogged = false;
  logDeviceSummary();
}

bool outputActivationDelayElapsed() {
  // Unsigned subtraction keeps this short post-boot elapsed-time check valid
  // even if millis() later wraps around.
  return (millis() - bootStartMillis) >= OUTPUT_BOOT_ACTIVATION_DELAY_MS;
}

void logDeferredOutputActivation() {
  if (outputActivationDeferredLogged) return;
  logStatus(String(F("Deferring output activation until ")) + String(OUTPUT_BOOT_ACTIVATION_DELAY_MS) +
            F(" ms after boot."));
  outputActivationDeferredLogged = true;
}

void applyOutputsWhenSafe() {
  if (outputsActivated) return;
  if (!outputActivationDelayElapsed()) {
    logDeferredOutputActivation();
    return;
  }
  logStatus(F("Post-boot output activation delay elapsed."));
  applyOutputsNow();
}

void refreshOutputsForCurrentBootPhase() {
  if (outputsActivated || outputActivationDelayElapsed()) {
    applyOutputsNow();
    return;
  }
  logStatus(F("Output configuration updated during boot delay; hardware activation remains deferred."));
  outputActivationDeferredLogged = false;
  logDeferredOutputActivation();
}

void prepareOutputsForBootPhase() {
  if (outputsActivated) return;
  if (outputActivationDelayElapsed()) {
    applyOutputsNow();
    return;
  }
  logDeferredOutputActivation();
}

void logWifiSummary(const String& softApSsid) {
  Serial.print(F("[INFO] SoftAP SSID: "));
  Serial.println(softApSsid);
  Serial.print(F("[INFO] Station target SSID: "));
  Serial.println(cfg.staSsid);
  Serial.print(F("[INFO] Hostname: "));
  Serial.println(cfg.hostname);
  Serial.print(F("[INFO] Software version: "));
  Serial.println(SOFTWARE_VERSION);
  Serial.print(F("[INFO] Wi-Fi power: "));
  Serial.print(cfg.wifiPower, 1);
  Serial.println(F(" dBm"));
  Serial.print(F("[INFO] SoftAP IP: "));
  Serial.println(WiFi.softAPIP());
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("[INFO] Station connected. IP: "));
    Serial.println(WiFi.localIP());
  } else {
    Serial.print(F("[INFO] Station status: "));
    Serial.println(wifiStatusToString(WiFi.status()));
  }
}

void resetWifiRecoveryState() {
  wifiDisconnectSinceMs = 0;
  wifiRecoveryAttempts = 0;
}

void performFactoryResetAndRestart(const String& reason) {
  logWarning(reason);
  setFactoryDefaults();
  if (!saveConfig()) {
    restartDevice(F("Failed to persist factory defaults during requested reset."));
  }
  delay(500);
  ESP.restart();
}

// ---------------------------------------------------------------------------
// DIAGNOSTICS — log the effective loaded Wi-Fi configuration values.
// Remove this function (and its call in setup()) once the Wi-Fi issue is
// resolved.
// ---------------------------------------------------------------------------
void logLoadedWifiConfig() {
  Serial.print(F("[INFO] [DIAG] Loaded station SSID: "));
  Serial.println(cfg.staSsid);
  Serial.print(F("[INFO] [DIAG] Loaded Wi-Fi power: "));
  Serial.print(cfg.wifiPower, 1);
  Serial.println(F(" dBm"));
}

bool detectStableFlashPressDuringBoot() {
  unsigned long lowSamples = 0;

  // This short blocking window is intentional: GPIO0 is only sampled during boot,
  // not polled continuously at runtime.
  for (unsigned long i = 0; i < FLASH_BOOT_SAMPLE_COUNT; ++i) {
    if (i > 0) {
      delay(FLASH_BOOT_SAMPLE_INTERVAL_MS);
    }
    if (digitalRead(FLASH_BUTTON_PIN) == LOW) {
      ++lowSamples;
    }
  }

  Serial.print(F("[INFO] [FLASH] Boot-time samples low="));
  Serial.print(lowSamples);
  Serial.print(F("/"));
  Serial.println(FLASH_BOOT_SAMPLE_COUNT);

  return lowSamples >= FLASH_BOOT_REQUIRED_LOW_SAMPLES;
}

void checkFlashFactoryResetOnBoot() {
  logStatus(F("[FLASH] Checking boot-time FLASH/GPIO0 factory reset trigger..."));
  bool stablePressed = detectStableFlashPressDuringBoot();

  Serial.print(F("[INFO] [FLASH] Stable pressed condition "));
  Serial.println(stablePressed ? F("detected.") : F("not detected."));

  if (!stablePressed) {
    logStatus(F("[FLASH] Boot-time FLASH/GPIO0 reset skipped."));
    return;
  }

  logWarning(F("[FLASH] Boot-time FLASH/GPIO0 reset trigger detected. Applying factory defaults."));
  performFactoryResetAndRestart(F("FLASH/GPIO0 was held low during the boot-time detection window."));
}

// ---------------------------------------------------------------------------
// DIAGNOSTICS — scan for visible Wi-Fi networks and print SSID + RSSI.
// Remove this function (and its call in setup()) once the Wi-Fi issue is
// resolved.
// ---------------------------------------------------------------------------
void logWifiScan() {
  logStatus(F("[DIAG] Scanning for visible Wi-Fi networks (may take a few seconds)..."));
  int n = WiFi.scanNetworks();
  if (n <= 0) {
    logStatus(F("[DIAG] No Wi-Fi networks found during scan."));
  } else {
    Serial.print(F("[INFO] [DIAG] Found "));
    Serial.print(n);
    Serial.println(F(" network(s):"));
    for (int i = 0; i < n; ++i) {
      Serial.print(F("[INFO] [DIAG]   SSID: \""));
      Serial.print(WiFi.SSID(i));
      Serial.print(F("\"  RSSI: "));
      Serial.print(WiFi.RSSI(i));
      Serial.println(F(" dBm"));
    }
  }
  WiFi.scanDelete();
}

void applyWifiSettings() {
  logStatus(F("Applying Wi-Fi settings..."));
#if WIFI_DIAG_APPLY_CUSTOM_MAC
  applyConfiguredMac();
#else
  logWarning(F("[DIAG] Custom MAC override disabled for diagnostics; using hardware MAC."));
#endif
  cfg.wifiPower = constrain(cfg.wifiPower, MIN_WIFI_POWER, MAX_WIFI_POWER);

  const String softApSsid = defaultSoftApSsidFromMac(cfg.mac);

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP_STA);
  WiFi.hostname(cfg.hostname);
  WiFi.setAutoReconnect(true);
  WiFi.setOutputPower(cfg.wifiPower);

  bool apStarted = WiFi.softAP(softApSsid.c_str(), cfg.apPassword.c_str());
  if (!apStarted) {
    restartDevice(F("Failed to start SoftAP with configured credentials."));
  }

  logStatus(String(F("Starting station connection to SSID: ")) + cfg.staSsid);
  WiFi.begin(cfg.staSsid.c_str(), cfg.staPassword.c_str());
  resetWifiRecoveryState();
  lastWifiStatus = WiFi.status();
  logWifiSummary(softApSsid);
}

String pinOption(int selectedPin, const PinMapping& mapping) {
  String selected = (selectedPin == mapping.gpio) ? " selected" : "";
  return "<option value=\"" + String(mapping.gpio) + "\"" + selected + ">" + String(mapping.label) + "</option>";
}

bool parsePinValue(const String& raw, int& pin) {
  if (raw == "-1") {
    pin = -1;
    return true;
  }
  if (raw.isEmpty()) return false;
  for (size_t i = 0; i < raw.length(); ++i) {
    if (raw[i] < '0' || raw[i] > '9') return false;
  }
  pin = raw.toInt();
  return isValidOutputPin(pin);
}

bool parseIndexValue(const String& raw, int& value) {
  if (raw.isEmpty()) return false;
  for (size_t i = 0; i < raw.length(); ++i) {
    if (raw[i] < '0' || raw[i] > '9') return false;
  }
  value = raw.toInt();
  return true;
}

bool parseFloatValue(const String& raw, float& value) {
  if (raw.isEmpty()) return false;
  char* endPtr = nullptr;
  value = strtof(raw.c_str(), &endPtr);
  return endPtr != raw.c_str() && *endPtr == '\0';
}

bool usingFactoryPassword() {
  return cfg.apPassword == DEFAULT_AP_PASSWORD || cfg.staPassword == DEFAULT_STA_PASSWORD;
}

String passwordWarningHtml() {
  return F("<p style='color:#b00020;'><strong>Warning:</strong> Factory default Wi-Fi password is active. "
           "Change it now for security.</p>");
}

bool ensureAuthorized() {
  if (server.authenticate("admin", cfg.apPassword.c_str())) return true;
  server.requestAuthentication();
  return false;
}

bool arduinoOtaActive = false;
bool arduinoOtaCallbacksConfigured = false;

void applyArduinoOtaSettings() {
  if (!cfg.arduinoOtaEnabled) {
    if (arduinoOtaActive) {
      logStatus(F("ArduinoOTA disabled in settings. OTA request handling is now paused."));
    } else {
      logStatus(F("ArduinoOTA is disabled."));
    }
    arduinoOtaActive = false;
    return;
  }

  ArduinoOTA.setHostname(cfg.hostname.c_str());
  ArduinoOTA.setPassword(cfg.apPassword.c_str());

  if (!arduinoOtaCallbacksConfigured) {
    ArduinoOTA.onStart([]() {
      String mode = (ArduinoOTA.getCommand() == U_FLASH) ? F("firmware") : F("filesystem");
      logStatus(String(F("ArduinoOTA start (")) + mode + F(")."));
      if (cfg.debugSerial) {
        Serial.print(F("[DEBUG] ArduinoOTA host: "));
        Serial.println(cfg.hostname);
      }
    });
    ArduinoOTA.onEnd([]() {
      logStatus(F("ArduinoOTA completed."));
      if (cfg.debugSerial) {
        Serial.println(F("[DEBUG] ArduinoOTA transfer finished successfully."));
      }
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      if (cfg.debugSerial) {
        unsigned int percent = (total == 0U) ? 0U : (progress * 100U) / total;
        Serial.print(F("[DEBUG] ArduinoOTA progress: "));
        Serial.print(percent);
        Serial.println(F("%"));
      }
    });
    ArduinoOTA.onError([](ota_error_t error) {
      logError(String(F("ArduinoOTA error #")) + String(static_cast<int>(error)));
      if (cfg.debugSerial) {
        Serial.println(F("[DEBUG] ArduinoOTA transfer aborted due to error."));
      }
    });
    arduinoOtaCallbacksConfigured = true;
  }

  ArduinoOTA.begin();
  arduinoOtaActive = true;
  logStatus(String(F("ArduinoOTA enabled on hostname: ")) + cfg.hostname);
}

// ---------------------------------------------------------------------------
// Background action helpers
// ---------------------------------------------------------------------------

bool isActionRunning() {
  return bgAction.phase != APHASE_NONE;
}

String actionPhaseName() {
  switch (bgAction.phase) {
    case APHASE_CYCLE_OFF:   return F("cycling off");
    case APHASE_CYCLE_ON:    return F("cycling on");
    case APHASE_FINAL_WAIT:  return F("waiting (final)");
    case APHASE_TRIGGER_OFF: return F("triggering off");
    case APHASE_TRIGGER_ON:  return F("triggering on");
    case APHASE_LEAVE_MESH_PREP_ON:    return F("leave mesh prep on");
    case APHASE_LEAVE_MESH_CYCLE_OFF:  return F("leave mesh cycling off");
    case APHASE_LEAVE_MESH_CYCLE_ON:   return F("leave mesh cycling on");
    case APHASE_LEAVE_MESH_FINAL_WAIT: return F("leave mesh waiting");
    case APHASE_LEAVE_MESH_TRIGGER_OFF: return F("leave mesh trigger off");
    case APHASE_LEAVE_MESH_TRIGGER_ON:  return F("leave mesh trigger on");
    case APHASE_FACTORY_RESET_PREP_ON:    return F("factory reset prep on");
    case APHASE_FACTORY_RESET_CYCLE_OFF:  return F("factory reset cycling off");
    case APHASE_FACTORY_RESET_CYCLE_ON:   return F("factory reset cycling on");
    case APHASE_FACTORY_RESET_FINAL_WAIT: return F("factory reset waiting");
    case APHASE_FACTORY_RESET_TRIGGER_OFF: return F("factory reset trigger off");
    case APHASE_FACTORY_RESET_TRIGGER_ON:  return F("factory reset trigger on");
    default:                 return F("idle");
  }
}

// Drive a single output pin directly (no blocking).
void setOutputDirect(uint8_t idx, bool state) {
  if (idx >= MAX_DEVICES) return;
  DeviceEntry& d = cfg.devices[idx];
  d.state = state;
  if (outputsActivated && isValidOutputPin(d.pin)) {
    pinMode(d.pin, OUTPUT);
    // Inverted output logic: logical ON -> LOW, logical OFF -> HIGH (active-low).
    digitalWrite(d.pin, state ? LOW : HIGH);
  }
}

bool isManagedOutput(uint8_t idx) {
  return idx < cfg.numOutputs && isValidOutputPin(cfg.devices[idx].pin);
}


// ---------------------------------------------------------------------------
// MQTT helpers
// ---------------------------------------------------------------------------

String mqttOutputStateTopic(uint8_t idx) {
  return String(F("vibrant/")) + cfg.hostname + "/out/" + String(idx) + "/state";
}

String mqttOutputSetTopic(uint8_t idx) {
  return String(F("vibrant/")) + cfg.hostname + "/out/" + String(idx) + "/set";
}

String mqttOutputActionTopic(uint8_t idx) {
  return String(F("vibrant/")) + cfg.hostname + "/out/" + String(idx) + "/action";

}


void mqttPublishOutputState(uint8_t idx) {
  if (!cfg.mqttEnabled || !mqttClient.connected()) return;
  if (idx >= cfg.numOutputs) return;
  const char* statePayload = cfg.devices[idx].state ? "ON" : "OFF";
  String topic = mqttOutputStateTopic(idx);
  bool ok = mqttClient.publish(topic.c_str(), statePayload, true);
  if (ok) {
    Serial.print(F("[INFO] [MQTT] Published state -> topic: "));
    Serial.print(topic);
    Serial.print(F(" payload: "));
    Serial.println(statePayload);
  } else {
    Serial.print(F("[WARN] [MQTT] Failed to publish state -> topic: "));
    Serial.println(topic);
  }
}

void mqttPublishAllOutputStates() {
  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    mqttPublishOutputState(i);
  }
}

// Drive outputs for the active background action (all managed outputs or just the active one).
void setOutputsForAction(bool state) {
  if (bgAction.allOutputs) {
    for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
      if (isManagedOutput(i)) {
        setOutputDirect(i, state);
        mqttPublishOutputState(i);
      }
    }
  } else {
    setOutputDirect(bgAction.deviceIdx, state);
    mqttPublishOutputState(bgAction.deviceIdx);
  }
}

int findNextManagedOutputIndex(uint8_t startIdx) {
  if (startIdx >= cfg.numOutputs) return -1;
  for (uint8_t i = startIdx; i < cfg.numOutputs; ++i) {
    if (isManagedOutput(i)) return static_cast<int>(i);
  }
  return -1;
}

void setActionOutputState(uint8_t idx, bool state) {
  setOutputDirect(idx, state);
  mqttPublishOutputState(idx);
}

// Set output state for the current action: writes all managed outputs when
// bgAction.allOutputs is true (parallel-all mode), otherwise writes only the
// single output at bgAction.deviceIdx.  Always call this before updating
// bgAction.phaseStartMs so the timer starts after all writes complete.
void setPhaseOutputState(bool state) {
  if (bgAction.allOutputs) {
    for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
      if (isManagedOutput(i)) {
        setOutputDirect(i, state);
        mqttPublishOutputState(i);
      }
    }
  } else {
    setOutputDirect(bgAction.deviceIdx, state);
    mqttPublishOutputState(bgAction.deviceIdx);
  }
}

void beginLeaveMeshForOutput(uint8_t idx, bool allOutputsAction) {
  bgAction.deviceIdx = idx;
  bgAction.allOutputs = allOutputsAction;
  bgAction.cyclesRemaining = LEAVE_MESH_CYCLES;
  bgAction.phaseStartMs = millis();
  if (!cfg.devices[idx].state) {
    if (!allOutputsAction) {
      logStatus(String(F("leave_mesh: output was OFF, turning ON for prep")));
    }
    if (cfg.debugSerial) {
      Serial.print(F("[DEBUG] leave_mesh"));
      if (allOutputsAction) Serial.print(F("_all"));
      Serial.print(F(": output "));
      Serial.print(idx + 1);
      Serial.println(F(" was OFF, turning ON for prep"));
    }
    setActionOutputState(idx, true);
    bgAction.phase = APHASE_LEAVE_MESH_PREP_ON;
  } else {
    if (!allOutputsAction) {
      logStatus(String(F("leave_mesh: output was ON, starting cycles immediately")));
    }
    if (cfg.debugSerial) {
      Serial.print(F("[DEBUG] leave_mesh"));
      if (allOutputsAction) Serial.print(F("_all"));
      Serial.print(F(": output "));
      Serial.print(idx + 1);
      Serial.println(F(" was ON, starting cycles immediately"));
    }
    setActionOutputState(idx, false);
    bgAction.phase = APHASE_LEAVE_MESH_CYCLE_OFF;
  }
}

void beginFactoryResetForOutput(uint8_t idx, bool allOutputsAction) {
  bgAction.deviceIdx = idx;
  bgAction.allOutputs = allOutputsAction;
  bgAction.cyclesRemaining = FACTORY_RESET_CYCLES;
  bgAction.phaseStartMs = millis();
  if (!cfg.devices[idx].state) {
    if (!allOutputsAction) {
      logStatus(String(F("factory_reset: output was OFF, turning ON for prep")));
    }
    if (cfg.debugSerial) {
      Serial.print(F("[DEBUG] factory_reset"));
      if (allOutputsAction) Serial.print(F("_all"));
      Serial.print(F(": output "));
      Serial.print(idx + 1);
      Serial.println(F(" was OFF, turning ON for prep"));
    }
    setActionOutputState(idx, true);
    bgAction.phase = APHASE_FACTORY_RESET_PREP_ON;
  } else {
    if (!allOutputsAction) {
      logStatus(String(F("factory_reset: output was ON, starting cycles immediately")));
    }
    if (cfg.debugSerial) {
      Serial.print(F("[DEBUG] factory_reset"));
      if (allOutputsAction) Serial.print(F("_all"));
      Serial.print(F(": output "));
      Serial.print(idx + 1);
      Serial.println(F(" was ON, starting cycles immediately"));
    }
    setActionOutputState(idx, false);
    bgAction.phase = APHASE_FACTORY_RESET_CYCLE_OFF;
  }
}



String stickserverMacToken() {
  String token;
  token.reserve(12);
  for (size_t i = 0; i < cfg.mac.length(); ++i) {
    char c = cfg.mac[i];
    if (isxdigit(static_cast<unsigned char>(c))) {
      token += static_cast<char>(tolower(static_cast<unsigned char>(c)));
    }
  }
  if (token.isEmpty()) token = F("unknown");
  return token;
}

String stickserverIdToken() {
  String token;
  token.reserve(cfg.hostname.length() + 8);
  for (size_t i = 0; i < cfg.hostname.length(); ++i) {
    char c = cfg.hostname[i];
    if (isalnum(static_cast<unsigned char>(c))) {
      token += static_cast<char>(tolower(static_cast<unsigned char>(c)));
    } else if (c == '-' || c == '_') {
      token += c;
    } else if (!token.endsWith("-")) {
      token += '-';
    }
  }
  while (token.endsWith("-")) {
    token.remove(token.length() - 1);
  }
  return token;
}

String stickserverInstanceId() {
  String instanceId = String(F("ssvr-")) + stickserverMacToken();
  String idToken = stickserverIdToken();
  if (!idToken.isEmpty()) {
    instanceId += '-';
    instanceId += idToken;
  }
  return instanceId;
}

String stickserverInstanceTopic() {
  return String(STICKSERVER_ROOT_TOPIC) + '/' + stickserverInstanceId();
}

uint8_t managedOutputCount() {
  uint8_t count = 0;
  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    if (isManagedOutput(i)) ++count;
  }
  return count;
}

uint8_t availableManagedOutputCount() {
  uint8_t count = 0;
  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    if (isManagedOutput(i) && !outputReservations[i].reserved) ++count;
  }
  return count;
}

String stickserverOutputEuid(uint8_t idx) {
  return String(F("vibrant-")) + stickserverMacToken() + F("-out-") + String(idx);
}

int findManagedOutputByEuid(const String& euid) {
  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    if (isManagedOutput(i) && euid == stickserverOutputEuid(i)) {
      return i;
    }
  }
  return -1;
}

void populateStickserverDevice(JsonObject obj, uint8_t idx) {
  obj["euid"] = stickserverOutputEuid(idx);
  obj["idx"] = idx;
  obj["name"] = cfg.devices[idx].name;
  obj["model"] = cfg.devices[idx].model;
  obj["pin"] = cfg.devices[idx].pin;
  obj["state"] = cfg.devices[idx].state ? "ON" : "OFF";
  obj["ntype"] = STICKSERVER_OUTPUT_TYPE;
  obj["reserved"] = outputReservations[idx].reserved;
  obj["available"] = isManagedOutput(idx) && !outputReservations[idx].reserved;
  if (outputReservations[idx].reserved && !outputReservations[idx].owner.isEmpty()) {
    obj["owner"] = outputReservations[idx].owner;
  }
}

void buildStickserverEnvelope(JsonDocument& doc,
                              const String& rsp,
                              int ver,
                              const String& mid,
                              const char* status) {
  doc["rsp"] = rsp;
  doc["ver"] = ver;
  doc["mid"] = mid;
  doc["status"] = status;
}

bool publishStickserverResponse(JsonDocument& doc) {
  if (!cfg.mqttEnabled || !mqttClient.connected()) return false;
  String payload;
  if (serializeJson(doc, payload) == 0) {
    logWarning(F("Stickserver: failed to serialize response JSON."));
    return false;
  }
  String topic = stickserverInstanceTopic();
  bool ok = mqttClient.publish(topic.c_str(), payload.c_str());
  if (ok) {
    Serial.print(F("[INFO] [MQTT] Stickserver response -> topic: "));
    Serial.print(topic);
    Serial.print(F(" rsp: "));
    Serial.print(doc["rsp"] | "?");
    Serial.print(F(" status: "));
    Serial.println(doc["status"] | "?");
  } else {
    Serial.print(F("[WARN] [MQTT] Stickserver publish failed -> topic: "));
    Serial.println(topic);
  }
  return ok;
}

void publishStickserverFailure(const String& rsp,
                               int ver,
                               const String& mid,
                               const String& error,
                               const String& member,
                               const String& message) {
  JsonDocument doc;
  buildStickserverEnvelope(doc, rsp, ver, mid, "error");
  doc["error"] = error;
  if (!member.isEmpty()) doc["member"] = member;
  if (!message.isEmpty()) doc["message"] = message;
  publishStickserverResponse(doc);
}

bool extractStringArray(JsonVariantConst value, String out[], size_t& count) {
  count = 0;
  if (value.is<JsonArrayConst>()) {
    JsonArrayConst array = value.as<JsonArrayConst>();
    for (JsonVariantConst item : array) {
      if (!item.is<const char*>()) return false;
      if (count >= MAX_DEVICES) return false;
      out[count] = item.as<String>();
      if (out[count].isEmpty()) return false;
      ++count;
    }
    return count > 0;
  }
  if (value.is<const char*>()) {
    out[0] = value.as<String>();
    count = out[0].isEmpty() ? 0 : 1;
    return count == 1;
  }
  return false;
}

const char* aggregateStickserverStatus(size_t okCount, size_t totalCount) {
  if (okCount == 0) return "not_found";
  if (okCount == totalCount) return "ok";
  return "partial";
}

void handleStickserverMessage(const String& topicStr, const String& payloadStr) {
  JsonDocument request;
  DeserializationError err = deserializeJson(request, payloadStr);
  if (err) {
    Serial.print(F("[WARN] [MQTT] Stickserver JSON parse error: "));
    Serial.println(err.c_str());
    publishStickserverFailure(F("error"),
                              STICKSERVER_PROTOCOL_VERSION,
                              String(),
                              F("invalid_json"),
                              String(),
                              err.c_str());
    return;
  }
  if (request["rsp"].is<const char*>()) return;

  String cmd = request["cmd"] | String("");
  String mid = "";
  if (request["mid"].is<const char*>()) {
    mid = request["mid"].as<const char*>();
  } else if (request["mid"].is<int>()) {
    mid = String(request["mid"].as<int>());
  } else if (request["mid"].is<long>()) {
    mid = String(request["mid"].as<long>());
  }
  Serial.print(F("[INFO] [MQTT] Stickserver cmd: "));
  Serial.print(cmd.isEmpty() ? String(F("(none)")) : cmd.c_str());
  Serial.print(F(" mid: "));
  Serial.println(mid.isEmpty() ? String(F("(none)")) : mid.c_str());
  if (!request["ver"].is<int>()) {
    publishStickserverFailure(cmd.isEmpty() ? String(F("error")) : cmd,
                              STICKSERVER_PROTOCOL_VERSION,
                              mid,
                              F("invalid_member"),
                              F("ver"),
                              F("Missing or invalid ver."));
    return;
  }
  int ver = request["ver"].as<int>();
  if (mid.isEmpty()) {
    publishStickserverFailure(cmd.isEmpty() ? String(F("error")) : cmd,
                              ver,
                              mid,
                              F("invalid_member"),
                              F("mid"),
                              F("Missing or invalid mid."));
    return;
  }
  if (cmd.isEmpty()) {
    publishStickserverFailure(F("error"),
                              ver,
                              mid,
                              F("invalid_member"),
                              F("cmd"),
                              F("Missing or invalid cmd."));
    return;
  }
  if (topicStr == STICKSERVER_ROOT_TOPIC && cmd != F("hello")) {
    publishStickserverFailure(cmd,
                              ver,
                              mid,
                              F("invalid_command"),
                              F("cmd"),
                              F("Only hello is accepted on the root topic."));
    return;
  }

  // Shared euid parser: accepts string or integer and normalizes to String.
  // Logs unrecognized types when cfg.debugSerial is enabled.
  auto parseEuidValue = [&](JsonVariantConst v) -> String {
    if (v.is<const char*>()) return v.as<const char*>();
    if (v.is<int>()) return String(v.as<int>());
    if (v.is<long>()) return String(v.as<long>());
    if (v.is<unsigned int>()) return String(v.as<unsigned int>());
    if (v.is<unsigned long>()) return String(v.as<unsigned long>());
    if (cfg.debugSerial && !v.isNull()) {
      Serial.println(F("[WARN] [MQTT] parseEuidValue: unrecognized euid type; treating as missing."));
    }
    return String("");
  };

  if (cmd == F("hello")) {
    JsonDocument response;
    buildStickserverEnvelope(response, cmd, ver, mid, "ok");
    response["id"] = cfg.hostname;
    response["mac"] = cfg.mac;
    response["topic"] = stickserverInstanceTopic();
    response["instance"] = stickserverInstanceId();
    response["count"] = managedOutputCount();
    response["available"] = availableManagedOutputCount();
    publishStickserverResponse(response);
    return;
  }

  if (cmd == F("list")) {
    JsonDocument response;
    buildStickserverEnvelope(response, cmd, ver, mid, "ok");
    response["id"] = cfg.hostname;
    response["mac"] = cfg.mac;
    response["count"] = managedOutputCount();
    response["available"] = availableManagedOutputCount();
    JsonArray devices = response["devices"].to<JsonArray>();
    for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
      if (!isManagedOutput(i)) continue;
      populateStickserverDevice(devices.add<JsonObject>(), i);
    }
    publishStickserverResponse(response);
    return;
  }

  if (cmd == F("reserve")) {
    String owner = request["owner"] | String("");
    String ntype = request["ntype"] | String("");
    if (owner.isEmpty()) {
      publishStickserverFailure(cmd, ver, mid, F("invalid_member"), F("owner"), F("Missing owner."));
      return;
    }
    if (ntype.isEmpty()) {
      publishStickserverFailure(cmd, ver, mid, F("invalid_member"), F("ntype"), F("Missing ntype."));
      return;
    }
    if (!request["count"].is<int>()) {
      publishStickserverFailure(cmd, ver, mid, F("invalid_member"), F("count"), F("Missing or invalid count."));
      return;
    }
    int requestedCount = request["count"].as<int>();
    if (requestedCount < 1) {
      publishStickserverFailure(cmd, ver, mid, F("invalid_member"), F("count"), F("count must be >= 1."));
      return;
    }

    uint8_t reservedIdx[MAX_DEVICES] = {0};
    bool newReservation[MAX_DEVICES] = {false};
    size_t reservedCount = 0;
    for (uint8_t i = 0; i < cfg.numOutputs && reservedCount < static_cast<size_t>(requestedCount); ++i) {
      if (!isManagedOutput(i)) continue;
      if (outputReservations[i].reserved && outputReservations[i].owner == owner) {
        reservedIdx[reservedCount] = i;
        newReservation[reservedCount] = false;
        ++reservedCount;
      }
    }
    for (uint8_t i = 0; i < cfg.numOutputs && reservedCount < static_cast<size_t>(requestedCount); ++i) {
      if (!isManagedOutput(i) || outputReservations[i].reserved) continue;
      outputReservations[i].reserved = true;
      outputReservations[i].owner = owner;
      reservedIdx[reservedCount] = i;
      newReservation[reservedCount] = true;
      ++reservedCount;
    }

    JsonDocument response;
    buildStickserverEnvelope(response,
                             cmd,
                             ver,
                             mid,
                             reservedCount == static_cast<size_t>(requestedCount)
                                 ? "ok"
                                 : (reservedCount > 0 ? "partial" : "unavailable"));
    response["owner"] = owner;
    response["ntype"] = ntype;
    response["requested"] = requestedCount;
    response["allocated"] = reservedCount;
    response["available"] = availableManagedOutputCount();
    JsonArray devices = response["devices"].to<JsonArray>();
    for (size_t i = 0; i < reservedCount; ++i) {
      JsonObject device = devices.add<JsonObject>();
      populateStickserverDevice(device, reservedIdx[i]);
      device["new"] = newReservation[i];
    }
    publishStickserverResponse(response);
    return;
  }

  if (cmd == F("release")) {
    String owner = request["owner"] | String("");
    String euids[MAX_DEVICES];
    size_t euidCount = 0;
    bool hasEuids = !request["euids"].isNull();
    if (hasEuids && !extractStringArray(request["euids"], euids, euidCount)) {
      publishStickserverFailure(cmd, ver, mid, F("invalid_member"), F("euids"), F("Invalid euids list."));
      return;
    }
    if (owner.isEmpty() && euidCount == 0) {
      publishStickserverFailure(cmd,
                                ver,
                                mid,
                                F("invalid_member"),
                                F("owner/euids"),
                                F("Release requires owner and/or euids."));
      return;
    }

    bool selected[MAX_DEVICES] = {false};
    JsonDocument response;
    JsonArray devices = response["devices"].to<JsonArray>();
    if (!owner.isEmpty()) {
      for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
        if (isManagedOutput(i) && outputReservations[i].reserved && outputReservations[i].owner == owner) {
          selected[i] = true;
        }
      }
      response["owner"] = owner;
    }
    for (size_t i = 0; i < euidCount; ++i) {
      int idx = findManagedOutputByEuid(euids[i]);
      if (idx < 0) {
        JsonObject device = devices.add<JsonObject>();
        device["euid"] = euids[i];
        device["released"] = false;
        device["status"] = "unknown_euid";
      } else {
        selected[idx] = true;
      }
    }

    size_t releasedCount = 0;
    size_t selectedCount = 0;
    for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
      if (!selected[i]) continue;
      ++selectedCount;
      JsonObject device = devices.add<JsonObject>();
      populateStickserverDevice(device, i);
      if (outputReservations[i].reserved) {
        outputReservations[i].reserved = false;
        outputReservations[i].owner = "";
        device["released"] = true;
        device["status"] = "ok";
        device["available"] = true;
        device.remove("owner");
        ++releasedCount;
      } else {
        device["released"] = false;
        device["status"] = "not_reserved";
      }
    }

    buildStickserverEnvelope(response,
                             cmd,
                             ver,
                             mid,
                             releasedCount == selectedCount && devices.size() == selectedCount
                                 ? "ok"
                                 : (releasedCount > 0 ? "partial" : "not_found"));
    response["released"] = releasedCount;
    response["available"] = availableManagedOutputCount();
    publishStickserverResponse(response);
    return;
  }

  if (cmd == F("status")) {
    String euids[MAX_DEVICES];
    size_t euidCount = 0;
    if (!extractStringArray(request["euids"], euids, euidCount)) {
      publishStickserverFailure(cmd, ver, mid, F("invalid_member"), F("euids"), F("Invalid euids list."));
      return;
    }

    JsonDocument response;
    buildStickserverEnvelope(response, cmd, ver, mid, "ok");
    JsonArray devices = response["devices"].to<JsonArray>();
    size_t okCount = 0;
    for (size_t i = 0; i < euidCount; ++i) {
      int idx = findManagedOutputByEuid(euids[i]);
      JsonObject device = devices.add<JsonObject>();
      if (idx < 0) {
        device["euid"] = euids[i];
        device["status"] = "unknown_euid";
      } else {
        populateStickserverDevice(device, static_cast<uint8_t>(idx));
        device["status"] = "ok";
        ++okCount;
      }
    }
    response["status"] = aggregateStickserverStatus(okCount, euidCount);
    publishStickserverResponse(response);
    return;
  }

  if (cmd == F("join") || cmd == F("reboot")) {
    String euid = parseEuidValue(request["euid"]);
    if (cfg.debugSerial) {
      Serial.print(F("[INFO] [MQTT] Parsed euid: "));
      Serial.println(euid.isEmpty() ? String(F("(none)")) : euid);
    }
    if (euid.isEmpty()) {
      publishStickserverFailure(cmd, ver, mid, F("invalid_member"), F("euid"), F("Missing euid."));
      return;
    }
    int idx = findManagedOutputByEuid(euid);
    if (cfg.debugSerial) {
      Serial.print(F("[INFO] [MQTT] Resolved idx from euid: "));
      Serial.println(idx);
    }
    if (idx < 0) {
      publishStickserverFailure(cmd, ver, mid, F("invalid_member"), F("euid"), F("Unknown euid."));
      return;
    }
    String mappedCmd;
    if (cmd == F("join")) {
      mappedCmd = F("power_on");
    } else {
      mappedCmd = F("reboot");
    }
    bool ok = handleLoadAction(static_cast<uint8_t>(idx), mappedCmd);

    JsonDocument response;
    buildStickserverEnvelope(response, cmd, ver, mid, ok ? "ok" : "busy");
    response["action"] = mappedCmd;
    JsonObject device = response["device"].to<JsonObject>();
    populateStickserverDevice(device, static_cast<uint8_t>(idx));
    device["status"] = ok ? "ok" : "busy";
    publishStickserverResponse(response);
    return;
  }

  // Commands routed by euid: power_on, power_off, factory_reset
  if (cmd == F("power_on") || cmd == F("power_off") || cmd == F("factory_reset")) {
    String euid = parseEuidValue(request["euid"]);
    if (cfg.debugSerial) {
      Serial.print(F("[INFO] [MQTT] Parsed euid: "));
      Serial.println(euid.isEmpty() ? String(F("(none)")) : euid);
    }
    if (euid.isEmpty()) {
      publishStickserverFailure(cmd, ver, mid, F("invalid_member"), F("euid"), F("Missing euid."));
      return;
    }
    int idx = findManagedOutputByEuid(euid);
    if (cfg.debugSerial) {
      Serial.print(F("[INFO] [MQTT] Resolved idx from euid: "));
      Serial.println(idx);
    }
    if (idx < 0) {
      publishStickserverFailure(cmd, ver, mid, F("invalid_member"), F("euid"), F("Unknown euid."));
      return;
    }
    bool ok = handleLoadAction(static_cast<uint8_t>(idx), cmd);

    JsonDocument response;
    buildStickserverEnvelope(response, cmd, ver, mid, ok ? "ok" : "busy");
    JsonObject device = response["device"].to<JsonObject>();
    populateStickserverDevice(device, static_cast<uint8_t>(idx));
    device["status"] = ok ? "ok" : "busy";
    publishStickserverResponse(response);
    return;
  }

  if (cmd == F("leave")) {
    // leave supports either:
    // - euid (single; string or int), or
    // - euids (array/string list; existing behavior)
    JsonDocument response;
    JsonArray devices = response["devices"].to<JsonArray>();
    size_t okCount = 0;
    size_t knownCount = 0;
    size_t unknownCount = 0;
    size_t totalRequested = 0;

    if (!request["euid"].isNull()) {
      String euid = parseEuidValue(request["euid"]);
      if (cfg.debugSerial) {
        Serial.print(F("[INFO] [MQTT] Parsed euid: "));
        Serial.println(euid.isEmpty() ? String(F("(none)")) : euid);
      }
      if (euid.isEmpty()) {
        publishStickserverFailure(cmd, ver, mid, F("invalid_member"), F("euid"), F("Missing euid."));
        return;
      }
      totalRequested = 1;
      int idx = findManagedOutputByEuid(euid);
      if (cfg.debugSerial) {
        Serial.print(F("[INFO] [MQTT] Resolved idx from euid: "));
        Serial.println(idx);
      }
      JsonObject device = devices.add<JsonObject>();
      if (idx < 0) {
        device["euid"] = euid;
        device["status"] = "unknown_euid";
        ++unknownCount;
      } else {
        ++knownCount;
        bool ok = handleLoadAction(static_cast<uint8_t>(idx), F("leave_mesh"));
        populateStickserverDevice(device, static_cast<uint8_t>(idx));
        device["action"] = "leave_mesh";
        device["status"] = ok ? "ok" : "busy";
        if (ok) ++okCount;
      }
    } else {
      String euids[MAX_DEVICES];
      size_t euidCount = 0;
      if (!extractStringArray(request["euids"], euids, euidCount)) {
        publishStickserverFailure(cmd, ver, mid, F("invalid_member"), F("euids"), F("Invalid euids list."));
        return;
      }
      totalRequested = euidCount;
      for (size_t i = 0; i < euidCount; ++i) {
        int idx = findManagedOutputByEuid(euids[i]);
        if (cfg.debugSerial) {
          Serial.print(F("[INFO] [MQTT] Resolved idx from euid: "));
          Serial.println(idx);
        }
        JsonObject device = devices.add<JsonObject>();
        if (idx < 0) {
          device["euid"] = euids[i];
          device["status"] = "unknown_euid";
          ++unknownCount;
          continue;
        }
        ++knownCount;
        bool ok = handleLoadAction(static_cast<uint8_t>(idx), F("leave_mesh"));
        populateStickserverDevice(device, static_cast<uint8_t>(idx));
        device["action"] = "leave_mesh";
        device["status"] = ok ? "ok" : "busy";
        if (ok) ++okCount;
      }
    }

    const char* responseStatus = "partial";
    if (okCount == totalRequested) {
      responseStatus = "ok";
    } else if (knownCount == 0) {
      responseStatus = "not_found";
    } else if (unknownCount == 0) {
      responseStatus = "busy";
    }
    buildStickserverEnvelope(response, cmd, ver, mid, responseStatus);
    publishStickserverResponse(response);
    return;
  }

  publishStickserverFailure(cmd,
                            ver,
                            mid,
                            F("invalid_command"),
                            F("cmd"),
                            F("Unsupported stickserver command."));
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (topic == nullptr || payload == nullptr) return;
  if (length == 0) {
    logWarning(F("MQTT: dropped empty payload."));
    return;
  }
  String topicStr(topic);
  if (topicStr.isEmpty()) {
    logWarning(F("MQTT: dropped message with empty topic."));
    return;
  }
  // MQTT payload bytes are not null-terminated; copy to String using explicit length.
  // reserve() pre-allocates so concat does not reallocate; failure means low memory.
  String payloadStr;
  payloadStr.reserve(length);
  if (!payloadStr.concat(reinterpret_cast<const char*>(payload), length)) {
    logWarning(F("MQTT: dropped message -- payload allocation failed."));
    return;
  }

  // Log every incoming message for debugging
  Serial.print(F("[INFO] [MQTT] Received -> topic: "));
  Serial.print(topicStr);
  Serial.print(F(" payload["));
  Serial.print(length);
  Serial.print(F("]: "));
  // Truncate long payloads in the log to avoid flooding serial
  if (payloadStr.length() <= MQTT_PAYLOAD_LOG_MAX_LEN) {
    Serial.println(payloadStr);
  } else {
    Serial.print(payloadStr.substring(0, MQTT_PAYLOAD_LOG_MAX_LEN));
    Serial.println(F("...(truncated)"));
  }

  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    if (topicStr == mqttOutputSetTopic(i)) {
      bool newState = (payloadStr == "ON" || payloadStr == "1" || payloadStr == "true");
      Serial.print(F("[INFO] [MQTT] Set command -> output "));
      Serial.print(i + 1);
      Serial.print(F(" state: "));
      Serial.println(newState ? F("ON") : F("OFF"));
      if (isActionRunning() && bgAction.deviceIdx == i) cancelAction();
      setOutputDirect(i, newState);
      mqttPublishOutputState(i);
      // Runtime state changes are not persisted to flash by design.
      return;
    }
    if (topicStr == mqttOutputActionTopic(i)) {
      Serial.print(F("[INFO] [MQTT] Action command -> output "));
      Serial.print(i + 1);
      Serial.print(F(" action: "));
      Serial.println(payloadStr);
      handleLoadAction(i, payloadStr);

      return;
    }
  }
  if (topicStr == STICKSERVER_ROOT_TOPIC || topicStr == stickserverInstanceTopic()) {
    Serial.print(F("[INFO] [MQTT] Stickserver message -> topic: "));
    Serial.println(topicStr);
    handleStickserverMessage(topicStr, payloadStr);
    return;
  }
  Serial.print(F("[WARN] [MQTT] Unmatched topic (no handler): "));
  Serial.println(topicStr);
}

const char* mqttStateString(int state) {
  switch (state) {
    case -4: return "CONNECTION_TIMEOUT";
    case -3: return "CONNECTION_LOST";
    case -2: return "CONNECT_FAILED";
    case -1: return "DISCONNECTED";
    case  0: return "CONNECTED";
    case  1: return "BAD_PROTOCOL";
    case  2: return "BAD_CLIENT_ID";
    case  3: return "UNAVAILABLE";
    case  4: return "BAD_CREDENTIALS";
    case  5: return "UNAUTHORIZED";
    default: return "UNKNOWN";
  }
}

bool mqttDoConnect() {
  if (!cfg.mqttEnabled || cfg.mqttHost.isEmpty()) return false;
  mqttClient.setBufferSize(MQTT_PACKET_BUFFER_SIZE);
  mqttClient.setServer(cfg.mqttHost.c_str(), cfg.mqttPort);
  mqttClient.setCallback(mqttCallback);
  String clientId = cfg.hostname;
  Serial.print(F("[INFO] [MQTT] Connecting -> host: "));
  Serial.print(cfg.mqttHost);
  Serial.print(F(" port: "));
  Serial.print(cfg.mqttPort);
  Serial.print(F(" clientId: "));
  Serial.println(clientId);
  bool connected;
  if (cfg.mqttUser.isEmpty()) {
    // No credentials: connect anonymously
    Serial.println(F("[INFO] [MQTT] Auth mode: anonymous"));
    connected = mqttClient.connect(clientId.c_str());
  } else {
    // User is set; password may be empty (broker may allow empty password for a named user)
    Serial.print(F("[INFO] [MQTT] Auth mode: credentials (user: "));
    Serial.print(cfg.mqttUser);
    Serial.println(F(")"));
    connected = mqttClient.connect(clientId.c_str(),
                                   cfg.mqttUser.c_str(),
                                   cfg.mqttPassword.c_str());
  }
  if (!connected) {
    Serial.print(F("[WARN] [MQTT] Connection refused -> state: "));
    Serial.println(mqttStateString(mqttClient.state()));
    return false;
  }
  // Subscribe to stickserver root and instance topics
  Serial.print(F("[INFO] [MQTT] Subscribing -> "));
  Serial.println(STICKSERVER_ROOT_TOPIC);
  mqttClient.subscribe(STICKSERVER_ROOT_TOPIC);
  String instanceTopic = stickserverInstanceTopic();
  Serial.print(F("[INFO] [MQTT] Subscribing -> "));
  Serial.println(instanceTopic);
  mqttClient.subscribe(instanceTopic.c_str());
  // Subscribe to per-output set and action topics
  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    String setTopic = mqttOutputSetTopic(i);
    String actionTopic = mqttOutputActionTopic(i);
    Serial.print(F("[INFO] [MQTT] Subscribing -> "));
    Serial.println(setTopic);
    mqttClient.subscribe(setTopic.c_str());
    Serial.print(F("[INFO] [MQTT] Subscribing -> "));
    Serial.println(actionTopic);
    mqttClient.subscribe(actionTopic.c_str());
  }
  mqttPublishAllOutputStates();
  logStatus(String(F("MQTT connected. Host: ")) + cfg.mqttHost + F(" port: ") + String(cfg.mqttPort));
  return true;
}

void applyMqttSettings() {
  if (!cfg.mqttEnabled || cfg.mqttHost.isEmpty()) {
    if (mqttClient.connected()) {
      logStatus(F("MQTT disabled or host cleared; disconnecting."));
      mqttClient.disconnect();
    } else {
      logStatus(F("MQTT disabled or host not configured; skipping."));
    }
    return;
  }
  Serial.print(F("[INFO] [MQTT] Applying settings -> host: "));
  Serial.print(cfg.mqttHost);
  Serial.print(F(" port: "));
  Serial.println(cfg.mqttPort);
  mqttClient.setBufferSize(MQTT_PACKET_BUFFER_SIZE);
  mqttClient.setServer(cfg.mqttHost.c_str(), cfg.mqttPort);
  mqttClient.setCallback(mqttCallback);
  if (mqttClient.connected()) {
    logStatus(F("MQTT settings updated; disconnecting to force reconnect."));
    mqttClient.disconnect();
  }
  lastMqttConnectAttemptMs = 0;
}

void maintainMqtt() {
  if (!cfg.mqttEnabled || cfg.mqttHost.isEmpty()) return;
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqttClient.connected()) {
    mqttClient.loop();
    return;
  }
  unsigned long now = millis();
  if (now - lastMqttConnectAttemptMs < MQTT_RECONNECT_INTERVAL_MS) return;
  lastMqttConnectAttemptMs = now;
  logStatus(F("Attempting MQTT connection..."));
  if (!mqttDoConnect()) {
    Serial.print(F("[WARN] [MQTT] Connection failed. State: "));
    Serial.println(mqttStateString(mqttClient.state()));
  }
}

void mqttEnsureConnected() {
  if (!cfg.mqttEnabled || cfg.mqttHost.isEmpty()) return;
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqttClient.connected()) return;
  lastMqttConnectAttemptMs = 0;
  maintainMqtt();
}

// ---------------------------------------------------------------------------
// Background load-action state machine
// ---------------------------------------------------------------------------

void startSequenceAction(uint8_t deviceIdx, uint8_t totalCycles) {
  bgAction.deviceIdx = deviceIdx;
  bgAction.allOutputs = false;
  bgAction.cyclesRemaining = totalCycles;
  bgAction.phaseStartMs = millis();
  bgAction.phase = APHASE_CYCLE_OFF;
  setOutputDirect(deviceIdx, false);
  mqttPublishOutputState(deviceIdx);
}

void startLeaveMeshAction(uint8_t deviceIdx) {
  beginLeaveMeshForOutput(deviceIdx, false);
}

void startFactoryResetAction(uint8_t deviceIdx) {
  beginFactoryResetForOutput(deviceIdx, false);
}

void startFactoryResetAllAction() {
  bool hasManaged = false;
  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    if (isManagedOutput(i)) { hasManaged = true; break; }
  }
  if (!hasManaged) {
    logStatus(F("factory_reset_all: no managed outputs"));
    return;
  }
  bgAction.allOutputs = true;
  bgAction.deviceIdx = 0;
  bgAction.cyclesRemaining = FACTORY_RESET_CYCLES;
  logStatus(F("factory_reset_all: turning all managed outputs ON for prep"));
  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    if (isManagedOutput(i)) {
      setOutputDirect(i, true);
      mqttPublishOutputState(i);
    }
  }
  bgAction.phaseStartMs = millis();
  bgAction.phase = APHASE_FACTORY_RESET_PREP_ON;
  if (cfg.debugSerial) {
    Serial.println(F("[DEBUG] factory_reset_all: prep ON started for all managed outputs"));
  }
}

void startLeaveMeshAllAction() {
  bool hasManaged = false;
  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    if (isManagedOutput(i)) { hasManaged = true; break; }
  }
  if (!hasManaged) {
    logStatus(F("leave_mesh_all: no managed outputs"));
    return;
  }
  bgAction.allOutputs = true;
  bgAction.deviceIdx = 0;
  bgAction.cyclesRemaining = LEAVE_MESH_CYCLES;
  logStatus(F("leave_mesh_all: turning all managed outputs ON for prep"));
  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    if (isManagedOutput(i)) {
      setOutputDirect(i, true);
      mqttPublishOutputState(i);
    }
  }
  bgAction.phaseStartMs = millis();
  bgAction.phase = APHASE_LEAVE_MESH_PREP_ON;
  if (cfg.debugSerial) {
    Serial.println(F("[DEBUG] leave_mesh_all: prep ON started for all managed outputs"));
  }
}

void finishAction() {
  uint8_t idx = bgAction.deviceIdx;
  bool wasAll = bgAction.allOutputs;
  bgAction.phase = APHASE_NONE;
  bgAction.allOutputs = false;
  if (wasAll) {
    for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
      if (isManagedOutput(i)) {
        setOutputDirect(i, true);
        mqttPublishOutputState(i);
      }
    }
    logStatus(F("Load action complete for all outputs"));
  } else {
    // Leave output ON after completing the sequence
    setOutputDirect(idx, true);
    mqttPublishOutputState(idx);
    logStatus(String(F("Load action complete for output ")) + String(idx + 1));
  }
}

void maintainBackgroundAction() {
  if (bgAction.phase == APHASE_NONE) return;
  unsigned long now = millis();
  uint8_t idx = bgAction.deviceIdx;

  switch (bgAction.phase) {
    case APHASE_CYCLE_OFF:
      if (now - bgAction.phaseStartMs >= ACTION_CYCLE_OFF_MS) {
        setOutputDirect(idx, true);
        mqttPublishOutputState(idx);
        bgAction.phase = APHASE_CYCLE_ON;
        bgAction.phaseStartMs = now;
      }
      break;
    case APHASE_CYCLE_ON:
      if (now - bgAction.phaseStartMs >= ACTION_CYCLE_ON_MS) {
        --bgAction.cyclesRemaining;
        if (bgAction.cyclesRemaining > 0) {
          setOutputDirect(idx, false);
          mqttPublishOutputState(idx);
          bgAction.phase = APHASE_CYCLE_OFF;
          bgAction.phaseStartMs = now;
        } else {
          // All cycles done; output is ON — enter final wait
          bgAction.phase = APHASE_FINAL_WAIT;
          bgAction.phaseStartMs = now;
        }
      }
      break;
    case APHASE_FINAL_WAIT:
      if (now - bgAction.phaseStartMs >= ACTION_FINAL_WAIT_MS) {
        setOutputDirect(idx, false);
        mqttPublishOutputState(idx);
        bgAction.phase = APHASE_TRIGGER_OFF;
        bgAction.phaseStartMs = now;
      }
      break;
    case APHASE_TRIGGER_OFF:
      if (now - bgAction.phaseStartMs >= ACTION_TRIGGER_OFF_MS) {
        setOutputDirect(idx, true);
        mqttPublishOutputState(idx);
        bgAction.phase = APHASE_TRIGGER_ON;
        bgAction.phaseStartMs = now;
      }
      break;
    case APHASE_TRIGGER_ON:
      if (now - bgAction.phaseStartMs >= ACTION_TRIGGER_ON_MS) {
        finishAction();
      }
      break;
    case APHASE_LEAVE_MESH_PREP_ON:
      if (now - bgAction.phaseStartMs >= LEAVE_MESH_PREP_ON_MS) {
        if (cfg.debugSerial) {
          Serial.println(bgAction.allOutputs
              ? F("[DEBUG] leave_mesh_all: prep done, cycling all outputs OFF (cycle 1)")
              : F("[DEBUG] leave_mesh: prep done, cycling output OFF (cycle 1)"));
        }
        setPhaseOutputState(false);
        bgAction.phase = APHASE_LEAVE_MESH_CYCLE_OFF;
        bgAction.phaseStartMs = millis();
      }
      break;
    case APHASE_LEAVE_MESH_CYCLE_OFF:
      if (now - bgAction.phaseStartMs >= LEAVE_MESH_CYCLE_OFF_MS) {
        if (cfg.debugSerial) {
          Serial.print(bgAction.allOutputs ? F("[DEBUG] leave_mesh_all") : F("[DEBUG] leave_mesh"));
          Serial.print(F(": cycle OFF done, turning ON (cycles remaining="));
          Serial.print(bgAction.cyclesRemaining);
          Serial.println(F(")"));
        }
        setPhaseOutputState(true);
        bgAction.phase = APHASE_LEAVE_MESH_CYCLE_ON;
        bgAction.phaseStartMs = millis();
      }
      break;
    case APHASE_LEAVE_MESH_CYCLE_ON:
      if (now - bgAction.phaseStartMs >= LEAVE_MESH_CYCLE_ON_MS) {
        --bgAction.cyclesRemaining;
        if (bgAction.cyclesRemaining > 0) {
          if (cfg.debugSerial) {
            Serial.print(bgAction.allOutputs ? F("[DEBUG] leave_mesh_all") : F("[DEBUG] leave_mesh"));
            Serial.print(F(": cycle ON done, cycling OFF (cycles remaining="));
            Serial.print(bgAction.cyclesRemaining);
            Serial.println(F(")"));
          }
          setPhaseOutputState(false);
          bgAction.phase = APHASE_LEAVE_MESH_CYCLE_OFF;
          bgAction.phaseStartMs = millis();
        } else {
          // All cycles done; outputs are ON — enter final wait (green window)
          if (cfg.debugSerial) {
            Serial.println(bgAction.allOutputs
                ? F("[DEBUG] leave_mesh_all: all cycles done, entering final wait (green window)")
                : F("[DEBUG] leave_mesh: all cycles done, entering final wait (green window)"));
          }
          bgAction.phase = APHASE_LEAVE_MESH_FINAL_WAIT;
          bgAction.phaseStartMs = now;
        }
      }
      break;
    case APHASE_LEAVE_MESH_FINAL_WAIT:
      if (now - bgAction.phaseStartMs >= LEAVE_MESH_FINAL_WAIT_MS) {
        if (cfg.debugSerial) {
          Serial.println(bgAction.allOutputs
              ? F("[DEBUG] leave_mesh_all: final wait done, trigger OFF (while green)")
              : F("[DEBUG] leave_mesh: final wait done, trigger OFF (while green)"));
        }
        setPhaseOutputState(false);
        bgAction.phase = APHASE_LEAVE_MESH_TRIGGER_OFF;
        bgAction.phaseStartMs = millis();
      }
      break;
    case APHASE_LEAVE_MESH_TRIGGER_OFF:
      if (now - bgAction.phaseStartMs >= LEAVE_MESH_TRIGGER_OFF_MS) {
        if (cfg.debugSerial) {
          Serial.println(bgAction.allOutputs
              ? F("[DEBUG] leave_mesh_all: trigger OFF done, trigger ON")
              : F("[DEBUG] leave_mesh: trigger OFF done, trigger ON"));
        }
        setPhaseOutputState(true);
        bgAction.phase = APHASE_LEAVE_MESH_TRIGGER_ON;
        bgAction.phaseStartMs = millis();
      }
      break;
    case APHASE_LEAVE_MESH_TRIGGER_ON:
      if (now - bgAction.phaseStartMs >= LEAVE_MESH_TRIGGER_ON_MS) {
        if (cfg.debugSerial) {
          Serial.println(bgAction.allOutputs
              ? F("[DEBUG] leave_mesh_all: trigger complete, finishing")
              : F("[DEBUG] leave_mesh: trigger complete, finishing"));
        }
        finishAction();
      }
      break;
    case APHASE_FACTORY_RESET_PREP_ON:
      if (now - bgAction.phaseStartMs >= FACTORY_RESET_PREP_ON_MS) {
        if (cfg.debugSerial) {
          Serial.println(bgAction.allOutputs
              ? F("[DEBUG] factory_reset_all: prep done, cycling all outputs OFF (cycle 1)")
              : F("[DEBUG] factory_reset: prep done, cycling output OFF (cycle 1)"));
        }
        setPhaseOutputState(false);
        bgAction.phase = APHASE_FACTORY_RESET_CYCLE_OFF;
        bgAction.phaseStartMs = millis();
      }
      break;
    case APHASE_FACTORY_RESET_CYCLE_OFF:
      if (now - bgAction.phaseStartMs >= FACTORY_RESET_CYCLE_OFF_MS) {
        if (cfg.debugSerial) {
          Serial.print(bgAction.allOutputs ? F("[DEBUG] factory_reset_all") : F("[DEBUG] factory_reset"));
          Serial.print(F(": cycle OFF done, turning ON (cycles remaining="));
          Serial.print(bgAction.cyclesRemaining);
          Serial.println(F(")"));
        }
        setPhaseOutputState(true);
        bgAction.phase = APHASE_FACTORY_RESET_CYCLE_ON;
        bgAction.phaseStartMs = millis();
      }
      break;
    case APHASE_FACTORY_RESET_CYCLE_ON:
      if (now - bgAction.phaseStartMs >= FACTORY_RESET_CYCLE_ON_MS) {
        --bgAction.cyclesRemaining;
        if (bgAction.cyclesRemaining > 0) {
          if (cfg.debugSerial) {
            Serial.print(bgAction.allOutputs ? F("[DEBUG] factory_reset_all") : F("[DEBUG] factory_reset"));
            Serial.print(F(": cycle ON done, cycling OFF (cycles remaining="));
            Serial.print(bgAction.cyclesRemaining);
            Serial.println(F(")"));
          }
          setPhaseOutputState(false);
          bgAction.phase = APHASE_FACTORY_RESET_CYCLE_OFF;
          bgAction.phaseStartMs = millis();
        } else {
          // All cycles done; outputs are ON — enter final wait (blue transition window)
          if (cfg.debugSerial) {
            Serial.println(bgAction.allOutputs
                ? F("[DEBUG] factory_reset_all: all cycles done, entering final wait (blue window)")
                : F("[DEBUG] factory_reset: all cycles done, entering final wait (blue window)"));
          }
          bgAction.phase = APHASE_FACTORY_RESET_FINAL_WAIT;
          bgAction.phaseStartMs = now;
        }
      }
      break;
    case APHASE_FACTORY_RESET_FINAL_WAIT:
      if (now - bgAction.phaseStartMs >= FACTORY_RESET_FINAL_WAIT_MS) {
        // Trigger reset while blue: power cycle (OFF then ON)
        if (cfg.debugSerial) {
          Serial.println(bgAction.allOutputs
              ? F("[DEBUG] factory_reset_all: final wait done, trigger OFF (while blue)")
              : F("[DEBUG] factory_reset: final wait done, trigger OFF (while blue)"));
        }
        setPhaseOutputState(false);
        bgAction.phase = APHASE_FACTORY_RESET_TRIGGER_OFF;
        bgAction.phaseStartMs = millis();
      }
      break;
    case APHASE_FACTORY_RESET_TRIGGER_OFF:
      if (now - bgAction.phaseStartMs >= FACTORY_RESET_TRIGGER_OFF_MS) {
        if (cfg.debugSerial) {
          Serial.println(bgAction.allOutputs
              ? F("[DEBUG] factory_reset_all: trigger OFF done, trigger ON")
              : F("[DEBUG] factory_reset: trigger OFF done, trigger ON"));
        }
        setPhaseOutputState(true);
        bgAction.phase = APHASE_FACTORY_RESET_TRIGGER_ON;
        bgAction.phaseStartMs = millis();
      }
      break;
    case APHASE_FACTORY_RESET_TRIGGER_ON:
      if (now - bgAction.phaseStartMs >= FACTORY_RESET_TRIGGER_ON_MS) {
        if (cfg.debugSerial) {
          Serial.println(bgAction.allOutputs
              ? F("[DEBUG] factory_reset_all: trigger complete, finishing")
              : F("[DEBUG] factory_reset: trigger complete, finishing"));
        }
        finishAction();
      }
      break;
    default:
      bgAction.phase = APHASE_NONE;
      break;
  }
}

// ---------------------------------------------------------------------------
// Load-action command dispatcher (shared by HTTP and MQTT)
// ---------------------------------------------------------------------------

// Cancel any running background action, publish current output MQTT state.
// Callers are responsible for saving config if needed.
void cancelAction() {
  if (!isActionRunning()) return;
  uint8_t idx = bgAction.deviceIdx;
  bool wasAll = bgAction.allOutputs;
  bgAction.phase = APHASE_NONE;
  bgAction.allOutputs = false;
  if (wasAll) {
    for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
      if (isManagedOutput(i)) mqttPublishOutputState(i);
    }
    logStatus(F("Action cancelled for all outputs"));
  } else {
    logStatus(String(F("Action cancelled for output ")) + String(idx + 1));
    mqttPublishOutputState(idx);
  }
}

bool handleLoadAction(uint8_t idx, const String& cmd) {
  if (idx >= cfg.numOutputs) return false;
  if (cmd == F("power_on")) {
    if (isActionRunning() && bgAction.deviceIdx == idx) cancelAction();
    setOutputDirect(idx, true);
    mqttPublishOutputState(idx);
    // Runtime state changes are not persisted to flash by design.
    return true;
  }
  if (cmd == F("power_off")) {
    if (isActionRunning() && bgAction.deviceIdx == idx) cancelAction();
    setOutputDirect(idx, false);
    mqttPublishOutputState(idx);
    // Runtime state changes are not persisted to flash by design.
    return true;
  }
  if (!isValidOutputPin(cfg.devices[idx].pin)) return false;
  if (isActionRunning()) return false;
  if (cmd == F("leave_mesh")) {
    logStatus(String(F("Starting leave_mesh action for output ")) + String(idx + 1));
    startLeaveMeshAction(idx);
    return true;
  }
  if (cmd == F("factory_reset")) {
    logStatus(String(F("Starting factory_reset action for output ")) + String(idx + 1));
    startFactoryResetAction(idx);
    return true;
  }
  if (cmd == F("reboot")) {
    logStatus(String(F("Starting reboot action for output ")) + String(idx + 1));
    startSequenceAction(idx, REBOOT_SEQUENCE_CYCLES);
    return true;
  }
  return false;

}

void handleHome() {
  bool actionRunning = isActionRunning();
  String html = F(
      "<!doctype html><html><head><meta charset='utf-8'><title>VIBRANT</title>"
      "<style>body{font-family:Arial,sans-serif;margin:20px;}table{border-collapse:collapse;width:100%;}"
      "th,td{border:1px solid #ddd;padding:8px;}th{background:#f5f5f5;}a,button{padding:4px 8px;margin:2px;}"
      "input[type='checkbox']{width:18px;height:18px;}"
      ".action-banner{background:#fff3cd;border:1px solid #ffc107;padding:10px;margin:10px 0;border-radius:4px;}</style>"
      "<script>\n"
      "var _wasRunning=false;\n"
      "function pollStatus(){\n"
      "  fetch('/action/status').then(function(r){return r.json();})\n"
      "  .then(function(d){\n"
      "    var div=document.getElementById('action-status');\n"
      "    if(d.running){\n"
      "      var detail=d.cyclesRemaining>0?' (cycles remaining: '+d.cyclesRemaining+')':'';\n"
      "      div.innerHTML='<div class=\"action-banner\"><strong>Action running on output '+(d.idx+1)+': '+d.phase+detail+'</strong>'"
      "      +' &nbsp; <form method=\"post\" action=\"/action/cancel\" style=\"display:inline;\"><button type=\"submit\">Cancel</button></form></div>';\n"
      "    } else {\n"
      "      div.innerHTML='';\n"
      "      if(_wasRunning){window.location.reload();}\n"
      "    }\n"
      "    _wasRunning=d.running;\n"
      "  }).catch(function(){});\n"
      "}\n"
      "setInterval(pollStatus,3000);\n"
      "</script>"
      "</head><body><h1>VIBRANT Output Control</h1><p>Version: ");
  html += SOFTWARE_VERSION;
  html += F("</p><p><a href='/settings'>Settings</a></p>");
  if (usingFactoryPassword()) {
    html += passwordWarningHtml();
  }
  // Initial action-status banner rendered server-side; JS polling keeps it updated.
  // The phase-detail string is also formatted by the JS updater; they share the same
  // visual format but run in different contexts (C++/server vs JS/browser).
  html += "<div id='action-status'>";
  if (actionRunning) {
    bool inCyclePhase = (bgAction.phase == APHASE_CYCLE_OFF || bgAction.phase == APHASE_CYCLE_ON ||
                         bgAction.phase == APHASE_LEAVE_MESH_CYCLE_OFF || bgAction.phase == APHASE_LEAVE_MESH_CYCLE_ON ||
                         bgAction.phase == APHASE_FACTORY_RESET_CYCLE_OFF || bgAction.phase == APHASE_FACTORY_RESET_CYCLE_ON);
    String phaseDetail = inCyclePhase
        ? String(F(" (cycles remaining: ")) + String(bgAction.cyclesRemaining) + ")"
        : "";
    String target = bgAction.allOutputs
        ? String(F("all outputs"))
        : String(F("output ")) + String(bgAction.deviceIdx + 1);
    html += "<div class='action-banner'><strong>Action running on " +
            target + ": " + actionPhaseName() + phaseDetail + "</strong>"
            " &nbsp; <form method='post' action='/action/cancel' style='display:inline;'>"
            "<button type='submit'>Cancel</button></form></div>";
  }
  html += "</div>";

  // Global bulk-action buttons
  const char* bulkDisabled = actionRunning ? " disabled" : "";
  html += String(F("<div style='margin:10px 0;'>"))
          + "<form method='post' action='/action/all-on' style='display:inline;margin:0;'>"
            "<button type='submit'" + bulkDisabled + ">Turn ON all</button></form>"
          + "<form method='post' action='/action/all-off' style='display:inline;margin:0;'>"
            "<button type='submit'" + bulkDisabled + ">Turn OFF all</button></form>"
          + "<form method='post' action='/action/leave-mesh-all' style='display:inline;margin:0;'"
            " onsubmit=\"return confirm('Run leave mesh signal on ALL outputs?');\">"
            "<button type='submit'" + bulkDisabled + ">Leave Mesh All</button></form>"
          + "<form method='post' action='/action/factory-reset-all' style='display:inline;margin:0;'"
            " onsubmit=\"return confirm('Run factory reset signal on ALL outputs?');\">"
            "<button type='submit'" + bulkDisabled + ">Factory Reset All</button></form>"
          + "</div>";

  html += F("<table><tr><th>#</th><th>Model</th><th>Name</th><th>Status</th><th>Output</th><th>Actions</th></tr>");

  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    const DeviceEntry& d = cfg.devices[i];
    bool mapped = isValidOutputPin(d.pin);
    String status = mapped ? (d.state ? String(F("ON")) : String(F("OFF"))) : String(F("Unassigned"));
    bool thisActionRunning = actionRunning && bgAction.deviceIdx == i;
    bool otherActionRunning = actionRunning && bgAction.deviceIdx != i;

    html += "<tr><td>" + String(i + 1) + "</td><td>" + htmlEscape(d.model) + "</td><td>" +
            htmlEscape(d.name) + "</td><td>" + status + "</td><td>";

    if (mapped) {
      html += "<form method='post' action='/toggle' style='margin:0;'>"
              "<input type='hidden' name='idx' value='" + String(i) + "'>"
              "<input type='hidden' name='state' id='state_" + String(i) + "' value='" + String(d.state ? 1 : 0) + "'>"
              "<input type='checkbox'";
      if (d.state) html += " checked";
      html += " onchange=\"document.getElementById('state_" + String(i) + "').value=this.checked?1:0;this.form.submit();\">"
              "</form>";
    } else {
      html += F("(none)");
    }

    html += "</td><td>";
    if (mapped) {
      if (thisActionRunning) {
        html += F("<em>Running...</em>");
      } else {
        const char* disabledAttr = otherActionRunning ? " disabled" : "";
        html += "<form method='post' action='/action' style='display:inline;margin:0;'>"
                "<input type='hidden' name='idx' value='" + String(i) + "'>"
                "<input type='hidden' name='cmd' value='power_on'>"
                "<button type='submit'" + disabledAttr + ">On</button></form>"
                "<form method='post' action='/action' style='display:inline;margin:0;'>"
                "<input type='hidden' name='idx' value='" + String(i) + "'>"
                "<input type='hidden' name='cmd' value='power_off'>"
                "<button type='submit'" + disabledAttr + ">Off</button></form>"
                "<form method='post' action='/action' style='display:inline;margin:0;'>"
                "<input type='hidden' name='idx' value='" + String(i) + "'>"
                "<input type='hidden' name='cmd' value='leave_mesh'>"
                "<button type='submit'" + disabledAttr + ">Leave Mesh</button></form>"
                "<form method='post' action='/action' style='display:inline;margin:0;'>"
                "<input type='hidden' name='idx' value='" + String(i) + "'>"
                "<input type='hidden' name='cmd' value='factory_reset'>"
                "<button type='submit'" + disabledAttr + ">Factory Reset</button></form>";
      }
    } else {
      html += F("(none)");
    }
    html += "</td></tr>";
  }

  html += F("</table></body></html>");
  server.send(200, "text/html", html);
}

void handleToggle() {
  if (!ensureAuthorized()) return;

  if (!server.hasArg("idx") || !server.hasArg("state")) {
    logError(F("Toggle request missing idx or state."));
    server.send(400, "text/plain", "Missing idx or state");
    return;
  }

  int idx = -1;
  if (!parseIndexValue(server.arg("idx"), idx) || idx < 0 || idx >= MAX_DEVICES) {
    logError(F("Toggle request contained invalid device index."));
    server.send(400, "text/plain", "Invalid device index");
    return;
  }
  if (server.arg("state") != "0" && server.arg("state") != "1") {
    logError(F("Toggle request contained invalid state value."));
    server.send(400, "text/plain", "Invalid state value");
    return;
  }

  DeviceEntry& d = cfg.devices[idx];
  if (!isValidOutputPin(d.pin)) {
    logError(String(F("Toggle request for unmapped output: ")) + d.name);
    server.sendHeader("Location", "/");
    server.send(303);
    return;
  }

  // Abort any running sequence action on this output
  if (isActionRunning() && bgAction.deviceIdx == static_cast<uint8_t>(idx)) {
    cancelAction();
  }

  d.state = server.arg("state") == "1";
  if (outputsActivated) {
    pinMode(d.pin, OUTPUT);
    // Inverted output logic: logical ON -> LOW, logical OFF -> HIGH (active-low).
    digitalWrite(d.pin, d.state ? LOW : HIGH);
  } else {
    applyOutputsWhenSafe();
  }
  mqttPublishOutputState(static_cast<uint8_t>(idx));
  Serial.print(F("[INFO] Output toggled: "));
  Serial.print(d.name);
  Serial.print(F(" -> "));
  Serial.println(d.state ? F("ON") : F("OFF"));
  // Runtime state changes are not persisted to flash by design.
  mqttPublishOutputState(static_cast<uint8_t>(idx));

  server.sendHeader("Location", "/");
  server.send(303);
}

void handleAction() {
  if (!ensureAuthorized()) return;
  if (!server.hasArg("idx") || !server.hasArg("cmd")) {
    server.send(400, "text/plain", "Missing idx or cmd");
    return;
  }
  int idx = -1;
  if (!parseIndexValue(server.arg("idx"), idx) || idx < 0 || idx >= MAX_DEVICES) {
    server.send(400, "text/plain", "Invalid device index");
    return;
  }
  String cmd = server.arg("cmd");
  if (!handleLoadAction(static_cast<uint8_t>(idx), cmd)) {
    if (isActionRunning()) {
      server.send(409, "text/plain", "Another action is already running");
    } else {
      server.send(400, "text/plain", "Unknown command or invalid output");
    }
    return;
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleCancelAction() {
  if (!ensureAuthorized()) return;
  if (isActionRunning()) {
    cancelAction();
    // Cancelling an action does not persist transient output state to flash.
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleActionStatus() {
  bool running = isActionRunning();
  String json = "{\"running\":";
  json += running ? "true" : "false";
  if (running) {
    bool inCyclePhase = (bgAction.phase == APHASE_CYCLE_OFF || bgAction.phase == APHASE_CYCLE_ON ||
                         bgAction.phase == APHASE_LEAVE_MESH_CYCLE_OFF || bgAction.phase == APHASE_LEAVE_MESH_CYCLE_ON ||
                         bgAction.phase == APHASE_FACTORY_RESET_CYCLE_OFF || bgAction.phase == APHASE_FACTORY_RESET_CYCLE_ON);
    json += ",\"idx\":" + String(bgAction.deviceIdx);
    json += ",\"allOutputs\":" + String(bgAction.allOutputs ? "true" : "false");
    json += ",\"phase\":\"" + actionPhaseName() + "\"";
    json += ",\"cyclesRemaining\":" + String(inCyclePhase ? bgAction.cyclesRemaining : 0);
  }
  json += "}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handleAllOn() {
  if (!ensureAuthorized()) return;
  logStatus(F("Turn ON all outputs requested."));
  bool first = true;
  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    if (!isManagedOutput(i)) continue;
    if (!first) delay(50);
    setOutputDirect(i, true);
    mqttPublishOutputState(i);
    first = false;
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleAllOff() {
  if (!ensureAuthorized()) return;
  logStatus(F("Turn OFF all outputs requested."));
  bool first = true;
  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    if (!isManagedOutput(i)) continue;
    if (!first) delay(50);
    setOutputDirect(i, false);
    mqttPublishOutputState(i);
    first = false;
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleFactoryResetAll() {
  if (!ensureAuthorized()) return;
  if (isActionRunning()) {
    server.send(409, "text/plain", "Another action is already running");
    return;
  }
  logStatus(F("Factory reset ALL outputs requested."));
  startFactoryResetAllAction();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleLeaveMeshAll() {
  if (!ensureAuthorized()) return;
  if (isActionRunning()) {
    server.send(409, "text/plain", "Another action is already running");
    return;
  }
  logStatus(F("Leave mesh ALL outputs requested."));
  startLeaveMeshAllAction();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleSettingsGet() {
  if (!ensureAuthorized()) return;

  String html = F(
      "<!doctype html><html><head><meta charset='utf-8'><title>VIBRANT Settings</title>"
      "<style>body{font-family:Arial,sans-serif;margin:20px;}fieldset{margin-bottom:16px;}"
      "label{display:block;margin:6px 0;}table{border-collapse:collapse;width:100%;}"
      "th,td{border:1px solid #ddd;padding:8px;}th{background:#f5f5f5;}input,select{width:100%;padding:6px;box-sizing:border-box;}"
      "button{padding:8px 10px;margin-right:8px;}.bulk-actions{margin:10px 0;}</style>");
  html += "<script>\n"
          "function parseNameSequence(template){\n"
          "  const match=template.match(/#(\\d+)/);\n"
          "  if(!match)return null;\n"
          "  return {token:match[0],start:parseInt(match[1],10)};\n"
          "}\n"
          "function sequentialNameValue(template,sequence,index){\n"
          "  if(!sequence)return template;\n"
          "  return template.replace(sequence.token,'#'+(sequence.start+index));\n"
          "}\n"
          "function copyFirstModelToAll(){\n"
          "  const form=document.forms[0];\n"
          "  if(!form)return;\n"
          "  const first=form.elements['model_0'];\n"
          "  if(!first)return;\n"
          "  const count=form.querySelectorAll(\"input[name^='model_']\").length;\n"
          "  for(let i=1;i<count;i++){\n"
          "    const field=form.elements['model_'+i];\n"
          "    if(field)field.value=first.value;\n"
          "  }\n"
          "}\n"
          "function copyFirstNameToAll(){\n"
          "  const form=document.forms[0];\n"
          "  if(!form)return;\n"
          "  const first=form.elements['name_0'];\n"
          "  if(!first)return;\n"
          "  const template=first.value;\n"
          "  const sequence=parseNameSequence(template);\n"
          "  const count=form.querySelectorAll(\"input[name^='name_']\").length;\n"
          "  for(let i=0;i<count;i++){\n"
          "    const field=form.elements['name_'+i];\n"
          "    if(field)field.value=sequentialNameValue(template,sequence,i);\n"
          "  }\n"
          "}\n"
          "function reverseGpioAssignments(){\n"
          "  const form=document.forms[0];\n"
          "  if(!form)return;\n"
          "  const fields=Array.from(form.querySelectorAll(\"select[name^='pin_']\"));\n"
          "  const values=fields.map((field)=>field.value).reverse();\n"
          "  fields.forEach((field,index)=>{field.value=values[index];});\n"
          "}\n"
          "</script>";
  html += F("</head><body><h1>Settings</h1>"
            "<p><a href='/'>Back to main page</a></p><form method='post' action='/settings'>");
  if (usingFactoryPassword()) {
    html += passwordWarningHtml();
  }

  html += "<fieldset><legend>Station network</legend>"
          "<label>Station SSID <input name='staSsid' value='" + htmlEscape(cfg.staSsid) + "'></label>"
          "<label for='staPassword'>Station password</label><input id='staPassword' name='staPassword' type='password' value='' placeholder='Leave empty to keep current station password'>"
          "</fieldset>";

  html += "<fieldset><legend>Access point</legend>"
          "<label>SoftAP SSID <input value='" + htmlEscape(defaultSoftApSsidFromMac(cfg.mac)) + "' readonly></label>"
          "<label for='apPassword'>SoftAP password</label><input id='apPassword' name='apPassword' type='password' value='' placeholder='Leave empty to keep current AP password'>"
          "</fieldset>";

  html += "<fieldset><legend>Network device settings</legend>"
          "<label>MAC address <input name='mac' value='" + htmlEscape(cfg.mac) + "' maxlength='17'></label>"
          "<label>Hostname for DHCP <input name='hostname' value='" + htmlEscape(cfg.hostname) + "'></label>"

          "<label>Wi-Fi power (5.0 - 20.5 dBm) <input name='wifiPower' type='number' min='5' max='20.5' step='0.1' value='" + String(cfg.wifiPower, 1) + "'></label>"
          "</fieldset>";

  html += "<fieldset><legend>Devices</legend>"
          "<label>Number of outputs (1 - 16) <input name='numOutputs' type='number' min='1' max='16' step='1' value='" + String(cfg.numOutputs) + "'></label>"
          "<div class='bulk-actions'><button type='button' onclick='copyFirstModelToAll()'>Use first Model for all</button>"
          "<button type='button' onclick='copyFirstNameToAll()'>Use first Name for all</button>"
          "<button type='button' onclick='reverseGpioAssignments()'>Reverse GPIO assignments</button>"
          "<span>If the first Name contains #5, copy keeps the first row at #5 and fills later rows as #6, #7, and so on.</span></div>"
          "<table><tr><th>#</th><th>Model</th><th>Name</th><th>Control output</th></tr>";
  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    html += "<tr><td>" + String(i + 1) + "</td>"
            "<td><input name='model_" + String(i) + "' value='" + htmlEscape(cfg.devices[i].model) + "'></td>"
            "<td><input name='name_" + String(i) + "' value='" + htmlEscape(cfg.devices[i].name) + "'></td>"
            "<td><select name='pin_" + String(i) + "'>";

    html += (cfg.devices[i].pin < 0) ? "<option value='-1' selected>none</option>" : "<option value='-1'>none</option>";
    for (size_t pinIndex = 0; pinIndex < OUTPUT_PIN_MAPPING_COUNT; ++pinIndex) {
      html += pinOption(cfg.devices[i].pin, OUTPUT_PIN_MAPPINGS[pinIndex]);
    }
    html += "</select></td></tr>";
  }
  html += "</table></fieldset>";

  html += "<fieldset><legend>MQTT</legend>"
          "<label><input type='checkbox' name='mqttEnabled' value='1'" +
          String(cfg.mqttEnabled ? " checked" : "") + "> Enable MQTT</label>"
          "<label for='mqttHost'>MQTT server host</label>"
          "<input id='mqttHost' name='mqttHost' value='" + htmlEscape(cfg.mqttHost) + "' placeholder='e.g. 192.168.1.6'>"
          "<label for='mqttPort'>MQTT port</label>"
          "<input id='mqttPort' name='mqttPort' type='number' min='1' max='65535' value='" + String(cfg.mqttPort) + "'>"
          "<label for='mqttUser'>MQTT user (optional)</label>"
          "<input id='mqttUser' name='mqttUser' value='" + htmlEscape(cfg.mqttUser) + "'>"
          "<label for='mqttPassword'>MQTT password (optional)</label>"
          "<input id='mqttPassword' name='mqttPassword' type='password' value='' placeholder='Leave empty to keep current'"
          " oninput=\"document.getElementById('mqttPasswordClear').checked = false;\">"
          "<label><input type='checkbox' id='mqttPasswordClear' name='mqttPasswordClear' value='1'"
          " onchange=\"if(this.checked)document.getElementById('mqttPassword').value='';\"> Clear MQTT password (remove broker authentication)</label>"
          "<p style='font-size:0.9em;color:#555;'>Topics (N = zero-based output index, e.g. 0 = Output 1): "
          "<code>vibrant/" + htmlEscape(cfg.hostname) + "/out/&lt;N&gt;/set</code> (ON/OFF) &amp; "
          "<code>vibrant/" + htmlEscape(cfg.hostname) + "/out/&lt;N&gt;/action</code> (power_on / power_off / leave_mesh / factory_reset). "
          "Stickserver compatibility subscribes to <code>" + htmlEscape(String(STICKSERVER_ROOT_TOPIC)) + "</code> and replies on the device topic.</p>"
          "</fieldset>";

  html += "<fieldset><legend>Diagnostics</legend>"
          "<label><input type='checkbox' name='arduinoOtaEnabled' value='1'" +
          String(cfg.arduinoOtaEnabled ? " checked" : "") + "> Enable ArduinoOTA service (developer OTA via IDE/tools; uses admin password for auth)</label>"
          "<p style='font-size:0.9em;color:#555;'>When enabled, ArduinoOTA uses hostname <code>" + htmlEscape(cfg.hostname) +
          "</code> and requires the current admin password.</p>"
          "<label><input type='checkbox' name='debugSerial' value='1'" +
          String(cfg.debugSerial ? " checked" : "") + "> Enable verbose serial debug logging (euid parsing, idx resolution, etc.)</label>"
          "</fieldset>";

  html += F("<button type='submit'>Save settings</button></form>");

  html += F(
      "<h2>Configuration maintenance</h2>"
      "<p><a href='/config/export'>Download configuration backup</a></p>");
  html += "<p>Hold the FLASH button during power-on (during the first " + String(FLASH_BOOT_DETECTION_WINDOW_MS) +
          " milliseconds of boot) to trigger factory reset and restart.</p>"
          "<form method='post' action='/config/factory-reset' onsubmit=\"return confirm('Factory reset?');\">"
          "<button type='submit'>Factory reset</button></form>"
          "<form method='post' action='/config/import' enctype='multipart/form-data'>"
          "<label>Import backup JSON <input type='file' name='config' accept='application/json' required></label>"
          "<button type='submit'>Upload and restore</button></form>"
          "<h2>Firmware update</h2>"
          "<p>Upload a compiled <code>.bin</code> to update firmware over the network. "
          "The device reboots automatically after a successful flash.</p>"
          "<p><a href='/firmware/update'>Open firmware update page</a></p>";

  html += F("</body></html>");
  server.send(200, "text/html", html);
}

void handleSettingsPost() {
  if (!ensureAuthorized()) return;

  String macValue = server.arg("mac");
  macValue.toUpperCase();
  uint8_t macBytes[6] = {0};
  if (!parseMac(macValue, macBytes)) {
    logError(F("Settings save rejected due to invalid MAC address."));
    server.send(400, "text/plain", "Invalid MAC address format. Use AA:BB:CC:DD:EE:FF");
    return;
  }

  float parsedPower = 0.0f;
  if (!parseFloatValue(server.arg("wifiPower"), parsedPower)) {
    logError(F("Settings save rejected due to invalid Wi-Fi power value."));
    server.send(400, "text/plain", "Invalid Wi-Fi power value");
    return;
  }

  cfg.mac = macValue;
  cfg.hostname = server.arg("hostname");
  cfg.staSsid = server.arg("staSsid");

  String newStaPassword = server.arg("staPassword");
  if (!newStaPassword.isEmpty()) {
    cfg.staPassword = newStaPassword;
  }

  String newApPassword = server.arg("apPassword");
  if (!newApPassword.isEmpty()) {
    cfg.apPassword = newApPassword;
  }

  cfg.wifiPower = constrain(parsedPower, MIN_WIFI_POWER, MAX_WIFI_POWER);

  int parsedOutputs = -1;
  if (!parseIndexValue(server.arg("numOutputs"), parsedOutputs) ||
      parsedOutputs < 1 || parsedOutputs > MAX_DEVICES) {
    logError(F("Settings save rejected: number of outputs must be between 1 and 16."));
    server.send(400, "text/plain", "Number of outputs must be 1-16");
    return;
  }

  if (cfg.staSsid.isEmpty() || cfg.staPassword.isEmpty() || cfg.apPassword.isEmpty()) {
    logError(F("Settings save rejected because station SSID or passwords were empty."));
    server.send(400, "text/plain", "Station SSID, station password, and AP password must not be empty");
    return;
  }
  if (cfg.hostname.isEmpty()) cfg.hostname = defaultHostnameFromMac(cfg.mac);

  cfg.numOutputs = static_cast<uint8_t>(parsedOutputs);

  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    cfg.devices[i].model = server.arg("model_" + String(i));
    cfg.devices[i].name = server.arg("name_" + String(i));
    int pin = -1;
    String pinArgName = "pin_" + String(i);
    if (!server.hasArg(pinArgName) || !parsePinValue(server.arg(pinArgName), pin)) {
      pin = -1;
    }
    cfg.devices[i].pin = pin;
    if (!isValidOutputPin(cfg.devices[i].pin)) {
      cfg.devices[i].state = false;
    }
  }

  cfg.mqttEnabled = server.hasArg("mqttEnabled") && server.arg("mqttEnabled") == "1";
  cfg.mqttHost = server.arg("mqttHost");  {
    int parsedPort = -1;
    if (parseIndexValue(server.arg("mqttPort"), parsedPort) && parsedPort >= 1 && parsedPort <= 65535) {
      cfg.mqttPort = static_cast<uint16_t>(parsedPort);
    } else {
      cfg.mqttPort = DEFAULT_MQTT_PORT;
    }
  }
  cfg.mqttUser = server.arg("mqttUser");
  bool clearMqttPassword = server.hasArg("mqttPasswordClear") && server.arg("mqttPasswordClear") == "1";
  if (clearMqttPassword) {
    cfg.mqttPassword = "";
  } else {
    String newMqttPassword = server.arg("mqttPassword");
    String legacyMqttPassword = server.arg("mqttPass");
    if (newMqttPassword.isEmpty()) {
      newMqttPassword = legacyMqttPassword;
    } else if (!legacyMqttPassword.isEmpty() && legacyMqttPassword != newMqttPassword) {
      logWarning(F("Settings POST contained both mqttPassword and legacy mqttPass; applying mqttPassword."));
    }
    if (!newMqttPassword.isEmpty()) {
      cfg.mqttPassword = newMqttPassword;
    }
  }

  cfg.arduinoOtaEnabled = server.hasArg("arduinoOtaEnabled") && server.arg("arduinoOtaEnabled") == "1";
  cfg.debugSerial = server.hasArg("debugSerial") && server.arg("debugSerial") == "1";

  logStatus(F("Settings updated from web UI."));
  if (!saveConfig()) {
    restartDevice(F("Failed to persist updated settings."));
  }
  applyWifiSettings();
  refreshOutputsForCurrentBootPhase();
  applyMqttSettings();
  mqttEnsureConnected();
  applyArduinoOtaSettings();

  server.sendHeader("Location", "/settings");
  server.send(303);
}

void handleConfigExport() {
  if (!ensureAuthorized()) return;

  if (!LittleFS.exists(CONFIG_PATH)) {
    logError(F("Config export requested but configuration file was not found."));
    server.send(404, "text/plain", "Configuration file not found");
    return;
  }
  File file = LittleFS.open(CONFIG_PATH, "r");
  if (!file) {
    logError(F("Config export failed because the file could not be opened."));
    server.send(500, "text/plain", "Unable to open configuration file");
    return;
  }
  logStatus(F("Configuration export started."));
  server.streamFile(file, "application/json");
  file.close();
}

void handleConfigImportUpload() {
  if (!server.authenticate("admin", cfg.apPassword.c_str())) {
    importFailed = true;
    server.requestAuthentication();
    return;
  }

  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    importFailed = false;
    logStatus(F("Configuration import upload started."));
    if (LittleFS.exists(IMPORT_CONFIG_PATH)) {
      LittleFS.remove(IMPORT_CONFIG_PATH);
    }
    importFile = LittleFS.open(IMPORT_CONFIG_PATH, "w");
    if (!importFile) {
      importFailed = true;
      logError(F("Failed to open temporary import file for writing."));
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (importFile) {
      if (importFile.write(upload.buf, upload.currentSize) != upload.currentSize) {
        importFailed = true;
        logError(F("Failed while writing uploaded config chunk."));
      }
    } else {
      importFailed = true;
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (importFile) {
      importFile.close();
    }
    logStatus(F("Configuration import upload finished."));
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    importFailed = true;
    if (importFile) {
      importFile.close();
    }
    logError(F("Configuration import upload was aborted."));
  }
}

void handleConfigImportDone() {
  if (!ensureAuthorized()) return;
  if (importFailed) {
    logError(F("Configuration upload failed before validation."));
    server.send(500, "text/plain", "Configuration upload failed");
    return;
  }
  File uploaded = LittleFS.open(IMPORT_CONFIG_PATH, "r");
  if (!uploaded) {
    logError(F("Uploaded configuration file was not found after upload."));
    server.send(400, "text/plain", "Uploaded configuration file not found");
    return;
  }
  JsonDocument verifyDoc;
  DeserializationError verifyError = deserializeJson(verifyDoc, uploaded);
  uploaded.close();
  if (verifyError) {
    LittleFS.remove(IMPORT_CONFIG_PATH);
    logError(String(F("Uploaded configuration JSON is invalid: ")) + verifyError.c_str());
    server.send(400, "text/plain", "Uploaded configuration JSON is invalid");
    return;
  }
  if (LittleFS.exists(CONFIG_PATH)) {
    LittleFS.remove(CONFIG_PATH);
  }
  if (!LittleFS.rename(IMPORT_CONFIG_PATH, CONFIG_PATH)) {
    LittleFS.remove(IMPORT_CONFIG_PATH);
    logError(F("Failed to replace active configuration with imported file."));
    server.send(500, "text/plain", "Failed to replace configuration file");
    return;
  }
  if (!loadConfig()) {
    restartDevice(F("Imported configuration could not be loaded after replace."));
  }
  if (!saveConfig()) {
    restartDevice(F("Failed to normalize and save imported configuration."));
  }
  logStatus(F("Configuration import applied successfully."));
  applyWifiSettings();
  refreshOutputsForCurrentBootPhase();
  server.sendHeader("Location", "/settings");
  server.send(303);
}

void handleFactoryReset() {
  if (!ensureAuthorized()) return;

  logStatus(F("Factory reset requested."));
  setFactoryDefaults();
  if (!saveConfig()) {
    restartDevice(F("Failed to persist factory reset configuration."));
  }
  applyWifiSettings();
  refreshOutputsForCurrentBootPhase();
  server.sendHeader("Location", "/settings");
  server.send(303);
}

void handleFirmwareUpdatePage() {
  if (!ensureAuthorized()) return;

  String html = F(
      "<!doctype html><html><head><meta charset='utf-8'><title>Firmware Update</title>"
      "<style>body{font-family:Arial,sans-serif;margin:20px;}fieldset{margin-bottom:16px;}"
      "label{display:block;margin:6px 0;}button{padding:8px 10px;margin-right:8px;}"
      ".warn{color:#b00020;font-weight:bold;}</style></head><body>"
      "<h1>Firmware Update</h1>"
      "<p><a href='/settings'>Back to Settings</a></p>"
      "<p>Upload a compiled <code>.bin</code> firmware file to update the device. "
      "The device will reboot automatically after a successful update.</p>"
      "<p class='warn'>Warning: Do not power off the device during an update. "
      "Interrupted updates may require USB reflashing to recover.</p>"
      "<form method='post' action='/firmware/update' enctype='multipart/form-data'>"
      "<label>Firmware file (.bin) <input type='file' name='firmware' accept='.bin' required></label>"
      "<button type='submit'>Upload and flash</button></form>"
      "</body></html>");
  server.send(200, "text/html", html);
}

void handleFirmwareUpdateUpload() {
  if (!server.authenticate("admin", cfg.apPassword.c_str())) {
    server.requestAuthentication();
    return;
  }

  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    otaUpdateFailed = false;
    otaUpdateError = "";
    logStatus(String(F("OTA firmware update upload started: ")) + upload.filename);
    // Reserve flash space for the new sketch; subtract 4 KB (0x1000) as safety margin
    // and align down to a 4 KB flash sector boundary (0xFFFFF000 mask).
    uint32_t maxSketchSize = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
    if (cfg.debugSerial) {
      Serial.print(F("[DEBUG] OTA max sketch size: "));
      Serial.println(maxSketchSize);
    }
    if (!Update.begin(maxSketchSize)) {
      otaUpdateFailed = true;
      otaUpdateError = Update.getErrorString();
      logError(String(F("OTA Update.begin failed: ")) + otaUpdateError);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (!otaUpdateFailed) {
      if (cfg.debugSerial) {
        Serial.print(F("[DEBUG] OTA write chunk: "));
        Serial.print(upload.currentSize);
        Serial.print(F(" bytes, total so far: "));
        Serial.println(upload.totalSize);
      }
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        otaUpdateFailed = true;
        otaUpdateError = Update.getErrorString();
        logError(String(F("OTA Update.write failed: ")) + otaUpdateError);
      }
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (!otaUpdateFailed) {
      if (!Update.end(true)) {
        otaUpdateFailed = true;
        otaUpdateError = Update.getErrorString();
        logError(String(F("OTA Update.end failed: ")) + otaUpdateError);
      } else {
        logStatus(String(F("OTA firmware update upload complete: ")) + String(upload.totalSize) + F(" bytes written."));
      }
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    otaUpdateFailed = true;
    otaUpdateError = F("Upload aborted by client.");
    Update.end(false);
    logError(F("OTA firmware update upload was aborted."));
  }
}

void handleFirmwareUpdateDone() {
  if (!ensureAuthorized()) return;
  if (otaUpdateFailed) {
    logError(String(F("OTA firmware update failed: ")) + otaUpdateError);
    String html = F(
        "<!doctype html><html><head><meta charset='utf-8'><title>Firmware Update Failed</title>"
        "<style>body{font-family:Arial,sans-serif;margin:20px;}.err{color:#b00020;}</style></head><body>"
        "<h1 class='err'>Firmware Update Failed</h1><p>");
    html += htmlEscape(otaUpdateError);
    html += F("</p><p><a href='/firmware/update'>Try again</a> | <a href='/settings'>Settings</a></p>"
              "</body></html>");
    server.send(500, "text/html", html);
    return;
  }
  logStatus(F("OTA firmware update succeeded. Rebooting..."));
  server.send(200, "text/html",
              F("<!doctype html><html><head><meta charset='utf-8'>"
                "<meta http-equiv='refresh' content='15;url=/'>"
                "<title>Update OK</title>"
                "<style>body{font-family:Arial,sans-serif;margin:20px;}</style></head><body>"
                "<h1>Firmware update successful</h1>"
                "<p>The device is rebooting. This page will reload in 15 seconds.</p>"
                "</body></html>"));
  // Flush the TCP send buffer and allow time for the HTTP response to reach the client.
  server.client().flush();
  delay(200);
  ESP.restart();
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

void maintainWifiConnection() {
  wl_status_t status = WiFi.status();
  if (status != lastWifiStatus) {
    Serial.print(F("[INFO] Wi-Fi status changed: "));
    Serial.print(wifiStatusToString(lastWifiStatus));
    Serial.print(F(" -> "));
    Serial.println(wifiStatusToString(status));
    lastWifiStatus = status;
  }

  if (status == WL_CONNECTED) {
    if (wifiDisconnectSinceMs != 0) {
      logStatus(String(F("Wi-Fi reconnected. Station IP: ")) + WiFi.localIP().toString());
    }
    resetWifiRecoveryState();
    return;
  }

  unsigned long now = millis();
  if (wifiDisconnectSinceMs == 0) {
    wifiDisconnectSinceMs = now;
    lastWifiConnectLogMs = 0;
    logWarning(String(F("Wi-Fi disconnected. Status: ")) + wifiStatusToString(status));
  }

  if (now - wifiDisconnectSinceMs >= WIFI_RECOVERY_WINDOW_MS && wifiRecoveryAttempts >= MAX_WIFI_RECOVERY_ATTEMPTS) {
    restartDevice(F("Wi-Fi could not be recovered within the configured window."));
  }

  if (lastWifiConnectLogMs == 0 || now - lastWifiConnectLogMs >= WIFI_CONNECT_LOG_INTERVAL_MS) {
    Serial.print(F("[INFO] Waiting for Wi-Fi recovery. Status="));
    Serial.print(wifiStatusToString(status));
    Serial.print(F(" Attempts="));
    Serial.println(wifiRecoveryAttempts);
    lastWifiConnectLogMs = now;
  }

  if (now - lastWifiReconnectAttemptMs >= WIFI_RECONNECT_INTERVAL_MS) {
    lastWifiReconnectAttemptMs = now;
    ++wifiRecoveryAttempts;
    Serial.print(F("[INFO] Attempting Wi-Fi reconnect #"));
    Serial.println(wifiRecoveryAttempts);
    WiFi.disconnect(false);
    logStatus(String(F("Reconnecting to SSID: ")) + cfg.staSsid);
    WiFi.begin(cfg.staSsid.c_str(), cfg.staPassword.c_str());
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  bootStartMillis = millis();
  delay(100);
  Serial.println();
  Serial.println(F("[INFO] VIBRANT boot starting..."));
  Serial.print(F("[INFO] Software version: "));
  Serial.println(SOFTWARE_VERSION);

  pinMode(FLASH_BUTTON_PIN, INPUT_PULLUP);

  logStatus(F("Mounting LittleFS..."));
  if (!LittleFS.begin()) {
    logError(F("LittleFS mount failed. Formatting filesystem; saved configuration will be erased."));
    if (!LittleFS.format()) {
      restartDevice(F("LittleFS format failed after mount failure."));
    }
    if (!LittleFS.begin()) {
      restartDevice(F("LittleFS mount failed after format."));
    }
    logStatus(F("LittleFS mount succeeded after format."));
  } else {
    logStatus(F("LittleFS mounted successfully."));
  }

  WiFi.mode(WIFI_AP_STA);
  logStatus(F("Loading runtime configuration..."));
  if (!loadConfig()) {
    logError(F("Configuration load path failed; restoring factory defaults."));
    setFactoryDefaults();
    if (!saveConfig()) {
      restartDevice(F("Failed to save factory defaults during boot recovery."));
    }
  }

  checkFlashFactoryResetOnBoot();
  logLoadedWifiConfig();   // [DIAG] log cfg.staSsid and cfg.wifiPower
  applyWifiSettings();
  logWifiScan();           // [DIAG] log visible SSIDs with RSSI
  prepareOutputsForBootPhase();
  mqttEnsureConnected();

  logStatus(F("Registering web routes..."));
  server.on("/", HTTP_GET, handleHome);
  server.on("/toggle", HTTP_POST, handleToggle);
  server.on("/action", HTTP_POST, handleAction);
  server.on("/action/cancel", HTTP_POST, handleCancelAction);
  server.on("/action/status", HTTP_GET, handleActionStatus);
  server.on("/action/all-on", HTTP_POST, handleAllOn);
  server.on("/action/all-off", HTTP_POST, handleAllOff);
  server.on("/action/leave-mesh-all", HTTP_POST, handleLeaveMeshAll);
  server.on("/action/factory-reset-all", HTTP_POST, handleFactoryResetAll);
  server.on("/settings", HTTP_GET, handleSettingsGet);
  server.on("/settings", HTTP_POST, handleSettingsPost);
  server.on("/config/export", HTTP_GET, handleConfigExport);
  server.on("/config/import", HTTP_POST, handleConfigImportDone, handleConfigImportUpload);
  server.on("/config/factory-reset", HTTP_POST, handleFactoryReset);
  server.on("/firmware/update", HTTP_GET, handleFirmwareUpdatePage);
  server.on("/firmware/update", HTTP_POST, handleFirmwareUpdateDone, handleFirmwareUpdateUpload);
  server.onNotFound(handleNotFound);

  server.begin();
  logStatus(F("HTTP server started on port 80."));
  applyMqttSettings();
  applyArduinoOtaSettings();
  logWifiSummary(defaultSoftApSsidFromMac(cfg.mac));
  logStatus(F("Boot sequence complete."));
}

void loop() {
  server.handleClient();
  if (arduinoOtaActive) {
    ArduinoOTA.handle();
  }
  if (!outputsActivated && outputActivationDelayElapsed()) {
    applyOutputsWhenSafe();
  }
  maintainWifiConnection();
  maintainMqtt();
  maintainBackgroundAction();
}
