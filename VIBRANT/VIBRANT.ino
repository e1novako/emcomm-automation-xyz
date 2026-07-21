#include <Arduino.h>
#include <LittleFS.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

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

constexpr const char* CONFIG_PATH = "/vibrant_config.json";
constexpr const char* IMPORT_CONFIG_PATH = "/vibrant_config_upload.json";
constexpr const char* DEFAULT_STA_SSID = "Z-Wave Automation";
constexpr const char* DEFAULT_STA_PASSWORD = "Fiber714Cvet";
constexpr const char* DEFAULT_AP_PASSWORD = "Fiber714Cvet";
constexpr const char* SOFTWARE_VERSION = "1.2.1";
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
// Background load-action timing
constexpr uint8_t LEAVE_MESH_CYCLES = 5;
constexpr uint8_t FACTORY_RESET_LOAD_CYCLES = 13;
// Number of D0-D7 entries at the start of OUTPUT_PIN_MAPPINGS used for default assignment
constexpr uint8_t DEFAULT_D0_D7_COUNT = 8;
constexpr unsigned long ACTION_CYCLE_OFF_MS = 5000UL;
constexpr unsigned long ACTION_CYCLE_ON_MS = 1000UL;
constexpr unsigned long ACTION_FINAL_WAIT_MS = 5000UL;
constexpr unsigned long ACTION_TRIGGER_OFF_MS = 2000UL;
constexpr unsigned long ACTION_TRIGGER_ON_MS = 1000UL;

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
static_assert(DEFAULT_D0_D7_COUNT <= OUTPUT_PIN_MAPPING_COUNT,
              "DEFAULT_D0_D7_COUNT exceeds available OUTPUT_PIN_MAPPINGS entries.");

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
};

// Background load-action state machine
enum ActionPhase : uint8_t {
  APHASE_NONE = 0,
  APHASE_CYCLE_OFF,
  APHASE_CYCLE_ON,
  APHASE_FINAL_WAIT,
  APHASE_TRIGGER_OFF,
  APHASE_TRIGGER_ON
};

struct ActiveAction {
  ActionPhase phase;
  uint8_t deviceIdx;
  uint8_t cyclesRemaining;
  unsigned long phaseStartMs;
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

DeviceConfig cfg;
ESP8266WebServer server(80);
File importFile;
bool importFailed = false;
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
ActiveAction bgAction = {APHASE_NONE, 0, 0, 0UL};

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
  for (uint8_t i = 0; i < MAX_DEVICES; ++i) {
    cfg.devices[i].model = String(F("Model ")) + String(i + 1);
    cfg.devices[i].name = String(F("Output ")) + String(i + 1);
    // Map first DEFAULT_D0_D7_COUNT outputs to D0-D7 by default; rest unassigned
    cfg.devices[i].pin = (i < DEFAULT_D0_D7_COUNT) ? OUTPUT_PIN_MAPPINGS[i].gpio : -1;
    cfg.devices[i].state = false;
  }
  cfg.mqttEnabled = false;
  cfg.mqttHost = "";
  cfg.mqttPort = DEFAULT_MQTT_PORT;
  cfg.mqttUser = "";
  cfg.mqttPassword = "";
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
      cfg.devices[i].state = d["state"] | false;
    } else {
      cfg.devices[i].model = String(F("Model ")) + String(i + 1);
      cfg.devices[i].name = String(F("Output ")) + String(i + 1);
      cfg.devices[i].pin = -1;
      cfg.devices[i].state = false;
    }
  }

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
    digitalWrite(pin, cfg.devices[i].state ? HIGH : LOW);

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
    digitalWrite(d.pin, state ? HIGH : LOW);
  }
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
  mqttClient.publish(mqttOutputStateTopic(idx).c_str(),
                     cfg.devices[idx].state ? "ON" : "OFF", true);
}

void mqttPublishAllOutputStates() {
  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    mqttPublishOutputState(i);
  }
}

// Forward declaration (defined below)
bool handleLoadAction(uint8_t idx, const String& cmd);

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String topicStr(topic);
  // MQTT payload is not null-terminated; build String safely via single-allocation loop
  String payloadStr;
  payloadStr.reserve(length);
  for (unsigned int j = 0; j < length; ++j) payloadStr += static_cast<char>(payload[j]);

  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    if (topicStr == mqttOutputSetTopic(i)) {
      bool newState = (payloadStr == "ON" || payloadStr == "1" || payloadStr == "true");
      if (isActionRunning() && bgAction.deviceIdx == i) {
        bgAction.phase = APHASE_NONE;
      }
      setOutputDirect(i, newState);
      mqttPublishOutputState(i);
      saveConfig();
      return;
    }
    if (topicStr == mqttOutputActionTopic(i)) {
      handleLoadAction(i, payloadStr);
      return;
    }
  }
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
  mqttClient.setServer(cfg.mqttHost.c_str(), cfg.mqttPort);
  mqttClient.setCallback(mqttCallback);
  String clientId = cfg.hostname;
  bool connected;
  if (cfg.mqttUser.isEmpty()) {
    connected = mqttClient.connect(clientId.c_str());
  } else {
    connected = mqttClient.connect(clientId.c_str(),
                                   cfg.mqttUser.c_str(),
                                   cfg.mqttPassword.c_str());
  }
  if (!connected) return false;
  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    mqttClient.subscribe(mqttOutputSetTopic(i).c_str());
    mqttClient.subscribe(mqttOutputActionTopic(i).c_str());
  }
  mqttPublishAllOutputStates();
  logStatus(String(F("MQTT connected. Host: ")) + cfg.mqttHost);
  return true;
}

void applyMqttSettings() {
  if (!cfg.mqttEnabled || cfg.mqttHost.isEmpty()) {
    if (mqttClient.connected()) mqttClient.disconnect();
    return;
  }
  mqttClient.setServer(cfg.mqttHost.c_str(), cfg.mqttPort);
  mqttClient.setCallback(mqttCallback);
  if (mqttClient.connected()) mqttClient.disconnect();
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
    logWarning(String(F("MQTT connection failed. State: ")) + mqttStateString(mqttClient.state()));
  }
}

// ---------------------------------------------------------------------------
// Background load-action state machine
// ---------------------------------------------------------------------------

void startSequenceAction(uint8_t deviceIdx, uint8_t totalCycles) {
  bgAction.deviceIdx = deviceIdx;
  bgAction.cyclesRemaining = totalCycles;
  bgAction.phaseStartMs = millis();
  bgAction.phase = APHASE_CYCLE_OFF;
  setOutputDirect(deviceIdx, false);
  mqttPublishOutputState(deviceIdx);
}

void finishAction() {
  uint8_t idx = bgAction.deviceIdx;
  bgAction.phase = APHASE_NONE;
  // Leave output ON after completing the sequence
  setOutputDirect(idx, true);
  mqttPublishOutputState(idx);
  saveConfig();
  logStatus(String(F("Load action complete for output ")) + String(idx + 1));
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
    default:
      bgAction.phase = APHASE_NONE;
      break;
  }
}

// ---------------------------------------------------------------------------
// Load-action command dispatcher (shared by HTTP and MQTT)
// ---------------------------------------------------------------------------

bool handleLoadAction(uint8_t idx, const String& cmd) {
  if (idx >= cfg.numOutputs) return false;
  if (cmd == F("power_on")) {
    if (isActionRunning() && bgAction.deviceIdx == idx) bgAction.phase = APHASE_NONE;
    setOutputDirect(idx, true);
    mqttPublishOutputState(idx);
    saveConfig();
    return true;
  }
  if (cmd == F("power_off")) {
    if (isActionRunning() && bgAction.deviceIdx == idx) bgAction.phase = APHASE_NONE;
    setOutputDirect(idx, false);
    mqttPublishOutputState(idx);
    saveConfig();
    return true;
  }
  if (!isValidOutputPin(cfg.devices[idx].pin)) return false;
  if (isActionRunning()) return false;
  if (cmd == F("leave_mesh")) {
    logStatus(String(F("Starting leave_mesh action for output ")) + String(idx + 1));
    startSequenceAction(idx, LEAVE_MESH_CYCLES);
    return true;
  }
  if (cmd == F("factory_reset")) {
    logStatus(String(F("Starting factory_reset action for output ")) + String(idx + 1));
    startSequenceAction(idx, FACTORY_RESET_LOAD_CYCLES);
    return true;
  }
  return false;
}

void handleHome() {
  bool actionRunning = isActionRunning();
  String html = F(
      "<!doctype html><html><head><meta charset='utf-8'><title>VIBRANT</title>");
  if (actionRunning) {
    html += F("<meta http-equiv='refresh' content='3'>");
  }
  html += F("<style>body{font-family:Arial,sans-serif;margin:20px;}table{border-collapse:collapse;width:100%;}"
            "th,td{border:1px solid #ddd;padding:8px;}th{background:#f5f5f5;}a,button{padding:4px 8px;margin:2px;}"
            "input[type='checkbox']{width:18px;height:18px;}"
            ".action-banner{background:#fff3cd;border:1px solid #ffc107;padding:10px;margin:10px 0;border-radius:4px;}</style>"
            "</head><body><h1>VIBRANT Output Control</h1><p>Version: ");
  html += SOFTWARE_VERSION;
  html += F("</p><p><a href='/settings'>Settings</a></p>");
  if (usingFactoryPassword()) {
    html += passwordWarningHtml();
  }
  if (actionRunning) {
    bool inCyclePhase = (bgAction.phase == APHASE_CYCLE_OFF || bgAction.phase == APHASE_CYCLE_ON);
    String phaseDetail = inCyclePhase
        ? String(F(" (cycles remaining: ")) + String(bgAction.cyclesRemaining) + ")"
        : String(F(""));
    html += "<div class='action-banner'><strong>Action running on output " +
            String(bgAction.deviceIdx + 1) + ": " + actionPhaseName() + phaseDetail + "</strong>"
            " &nbsp; <form method='post' action='/action/cancel' style='display:inline;'>"
            "<button type='submit'>Cancel</button></form></div>";
  }
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
        String disabledAttr = otherActionRunning ? String(F(" disabled")) : String(F(""));
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
    logStatus(F("Background action aborted by toggle request."));
    bgAction.phase = APHASE_NONE;
  }

  d.state = server.arg("state") == "1";
  if (outputsActivated) {
    pinMode(d.pin, OUTPUT);
    digitalWrite(d.pin, d.state ? HIGH : LOW);
  } else {
    applyOutputsWhenSafe();
  }
  mqttPublishOutputState(static_cast<uint8_t>(idx));
  Serial.print(F("[INFO] Output toggled: "));
  Serial.print(d.name);
  Serial.print(F(" -> "));
  Serial.println(d.state ? F("ON") : F("OFF"));
  if (!saveConfig()) {
    restartDevice(F("Failed to persist output toggle state."));
  }

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
    logStatus(F("Background action cancelled by user."));
    bgAction.phase = APHASE_NONE;
    saveConfig();
  }
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
          "<input id='mqttPassword' name='mqttPassword' type='password' value='' placeholder='Leave empty to keep current'>"
          "<label><input type='checkbox' name='mqttPasswordClear' value='1'> Clear MQTT password (remove broker authentication)</label>"
          "<p style='font-size:0.9em;color:#555;'>Topics (N = zero-based output index, e.g. 0 = Output 1): "
          "<code>vibrant/" + htmlEscape(cfg.hostname) + "/out/&lt;N&gt;/set</code> (ON/OFF) &amp; "
          "<code>vibrant/" + htmlEscape(cfg.hostname) + "/out/&lt;N&gt;/action</code> (power_on / power_off / leave_mesh / factory_reset)</p>"
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
          "<button type='submit'>Upload and restore</button></form>";

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

  logStatus(F("Settings updated from web UI."));
  if (!saveConfig()) {
    restartDevice(F("Failed to persist updated settings."));
  }
  applyWifiSettings();
  refreshOutputsForCurrentBootPhase();

  // Parse and apply MQTT settings
  cfg.mqttEnabled = server.hasArg("mqttEnabled") && server.arg("mqttEnabled") == "1";
  cfg.mqttHost = server.arg("mqttHost");
  int parsedMqttPort = -1;
  if (parseIndexValue(server.arg("mqttPort"), parsedMqttPort) && parsedMqttPort > 0 && parsedMqttPort <= 65535) {
    cfg.mqttPort = static_cast<uint16_t>(parsedMqttPort);
  } else {
    cfg.mqttPort = DEFAULT_MQTT_PORT;
  }
  cfg.mqttUser = server.arg("mqttUser");
  bool clearMqttPassword = server.hasArg("mqttPasswordClear") && server.arg("mqttPasswordClear") == "1";
  if (clearMqttPassword) {
    cfg.mqttPassword = "";
  } else {
    String newMqttPassword = server.arg("mqttPassword");
    if (!newMqttPassword.isEmpty()) {
      cfg.mqttPassword = newMqttPassword;
    }
  }
  if (!saveConfig()) {
    restartDevice(F("Failed to persist MQTT settings."));
  }
  applyMqttSettings();

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

  logStatus(F("Registering web routes..."));
  server.on("/", HTTP_GET, handleHome);
  server.on("/toggle", HTTP_POST, handleToggle);
  server.on("/action", HTTP_POST, handleAction);
  server.on("/action/cancel", HTTP_POST, handleCancelAction);
  server.on("/settings", HTTP_GET, handleSettingsGet);
  server.on("/settings", HTTP_POST, handleSettingsPost);
  server.on("/config/export", HTTP_GET, handleConfigExport);
  server.on("/config/import", HTTP_POST, handleConfigImportDone, handleConfigImportUpload);
  server.on("/config/factory-reset", HTTP_POST, handleFactoryReset);
  server.onNotFound(handleNotFound);

  server.begin();
  logStatus(F("HTTP server started on port 80."));
  applyMqttSettings();
  logWifiSummary(defaultSoftApSsidFromMac(cfg.mac));
  logStatus(F("Boot sequence complete."));
}

void loop() {
  server.handleClient();
  if (!outputsActivated && outputActivationDelayElapsed()) {
    applyOutputsWhenSafe();
  }
  maintainWifiConnection();
  maintainMqtt();
  maintainBackgroundAction();
}
