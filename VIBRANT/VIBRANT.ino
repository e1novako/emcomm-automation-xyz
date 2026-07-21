#include <Arduino.h>
#include <LittleFS.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
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
constexpr int8_t MAX_GPIO_PIN = 15;
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
};

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
    cfg.devices[i].pin = -1;
    cfg.devices[i].state = false;
  }
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


  logStatus(F("Configuration loaded successfully."));
  return true;
}

String pinLabel(int8_t pin) {
  if (pin < 0 || pin > MAX_GPIO_PIN) {
    return F("none");
  }
  return String(F("D")) + String(pin);
}

bool isValidOutputPin(int8_t pin) {
  return pin >= 0 && pin <= MAX_GPIO_PIN;
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
  if (outputsActivated) {
    applyOutputsNow();
    return;
  }
  if (outputActivationDelayElapsed()) {
    applyOutputsNow();
    return;
  }
  logStatus(F("Output configuration updated during boot delay; hardware activation remains deferred."));
  outputActivationDeferredLogged = false;
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

String pinOption(int selectedPin, int pin) {
  String selected = (selectedPin == pin) ? " selected" : "";
  return "<option value=\"" + String(pin) + "\"" + selected + ">D" + String(pin) + "</option>";
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
  return pin >= 0 && pin <= MAX_GPIO_PIN;
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

void handleHome() {
  String html = F(
      "<!doctype html><html><head><meta charset='utf-8'><title>VIBRANT</title>"
      "<style>body{font-family:Arial,sans-serif;margin:20px;}table{border-collapse:collapse;width:100%;}"
      "th,td{border:1px solid #ddd;padding:8px;}th{background:#f5f5f5;}a,button{padding:8px 10px;}"
      "input[type='checkbox']{width:18px;height:18px;}</style>"
      "</head><body><h1>VIBRANT Output Control</h1><p>Version: ");
  html += SOFTWARE_VERSION;
  html += F("</p><p><a href='/settings'>Settings</a></p>"
            "<table><tr><th>#</th><th>Model</th><th>Name</th><th>Status</th><th>Output</th></tr>");
  if (usingFactoryPassword()) {
    html += passwordWarningHtml();
  }

  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    const DeviceEntry& d = cfg.devices[i];
    String status = F("Unassigned");
    bool mapped = isValidOutputPin(d.pin);
    if (mapped) status = d.state ? F("ON") : F("OFF");

    html += "<tr><td>" + String(i + 1) + "</td><td>" + htmlEscape(d.model) + "</td><td>" + htmlEscape(d.name) +
            "</td><td>" + status + "</td><td>";

    if (mapped) {
      html += "<form method='post' action='/toggle' style='margin:0;'>"
              "<input type='hidden' name='idx' value='" + String(i) + "'>"
              "<input type='hidden' name='state' id='state_" + String(i) + "' value='" + String(d.state ? 1 : 0) + "'>"
              "<input type='checkbox'";
      if (d.state) html += " checked";
      html += " onchange=\"document.getElementById('state_" + String(i) + "').value=this.checked?1:0;this.form.submit();\">"
              "</form>";
    } else {
      html += "(none)";
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

  d.state = server.arg("state") == "1";
  if (outputsActivated) {
    pinMode(d.pin, OUTPUT);
    digitalWrite(d.pin, d.state ? HIGH : LOW);
  } else {
    applyOutputsWhenSafe();
    if (!outputsActivated) {
      logStatus(F("Stored output toggle during boot delay; hardware update remains deferred."));
    }
  }
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

void handleSettingsGet() {
  if (!ensureAuthorized()) return;

  String html = F(
      "<!doctype html><html><head><meta charset='utf-8'><title>VIBRANT Settings</title>"
      "<style>body{font-family:Arial,sans-serif;margin:20px;}fieldset{margin-bottom:16px;}"
      "label{display:block;margin:6px 0;}table{border-collapse:collapse;width:100%;}"
      "th,td{border:1px solid #ddd;padding:8px;}th{background:#f5f5f5;}input,select{width:100%;padding:6px;box-sizing:border-box;}"
      "button{padding:8px 10px;margin-right:8px;}.bulk-actions{margin:10px 0;}</style>");
  html += "<script>\n"
          "function sequentialNameValue(template,index){\n"
          "  return template.replace(/#[0-9]+/g,'#'+(index+1));\n"
          "}\n"
          "function copyFirstModelToAll(){\n"
          "  const form=document.forms[0];\n"
          "  if(!form)return;\n"
          "  const first=form.elements['model_0'];\n"
          "  if(!first)return;\n"
          "  for(let i=1;i<" + String(cfg.numOutputs) + ";i++){\n"
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
          "  first.value=sequentialNameValue(template,0);\n"
          "  for(let i=1;i<" + String(cfg.numOutputs) + ";i++){\n"
          "    const field=form.elements['name_'+i];\n"
          "    if(field)field.value=sequentialNameValue(template,i);\n"
          "  }\n"
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
          "<span>If the first Name contains #&lt;number&gt;, copy starts by normalizing the first row to #1, then fills later rows as #2, #3, and so on.</span></div>"
          "<table><tr><th>#</th><th>Model</th><th>Name</th><th>Control output</th></tr>";
  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    html += "<tr><td>" + String(i + 1) + "</td>"
            "<td><input name='model_" + String(i) + "' value='" + htmlEscape(cfg.devices[i].model) + "'></td>"
            "<td><input name='name_" + String(i) + "' value='" + htmlEscape(cfg.devices[i].name) + "'></td>"
            "<td><select name='pin_" + String(i) + "'>";

    html += (cfg.devices[i].pin < 0) ? "<option value='-1' selected>none</option>" : "<option value='-1'>none</option>";
    for (int pin = 0; pin <= MAX_GPIO_PIN; ++pin) {
      html += pinOption(cfg.devices[i].pin, pin);
    }
    html += "</select></td></tr>";
  }
  html += "</table></fieldset><button type='submit'>Save settings</button></form>";

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
  delay(100);
  bootStartMillis = millis();
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
  applyOutputsWhenSafe();

  logStatus(F("Registering web routes..."));
  server.on("/", HTTP_GET, handleHome);
  server.on("/toggle", HTTP_POST, handleToggle);
  server.on("/settings", HTTP_GET, handleSettingsGet);
  server.on("/settings", HTTP_POST, handleSettingsPost);
  server.on("/config/export", HTTP_GET, handleConfigExport);
  server.on("/config/import", HTTP_POST, handleConfigImportDone, handleConfigImportUpload);
  server.on("/config/factory-reset", HTTP_POST, handleFactoryReset);
  server.onNotFound(handleNotFound);

  server.begin();
  logStatus(F("HTTP server started on port 80."));
  logWifiSummary(defaultSoftApSsidFromMac(cfg.mac));
  logStatus(F("Boot sequence complete."));
}

void loop() {
  server.handleClient();
  applyOutputsWhenSafe();
  maintainWifiConnection();
}
