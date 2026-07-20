#include <Arduino.h>
#include <LittleFS.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>

extern "C" {
#include "user_interface.h"
}

namespace {

constexpr const char* CONFIG_PATH = "/vibrant_config.json";
constexpr const char* IMPORT_CONFIG_PATH = "/vibrant_config_upload.json";
constexpr const char* DEFAULT_AP_SSID = "Z-Wave Automation";
constexpr const char* DEFAULT_AP_PASSWORD = "Fiber714Cvet";
constexpr uint8_t MAX_DEVICES = 16;
constexpr int8_t MAX_GPIO_PIN = 15;
constexpr float MIN_WIFI_POWER = 5.0f;
constexpr float MAX_WIFI_POWER = 20.5f;
constexpr unsigned long WIFI_RECONNECT_INTERVAL_MS = 15000UL;
constexpr unsigned long WIFI_CONNECT_LOG_INTERVAL_MS = 5000UL;
constexpr unsigned long WIFI_RECOVERY_WINDOW_MS = 180000UL;
constexpr uint8_t MAX_WIFI_RECOVERY_ATTEMPTS = 12;

struct DeviceEntry {
  String model;
  String name;
  int8_t pin;
  bool state;
};

struct DeviceConfig {
  String mac;
  String hostname;
  String ssid;
  String password;
  float wifiPower;
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
    logError(F("Configured MAC address is invalid; skipping MAC apply."));
    return false;
  }

  bool stationOk = wifi_set_macaddr(STATION_IF, mac);
  bool apOk = wifi_set_macaddr(SOFTAP_IF, mac);
  if (!stationOk || !apOk) {
    logError(F("Failed to apply configured MAC address to one or more interfaces."));
    return false;
  }

  logStatus(String(F("Applied MAC address: ")) + cfg.mac);
  return true;
}

void setFactoryDefaults() {
  cfg.mac = WiFi.softAPmacAddress();
  cfg.hostname = defaultHostnameFromMac(cfg.mac);
  cfg.ssid = DEFAULT_AP_SSID;
  cfg.password = DEFAULT_AP_PASSWORD;
  cfg.wifiPower = 17.5f;
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
  doc["ssid"] = cfg.ssid;
  doc["password"] = cfg.password;
  doc["wifiPower"] = cfg.wifiPower;

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
  cfg.ssid = doc["ssid"] | DEFAULT_AP_SSID;
  cfg.password = doc["password"] | DEFAULT_AP_PASSWORD;
  cfg.wifiPower = doc["wifiPower"] | 17.5f;

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

  logStatus(F("Configuration loaded successfully."));
  return true;
}

String pinLabel(int8_t pin) {
  if (pin < 0 || pin > MAX_GPIO_PIN) {
    return F("none");
  }
  return String(F("D")) + String(pin);
}

void logDeviceSummary() {
  Serial.println(F("[INFO] Configured outputs:"));
  for (uint8_t i = 0; i < MAX_DEVICES; ++i) {
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

void applyOutputs() {
  logStatus(F("Applying output states..."));
  for (uint8_t i = 0; i < MAX_DEVICES; ++i) {
    int8_t pin = cfg.devices[i].pin;
    if (pin < 0 || pin > MAX_GPIO_PIN) {
      continue;
    }
    pinMode(pin, OUTPUT);
    digitalWrite(pin, cfg.devices[i].state ? HIGH : LOW);
  }
  logDeviceSummary();
}

void logWifiSummary() {
  Serial.print(F("[INFO] SoftAP SSID: "));
  Serial.println(cfg.ssid);
  Serial.print(F("[INFO] Hostname: "));
  Serial.println(cfg.hostname);
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

void applyWifiSettings() {
  logStatus(F("Applying Wi-Fi settings..."));
  applyConfiguredMac();
  if (cfg.hostname.isEmpty()) {
    cfg.hostname = defaultHostnameFromMac(cfg.mac);
    logStatus(String(F("Hostname was empty, defaulted to ")) + cfg.hostname);
  }
  cfg.wifiPower = constrain(cfg.wifiPower, MIN_WIFI_POWER, MAX_WIFI_POWER);

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP_STA);
  WiFi.hostname(cfg.hostname);
  WiFi.setAutoReconnect(true);
  WiFi.setOutputPower(cfg.wifiPower);

  bool apStarted = WiFi.softAP(cfg.ssid.c_str(), cfg.password.c_str());
  if (!apStarted) {
    restartDevice(F("Failed to start SoftAP with configured credentials."));
  }

  WiFi.begin(cfg.ssid.c_str(), cfg.password.c_str());
  logStatus(String(F("Starting station connection to SSID: ")) + cfg.ssid);
  resetWifiRecoveryState();
  lastWifiStatus = WiFi.status();
  logWifiSummary();
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
  return cfg.password == DEFAULT_AP_PASSWORD;
}

String passwordWarningHtml() {
  return F("<p style='color:#b00020;'><strong>Warning:</strong> Factory default Wi-Fi password is active. "
           "Change it now for security.</p>");
}

bool ensureAuthorized() {
  if (server.authenticate("admin", cfg.password.c_str())) return true;
  server.requestAuthentication();
  return false;
}

void handleHome() {
  String html = F(
      "<!doctype html><html><head><meta charset='utf-8'><title>VIBRANT</title>"
      "<style>body{font-family:Arial,sans-serif;margin:20px;}table{border-collapse:collapse;width:100%;}"
      "th,td{border:1px solid #ddd;padding:8px;}th{background:#f5f5f5;}a,button{padding:8px 10px;}</style>"
      "</head><body><h1>VIBRANT Output Control</h1><p><a href='/settings'>Settings</a></p>"
      "<table><tr><th>#</th><th>Model</th><th>Name</th><th>Status</th><th>Toggle</th></tr>");
  if (usingFactoryPassword()) {
    html += passwordWarningHtml();
  }

  for (uint8_t i = 0; i < MAX_DEVICES; ++i) {
    const DeviceEntry& d = cfg.devices[i];
    String status = F("Unassigned");
    bool mapped = d.pin >= 0 && d.pin <= MAX_GPIO_PIN;
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
  if (d.pin < 0 || d.pin > MAX_GPIO_PIN) {
    logError(String(F("Toggle request for unmapped output: ")) + d.name);
    server.sendHeader("Location", "/");
    server.send(303);
    return;
  }

  d.state = server.arg("state") == "1";
  pinMode(d.pin, OUTPUT);
  digitalWrite(d.pin, d.state ? HIGH : LOW);
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
      "button{padding:8px 10px;margin-right:8px;}</style></head><body><h1>Settings</h1>"
      "<p><a href='/'>Back to main page</a></p><form method='post' action='/settings'>");
  if (usingFactoryPassword()) {
    html += passwordWarningHtml();
  }

  html += "<fieldset><legend>Network</legend>"
          "<label>MAC address <input name='mac' value='" + htmlEscape(cfg.mac) + "' maxlength='17'></label>"
          "<label>Hostname for DHCP <input name='hostname' value='" + htmlEscape(cfg.hostname) + "'></label>"
          "<label>SSID <input name='ssid' value='" + htmlEscape(cfg.ssid) + "'></label>"
          "<label for='password'>Password</label><input id='password' name='password' type='password' value='' placeholder='Leave empty to keep current password'>"
          "<label>Wi-Fi power (5.0 - 20.5 dBm) <input name='wifiPower' type='number' min='5' max='20.5' step='0.1' value='" + String(cfg.wifiPower, 1) + "'></label>"
          "</fieldset>";

  html += "<fieldset><legend>Devices (up to 16)</legend><table><tr><th>#</th><th>Model</th><th>Name</th><th>Control output</th></tr>";
  for (uint8_t i = 0; i < MAX_DEVICES; ++i) {
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
      "<p><a href='/config/export'>Download configuration backup</a></p>"
      "<form method='post' action='/config/factory-reset' onsubmit=\"return confirm('Factory reset?');\">"
      "<button type='submit'>Factory reset</button></form>"
      "<form method='post' action='/config/import' enctype='multipart/form-data'>"
      "<label>Import backup JSON <input type='file' name='config' accept='application/json' required></label>"
      "<button type='submit'>Upload and restore</button></form>");

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
  cfg.ssid = server.arg("ssid");
  String newPassword = server.arg("password");
  if (!newPassword.isEmpty()) {
    cfg.password = newPassword;
  }
  cfg.wifiPower = constrain(parsedPower, MIN_WIFI_POWER, MAX_WIFI_POWER);

  if (cfg.ssid.isEmpty() || cfg.password.isEmpty()) {
    logError(F("Settings save rejected because SSID or password was empty."));
    server.send(400, "text/plain", "SSID and password must not be empty");
    return;
  }
  if (cfg.hostname.isEmpty()) cfg.hostname = defaultHostnameFromMac(cfg.mac);

  for (uint8_t i = 0; i < MAX_DEVICES; ++i) {
    cfg.devices[i].model = server.arg("model_" + String(i));
    cfg.devices[i].name = server.arg("name_" + String(i));
    int pin = -1;
    String pinArgName = "pin_" + String(i);
    if (!server.hasArg(pinArgName) || !parsePinValue(server.arg(pinArgName), pin)) {
      pin = -1;
    }
    cfg.devices[i].pin = pin;
    if (cfg.devices[i].pin < 0) {
      cfg.devices[i].state = false;
    }
  }

  logStatus(F("Settings updated from web UI."));
  if (!saveConfig()) {
    restartDevice(F("Failed to persist updated settings."));
  }
  applyWifiSettings();
  applyOutputs();

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
  if (!server.authenticate("admin", cfg.password.c_str())) {
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
  applyOutputs();
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
  applyOutputs();
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
    logError(String(F("Wi-Fi disconnected. Status: ")) + wifiStatusToString(status));
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
    WiFi.begin(cfg.ssid.c_str(), cfg.password.c_str());
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println(F("[INFO] VIBRANT boot starting..."));

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

  applyWifiSettings();
  applyOutputs();

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
  logWifiSummary();
  logStatus(F("Boot sequence complete."));
}

void loop() {
  server.handleClient();
  maintainWifiConnection();
}
