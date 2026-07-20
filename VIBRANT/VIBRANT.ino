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
constexpr const char* DEFAULT_AP_PASSWORD = "KoToTamoPeva2016";
constexpr uint8_t MAX_DEVICES = 16;
constexpr int8_t MAX_GPIO_PIN = 15;
constexpr float MIN_WIFI_POWER = 0.0f;
constexpr float MAX_WIFI_POWER = 20.5f;

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

void applyConfiguredMac() {
  uint8_t mac[6] = {0};
  if (!parseMac(cfg.mac, mac)) return;
  wifi_set_macaddr(STATION_IF, mac);
  wifi_set_macaddr(SOFTAP_IF, mac);
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
  if (!file) return false;
  serializeJsonPretty(doc, file);
  file.close();
  return true;
}

bool loadConfig() {
  if (!LittleFS.exists(CONFIG_PATH)) {
    setFactoryDefaults();
    return saveConfig();
  }

  File file = LittleFS.open(CONFIG_PATH, "r");
  if (!file) {
    setFactoryDefaults();
    return saveConfig();
  }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, file);
  file.close();
  if (err) {
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

  return true;
}

void applyOutputs() {
  for (uint8_t i = 0; i < MAX_DEVICES; ++i) {
    int8_t pin = cfg.devices[i].pin;
    if (pin < 0 || pin > MAX_GPIO_PIN) continue;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, cfg.devices[i].state ? HIGH : LOW);
  }
}

void applyWifiSettings() {
  applyConfiguredMac();
  if (cfg.hostname.isEmpty()) {
    cfg.hostname = defaultHostnameFromMac(cfg.mac);
  }
  cfg.wifiPower = constrain(cfg.wifiPower, MIN_WIFI_POWER, MAX_WIFI_POWER);

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP_STA);
  WiFi.hostname(cfg.hostname);
  WiFi.setOutputPower(cfg.wifiPower);
  WiFi.softAP(cfg.ssid.c_str(), cfg.password.c_str());
  WiFi.begin(cfg.ssid.c_str(), cfg.password.c_str());
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
    server.send(400, "text/plain", "Missing idx or state");
    return;
  }

  int idx = -1;
  if (!parseIndexValue(server.arg("idx"), idx) || idx < 0 || idx >= MAX_DEVICES) {
    server.send(400, "text/plain", "Invalid device index");
    return;
  }
  if (server.arg("state") != "0" && server.arg("state") != "1") {
    server.send(400, "text/plain", "Invalid state value");
    return;
  }

  DeviceEntry& d = cfg.devices[idx];
  if (d.pin < 0 || d.pin > MAX_GPIO_PIN) {
    server.sendHeader("Location", "/");
    server.send(303);
    return;
  }

  d.state = server.arg("state") == "1";
  pinMode(d.pin, OUTPUT);
  digitalWrite(d.pin, d.state ? HIGH : LOW);
  saveConfig();

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
          "<label>Wi-Fi power (0.0 - 20.5 dBm) <input name='wifiPower' type='number' min='0' max='20.5' step='0.1' value='" + String(cfg.wifiPower, 1) + "'></label>"
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
    server.send(400, "text/plain", "Invalid MAC address format. Use AA:BB:CC:DD:EE:FF");
    return;
  }

  float parsedPower = 0.0f;
  if (!parseFloatValue(server.arg("wifiPower"), parsedPower)) {
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

  saveConfig();
  applyWifiSettings();
  applyOutputs();

  server.sendHeader("Location", "/settings");
  server.send(303);
}

void handleConfigExport() {
  if (!ensureAuthorized()) return;

  if (!LittleFS.exists(CONFIG_PATH)) {
    server.send(404, "text/plain", "Configuration file not found");
    return;
  }
  File file = LittleFS.open(CONFIG_PATH, "r");
  if (!file) {
    server.send(500, "text/plain", "Unable to open configuration file");
    return;
  }
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
    if (LittleFS.exists(IMPORT_CONFIG_PATH)) {
      LittleFS.remove(IMPORT_CONFIG_PATH);
    }
    importFile = LittleFS.open(IMPORT_CONFIG_PATH, "w");
    if (!importFile) {
      importFailed = true;
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (importFile) {
      importFile.write(upload.buf, upload.currentSize);
    } else {
      importFailed = true;
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (importFile) {
      importFile.close();
    }
  }
}

void handleConfigImportDone() {
  if (!ensureAuthorized()) return;
  if (importFailed) {
    server.send(500, "text/plain", "Configuration upload failed");
    return;
  }
  File uploaded = LittleFS.open(IMPORT_CONFIG_PATH, "r");
  if (!uploaded) {
    server.send(400, "text/plain", "Uploaded configuration file not found");
    return;
  }
  JsonDocument verifyDoc;
  DeserializationError verifyError = deserializeJson(verifyDoc, uploaded);
  uploaded.close();
  if (verifyError) {
    LittleFS.remove(IMPORT_CONFIG_PATH);
    server.send(400, "text/plain", "Uploaded configuration JSON is invalid");
    return;
  }
  if (LittleFS.exists(CONFIG_PATH)) {
    LittleFS.remove(CONFIG_PATH);
  }
  if (!LittleFS.rename(IMPORT_CONFIG_PATH, CONFIG_PATH)) {
    LittleFS.remove(IMPORT_CONFIG_PATH);
    server.send(500, "text/plain", "Failed to replace configuration file");
    return;
  }
  if (!loadConfig()) {
    server.send(400, "text/plain", "Invalid configuration file");
    return;
  }
  saveConfig();
  applyWifiSettings();
  applyOutputs();
  server.sendHeader("Location", "/settings");
  server.send(303);
}

void handleFactoryReset() {
  if (!ensureAuthorized()) return;

  setFactoryDefaults();
  saveConfig();
  applyWifiSettings();
  applyOutputs();
  server.sendHeader("Location", "/settings");
  server.send(303);
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(100);

  if (!LittleFS.begin()) {
    Serial.println(F("LittleFS mount failed. WARNING: formatting will erase all saved configuration."));
    LittleFS.format();
    if (!LittleFS.begin()) {
      Serial.println(F("LittleFS mount failed after format. Running in degraded mode without persistent config."));
    }
  }

  WiFi.mode(WIFI_AP_STA);
  if (!loadConfig()) {
    setFactoryDefaults();
    saveConfig();
  }

  applyWifiSettings();
  applyOutputs();

  server.on("/", HTTP_GET, handleHome);
  server.on("/toggle", HTTP_POST, handleToggle);
  server.on("/settings", HTTP_GET, handleSettingsGet);
  server.on("/settings", HTTP_POST, handleSettingsPost);
  server.on("/config/export", HTTP_GET, handleConfigExport);
  server.on("/config/import", HTTP_POST, handleConfigImportDone, handleConfigImportUpload);
  server.on("/config/factory-reset", HTTP_POST, handleFactoryReset);
  server.onNotFound(handleNotFound);

  server.begin();
}

void loop() {
  server.handleClient();
}
