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
constexpr uint8_t MAX_DEVICES = 16;
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
  cfg.ssid = F("Z-Wave Automation");
  cfg.password = F("KoToTamoPeva2016");
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
  cfg.ssid = doc["ssid"] | "Z-Wave Automation";
  cfg.password = doc["password"] | "KoToTamoPeva2016";
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
    if (pin < 0 || pin > 15) continue;
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

void handleHome() {
  String html = F(
      "<!doctype html><html><head><meta charset='utf-8'><title>VIBRANT</title>"
      "<style>body{font-family:Arial,sans-serif;margin:20px;}table{border-collapse:collapse;width:100%;}"
      "th,td{border:1px solid #ddd;padding:8px;}th{background:#f5f5f5;}a,button{padding:8px 10px;}</style>"
      "</head><body><h1>VIBRANT Output Control</h1><p><a href='/settings'>Settings</a></p>"
      "<table><tr><th>#</th><th>Model</th><th>Name</th><th>Status</th><th>Toggle</th></tr>");

  for (uint8_t i = 0; i < MAX_DEVICES; ++i) {
    const DeviceEntry& d = cfg.devices[i];
    String status = F("Unassigned");
    bool mapped = d.pin >= 0 && d.pin <= 15;
    if (mapped) {
      status = digitalRead(d.pin) == HIGH ? F("ON") : F("OFF");
    }

    html += "<tr><td>" + String(i + 1) + "</td><td>" + htmlEscape(d.model) + "</td><td>" + htmlEscape(d.name) +
            "</td><td>" + status + "</td><td>";

    if (mapped) {
      html += "<input type='checkbox'";
      if (d.state) html += " checked";
      html += " onchange=\"location.href='/toggle?idx=" + String(i) + "&state='+(this.checked?1:0);\">";
    } else {
      html += "(none)";
    }

    html += "</td></tr>";
  }

  html += F("</table></body></html>");
  server.send(200, "text/html", html);
}

void handleToggle() {
  if (!server.hasArg("idx") || !server.hasArg("state")) {
    server.send(400, "text/plain", "Missing idx or state");
    return;
  }

  int idx = server.arg("idx").toInt();
  if (idx < 0 || idx >= MAX_DEVICES) {
    server.send(400, "text/plain", "Invalid device index");
    return;
  }

  DeviceEntry& d = cfg.devices[idx];
  if (d.pin < 0 || d.pin > 15) {
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
  String html = F(
      "<!doctype html><html><head><meta charset='utf-8'><title>VIBRANT Settings</title>"
      "<style>body{font-family:Arial,sans-serif;margin:20px;}fieldset{margin-bottom:16px;}"
      "label{display:block;margin:6px 0;}table{border-collapse:collapse;width:100%;}"
      "th,td{border:1px solid #ddd;padding:8px;}th{background:#f5f5f5;}input,select{width:100%;padding:6px;box-sizing:border-box;}"
      "button{padding:8px 10px;margin-right:8px;}</style></head><body><h1>Settings</h1>"
      "<p><a href='/'>Back to main page</a></p><form method='post' action='/settings'>");

  html += "<fieldset><legend>Network</legend>"
          "<label>MAC address <input name='mac' value='" + htmlEscape(cfg.mac) + "' maxlength='17'></label>"
          "<label>Hostname for DHCP <input name='hostname' value='" + htmlEscape(cfg.hostname) + "'></label>"
          "<label>SSID <input name='ssid' value='" + htmlEscape(cfg.ssid) + "'></label>"
          "<label>Password <input name='password' value='" + htmlEscape(cfg.password) + "'></label>"
          "<label>Wi-Fi power (0.0 - 20.5 dBm) <input name='wifiPower' type='number' min='0' max='20.5' step='0.1' value='" + String(cfg.wifiPower, 1) + "'></label>"
          "</fieldset>";

  html += "<fieldset><legend>Devices (up to 16)</legend><table><tr><th>#</th><th>Model</th><th>Name</th><th>Control output</th></tr>";
  for (uint8_t i = 0; i < MAX_DEVICES; ++i) {
    html += "<tr><td>" + String(i + 1) + "</td>"
            "<td><input name='model_" + String(i) + "' value='" + htmlEscape(cfg.devices[i].model) + "'></td>"
            "<td><input name='name_" + String(i) + "' value='" + htmlEscape(cfg.devices[i].name) + "'></td>"
            "<td><select name='pin_" + String(i) + "'>";

    html += (cfg.devices[i].pin < 0) ? "<option value='-1' selected>none</option>" : "<option value='-1'>none</option>";
    for (int pin = 0; pin <= 15; ++pin) {
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
  cfg.mac = server.arg("mac");
  cfg.mac.toUpperCase();
  cfg.hostname = server.arg("hostname");
  cfg.ssid = server.arg("ssid");
  cfg.password = server.arg("password");
  cfg.wifiPower = server.arg("wifiPower").toFloat();

  if (cfg.ssid.isEmpty()) cfg.ssid = F("Z-Wave Automation");
  if (cfg.password.isEmpty()) cfg.password = F("KoToTamoPeva2016");
  if (cfg.hostname.isEmpty()) cfg.hostname = defaultHostnameFromMac(cfg.mac);

  for (uint8_t i = 0; i < MAX_DEVICES; ++i) {
    cfg.devices[i].model = server.arg("model_" + String(i));
    cfg.devices[i].name = server.arg("name_" + String(i));
    int pin = server.arg("pin_" + String(i)).toInt();
    cfg.devices[i].pin = (pin >= 0 && pin <= 15) ? pin : -1;
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
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    if (LittleFS.exists(CONFIG_PATH)) {
      LittleFS.remove(CONFIG_PATH);
    }
    importFile = LittleFS.open(CONFIG_PATH, "w");
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (importFile) {
      importFile.write(upload.buf, upload.currentSize);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (importFile) {
      importFile.close();
    }
  }
}

void handleConfigImportDone() {
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
    LittleFS.format();
    LittleFS.begin();
  }

  WiFi.mode(WIFI_AP_STA);
  if (!loadConfig()) {
    setFactoryDefaults();
    saveConfig();
  }

  applyWifiSettings();
  applyOutputs();

  server.on("/", HTTP_GET, handleHome);
  server.on("/toggle", HTTP_GET, handleToggle);
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
