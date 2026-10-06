#include "WebServer.h"
#include "BootDiagnostics.h"
#include "Config.h"
#include "Debug.h"
#include "Display.h"
#include "Version.h"
#include "WebAssets.h"
#include "WebUpdate.h"
#include "WifiManager.h"
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace c4matrix {
ESP8266WebServer server(80);

void registerWebRoutes() {
  server.on("/", HTTP_GET, handleHome);
  server.on("/config", HTTP_GET, handleConfig);
  server.on("/config/save", HTTP_POST, handleConfigSave);
  server.on("/config/factory-reset", HTTP_POST, handleFactoryReset);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/text", HTTP_POST, handleTextPost);
  server.on("/api/scroll", HTTP_POST, handleScrollPost);
  server.on("/update", HTTP_GET, handleFirmwareUpdatePage);
  server.on("/update", HTTP_POST, handleFirmwareUpdateDone,
            handleFirmwareUpdateUpload);
  server.onNotFound(handleNotFound);
  server.begin();
  logStatus(F("HTTP server started on port 80."));
}

String htmlEscape(const String &value) {
  String out;
  out.reserve(value.length() + 12);
  for (size_t i = 0; i < value.length(); ++i) {
    switch (value[i]) {
    case '&':
      out += F("&amp;");
      break;
    case '<':
      out += F("&lt;");
      break;
    case '>':
      out += F("&gt;");
      break;
    case '"':
      out += F("&quot;");
      break;
    case '\'':
      out += F("&#39;");
      break;
    default:
      out += value[i];
    }
  }
  return out;
}

String jsonEscape(const String &value) {
  String out;
  out.reserve(value.length() + 8);
  for (size_t i = 0; i < value.length(); ++i) {
    char c = value[i];
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (static_cast<uint8_t>(c) < 0x20) {
      out += ' ';
    } else {
      out += c;
    }
  }
  return out;
}

bool ensureAuthorized() {
  if (server.authenticate("admin", cfg.apPassword.c_str()))
    return true;
  server.requestAuthentication();
  return false;
}

static String checkbox(const char *name, bool checked) {
  String html = F("<input type='checkbox' name='");
  html += name;
  html += F("' value='1'");
  if (checked)
    html += F(" checked");
  html += F(">");
  return html;
}

static String selectedOption(int pin, int optionPin, const char *label) {
  String html = F("<option value='");
  html += String(optionPin);
  html += '\'';
  if (pin == optionPin)
    html += F(" selected");
  html += '>';
  html += label;
  html += F("</option>");
  return html;
}

static bool parseColor(const String &value, uint32_t &color) {
  if (value.length() != 7 || value[0] != '#')
    return false;
  color = 0;
  for (uint8_t i = 1; i < 7; ++i) {
    char c = value[i];
    uint8_t digit;
    if (c >= '0' && c <= '9')
      digit = c - '0';
    else if (c >= 'a' && c <= 'f')
      digit = c - 'a' + 10;
    else if (c >= 'A' && c <= 'F')
      digit = c - 'A' + 10;
    else
      return false;
    color = (color << 4) | digit;
  }
  return true;
}

static String colorHex(uint32_t color) {
  char value[8];
  snprintf(value, sizeof(value), "#%06lX",
           static_cast<unsigned long>(color & 0xFFFFFFUL));
  return String(value);
}

static bool parseIntegerArg(const String &value, long &number) {
  if (value.isEmpty())
    return false;
  char *end = nullptr;
  number = strtol(value.c_str(), &end, 10);
  return end != value.c_str() && *end == '\0';
}

static bool parseFloatArg(const String &value, float &number) {
  if (value.isEmpty())
    return false;
  char *end = nullptr;
  number = strtof(value.c_str(), &end);
  return end != value.c_str() && *end == '\0' && isfinite(number);
}

static bool validHostname(const String &hostname) {
  if (hostname.isEmpty() || hostname.length() > 32 ||
      hostname[0] == '-' || hostname[hostname.length() - 1] == '-')
    return false;
  for (size_t i = 0; i < hostname.length(); ++i) {
    char c = hostname[i];
    if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
          (c >= '0' && c <= '9') || c == '-'))
      return false;
  }
  return true;
}

void handleHome() {
  String html = F("<!doctype html><html><head><meta charset='utf-8'><meta "
                  "name='viewport' content='width=device-width,initial-scale=1'>"
                  "<title>C4-MATRIX</title>");
  html += FPSTR(MATRIX_PAGE_STYLE);
  html += F("</head><body><h1>C4-MATRIX</h1><p><a href='/config'>"
            "Configuration</a></p><form id='text-form'><label>Text to "
            "display<input id='text' name='text' maxlength='128' "
            "autocomplete='off' required></label><button type='submit'>"
            "Display text</button></form><p id='message' role='status'></p>");
  html += FPSTR(HOME_PAGE_SCRIPT);
  html += F("</body></html>");
  server.send(200, "text/html; charset=utf-8", html);
}

void handleConfig() {
  if (!ensureAuthorized())
    return;
  String html = F("<!doctype html><html><head><meta charset='utf-8'><meta "
                  "name='viewport' content='width=device-width,initial-scale=1'>"
                  "<title>C4-MATRIX Configuration</title>");
  html += FPSTR(MATRIX_PAGE_STYLE);
  html += F("</head><body><h1>C4-MATRIX Configuration</h1><p><a href='/'>"
            "Display</a> | <a href='/update'>Web firmware update</a></p>"
            "<fieldset><legend>Status</legend><div id='status' class='status'>"
            "Loading...</div></fieldset><form method='post' action='/config/"
            "save'><fieldset><legend>Network</legend><label>Hostname<input "
            "name='hostname' maxlength='32' value='");
  html += htmlEscape(cfg.hostname);
  html += F("'></label><label>Wi-Fi power (dBm)<input type='number' "
            "name='wifiPower' min='5' max='20.5' step='0.5' value='");
  html += String(cfg.wifiPower, 1);
  html += F("'></label><label>Station SSID<input name='staSsid' maxlength='32' "
            "value='");
  html += htmlEscape(cfg.staSsid);
  html += F("'></label><label>Station password (leave blank to keep current)"
            "<input type='password' name='staPassword' maxlength='64' "
            "autocomplete='new-password'></label><label>Custom MAC ");
  html += checkbox("useCustomMac", cfg.useCustomMac);
  html += F("</label><label>MAC address<input name='mac' maxlength='17' "
            "value='");
  html += htmlEscape(cfg.mac);
  html += F("'></label></fieldset><fieldset><legend>Display</legend>"
            "<label>Data output pin<select name='displayPin'>");
  html += selectedOption(cfg.displayPin, 16, "D0 (GPIO16)");
  html += selectedOption(cfg.displayPin, 5, "D1 (GPIO5)");
  html += selectedOption(cfg.displayPin, 4, "D2 (GPIO4)");
  html += selectedOption(cfg.displayPin, 0, "D3 (GPIO0)");
  html += selectedOption(cfg.displayPin, 2, "D4 (GPIO2)");
  html += selectedOption(cfg.displayPin, 14, "D5 (GPIO14)");
  html += selectedOption(cfg.displayPin, 12, "D6 (GPIO12)");
  html += selectedOption(cfg.displayPin, 13, "D7 (GPIO13)");
  html += selectedOption(cfg.displayPin, 15, "D8 (GPIO15)");
  html += F("</select></label><label>Brightness (0-255)<input "
            "type='number' name='brightness' min='0' max='255' value='");
  html += String(cfg.brightness);
  html += F("'></label><label>Text color<input type='color' name='textColor' "
            "value='");
  html += colorHex(cfg.textColor);
  html += F("'></label><label>Serpentine wiring ");
  html += checkbox("serpentine", cfg.serpentine);
  html += F("</label><label>Flip horizontal orientation ");
  html += checkbox("flipHorizontal", cfg.flipHorizontal);
  html += F("</label></fieldset><fieldset><legend>Scroll</legend><label>"
            "Enable scrolling ");
  html += checkbox("scrollEnabled", cfg.scrollEnabled);
  html += F("</label><label>Direction<select name='direction'><option "
            "value='left'");
  if (!cfg.scrollRight)
    html += F(" selected");
  html += F(">Left</option><option value='right'");
  if (cfg.scrollRight)
    html += F(" selected");
  html += F(">Right</option></select></label><label>Speed (ms per column, "
            "10-1000)<input type='number' name='scrollSpeed' min='10' "
            "max='1000' value='");
  html += String(cfg.scrollSpeed);
  html += F("'></label></fieldset><fieldset><legend>Services</legend>"
            "<label>ArduinoOTA enabled ");
  html += checkbox("arduinoOtaEnabled", cfg.arduinoOtaEnabled);
  html += F("</label><label>Serial debug logging ");
  html += checkbox("debugSerial", cfg.debugSerial);
  html += F("</label></fieldset><button type='submit'>Save and reboot</button>"
            "</form><form method='post' action='/config/factory-reset' "
            "onsubmit=\"return confirm('Restore factory defaults and reboot?')\">"
            "<button type='submit'>Factory reset</button></form><script>"
            "async function refreshStatus(){try{let r=await fetch('/api/status');"
            "let s=await r.json();document.querySelector('#status').textContent="
            "`Version: ${s.version}\\nIP: ${s.ip}\\nMAC: ${s.mac}\\nHeap: "
            "${s.heap} bytes\\nUptime: ${s.uptime} s\\nText: ${s.text}\\nScroll: "
            "${s.scrollEnabled?'on':'off'} ${s.direction}, ${s.speed} ms/column`}"
            "}catch(e){document.querySelector('#status').textContent='Status "
            "unavailable.'}}refreshStatus();</script></body></html>");
  server.send(200, "text/html; charset=utf-8", html);
}

void handleConfigSave() {
  if (!ensureAuthorized())
    return;
  if (!server.hasArg("hostname") || !server.hasArg("staSsid") ||
      !server.hasArg("displayPin") || !server.hasArg("brightness") ||
      !server.hasArg("textColor") || !server.hasArg("scrollSpeed") ||
      !server.hasArg("wifiPower") || !server.hasArg("direction")) {
    server.send(400, "text/plain", "Missing configuration field");
    return;
  }
  String hostname = server.arg("hostname");
  String ssid = server.arg("staSsid");
  String password = server.arg("staPassword");
  String mac = server.arg("mac");
  long pinValue = -1, brightnessValue = -1, speedValue = -1;
  float power = 0;
  bool parsedNumbers = parseIntegerArg(server.arg("displayPin"), pinValue) &&
                       parseIntegerArg(server.arg("brightness"),
                                       brightnessValue) &&
                       parseIntegerArg(server.arg("scrollSpeed"), speedValue) &&
                       parseFloatArg(server.arg("wifiPower"), power);
  uint32_t color;
  uint8_t parsedMac[6];
  const bool validPin = pinValue == 16 || pinValue == 5 || pinValue == 4 ||
                        pinValue == 0 || pinValue == 2 || pinValue == 14 ||
                        pinValue == 12 || pinValue == 13 || pinValue == 15;
  const bool customMac = server.hasArg("useCustomMac");
  if (!parsedNumbers || !validHostname(hostname) ||
      ssid.length() == 0 || ssid.length() > 32 || password.length() > 64 ||
      !validPin || brightnessValue < 0 || brightnessValue > 255 ||
      speedValue < 10 || speedValue > 1000 || power < 5.0f || power > 20.5f ||
      (server.arg("direction") != "left" &&
       server.arg("direction") != "right") ||
      !parseColor(server.arg("textColor"), color) ||
      (!mac.isEmpty() && !parseMac(mac, parsedMac)) ||
      (customMac && mac.isEmpty()) ||
      (password.length() > 0 && password.length() < 8)) {
    server.send(400, "text/plain", "Invalid configuration value");
    return;
  }

  cfg.hostname = hostname;
  cfg.staSsid = ssid;
  if (!password.isEmpty())
    cfg.staPassword = password;
  if (!mac.isEmpty())
    cfg.mac = mac;
  cfg.useCustomMac = customMac;
  cfg.wifiPower = power;
  cfg.displayPin = static_cast<int8_t>(pinValue);
  cfg.brightness = static_cast<uint8_t>(brightnessValue);
  cfg.textColor = color;
  cfg.serpentine = server.hasArg("serpentine");
  cfg.flipHorizontal = server.hasArg("flipHorizontal");
  cfg.scrollSpeed = static_cast<uint16_t>(speedValue);
  cfg.scrollRight = server.arg("direction") == "right";
  bool wasScrolling = cfg.scrollEnabled;
  cfg.scrollEnabled = server.hasArg("scrollEnabled");
  cfg.arduinoOtaEnabled = server.hasArg("arduinoOtaEnabled");
  cfg.debugSerial = server.hasArg("debugSerial");
  if (!saveConfig()) {
    server.send(500, "text/plain", "Could not save configuration");
    return;
  }
  if (wasScrolling != cfg.scrollEnabled) {
    if (cfg.scrollEnabled)
      scrollStart();
    else
      scrollStop();
  }
  server.send(200, "text/html; charset=utf-8",
              F("<!doctype html><html><body><p>Configuration saved. Rebooting "
                "now...</p></body></html>"));
  server.client().flush();
  delay(250);
  ESP.restart();
}

void handleFactoryReset() {
  if (!ensureAuthorized())
    return;
  performFactoryResetAndRestart(F("Factory reset requested from web UI."));
}

void handleStatus() {
  uint32_t uptime = bootStartMillis ? (millis() - bootStartMillis) / 1000UL : 0;
  String json = F("{\"version\":\"");
  json += SOFTWARE_VERSION;
  json += F("\",\"ip\":\"");
  json += WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString()
                                         : WiFi.softAPIP().toString();
  json += F("\",\"mac\":\"");
  json += WiFi.macAddress();
  json += F("\",\"heap\":");
  json += String(ESP.getFreeHeap());
  json += F(",\"uptime\":");
  json += String(uptime);
  json += F(",\"text\":\"");
  json += jsonEscape(cfg.text);
  json += F("\",\"scrollEnabled\":");
  json += cfg.scrollEnabled ? F("true") : F("false");
  json += F(",\"direction\":\"");
  json += cfg.scrollRight ? F("right") : F("left");
  json += F("\",\"speed\":");
  json += String(cfg.scrollSpeed);
  json += '}';
  server.send(200, "application/json", json);
}

void handleTextPost() {
  String payload = server.arg("plain");
  if (payload.length() > 1024) {
    server.send(413, "application/json",
                F("{\"error\":\"Request body is too large\"}"));
    return;
  }
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error || !doc["text"].is<String>()) {
    server.send(400, "application/json",
                F("{\"error\":\"Expected a JSON text string\"}"));
    return;
  }
  String text = doc["text"].as<String>();
  if (text.length() > 128) {
    server.send(400, "application/json",
                F("{\"error\":\"Text must be at most 128 characters\"}"));
    return;
  }
  displaySetText(text);
  if (!saveConfig()) {
    server.send(500, "application/json",
                F("{\"error\":\"Could not save text\"}"));
    return;
  }
  server.send(200, "application/json", F("{\"ok\":true}"));
}

void handleScrollPost() {
  String payload = server.arg("plain");
  if (payload.length() > 512) {
    server.send(413, "application/json",
                F("{\"error\":\"Request body is too large\"}"));
    return;
  }
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    server.send(400, "application/json", F("{\"error\":\"Invalid JSON\"}"));
    return;
  }
  bool enabled = doc["enabled"] | cfg.scrollEnabled;
  const char *direction = doc["direction"] | (cfg.scrollRight ? "right" : "left");
  int speed = doc["speed"] | cfg.scrollSpeed;
  if ((strcmp(direction, "left") != 0 && strcmp(direction, "right") != 0) ||
      speed < 10 || speed > 1000) {
    server.send(400, "application/json",
                F("{\"error\":\"Invalid scroll direction or speed\"}"));
    return;
  }
  scrollSetDirection(strcmp(direction, "right") == 0 ? ScrollDirection::Right
                                                      : ScrollDirection::Left);
  scrollSetSpeed(speed);
  if (enabled)
    scrollStart();
  else
    scrollStop();
  if (!saveConfig()) {
    server.send(500, "application/json",
                F("{\"error\":\"Could not save scroll settings\"}"));
    return;
  }
  server.send(200, "application/json", F("{\"ok\":true}"));
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}
}
