#include "WebServer.h"
#include "BootDiagnostics.h"
#include "Config.h"
#include "Debug.h"
#include "Display.h"
#include "NtpClock.h"
#include "OtaService.h"
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
  server.on("/config/network", HTTP_GET, handleConfigNetwork);
  server.on("/config/network", HTTP_POST, handleConfigNetworkSave);
  server.on("/config/display", HTTP_GET, handleConfigDisplay);
  server.on("/config/display", HTTP_POST, handleConfigDisplaySave);
  server.on("/config/ota", HTTP_GET, handleConfigOta);
  server.on("/config/ota", HTTP_POST, handleConfigOtaSave);
  server.on("/config/diagnostics", HTTP_GET, handleConfigDiagnostics);
  server.on("/config/diagnostics", HTTP_POST, handleConfigDiagnosticsSave);
  server.on("/config/factory-reset", HTTP_POST, handleFactoryReset);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/text", HTTP_POST, handleTextPost);
  server.on("/api/scroll", HTTP_POST, handleScrollPost);
  server.on("/api/leds", HTTP_POST, handleLedsPost);
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

// Builds the round-pill navigation bar shown at the top of every page, with
// the current page's button highlighted light blue (matching C4-VIBRANT's
// settings-nav/.nav-btn.current style).
static String pageNavigation(const char *activePage) {
  const char *paths[] = {"/", "/config/network", "/config/display",
                         "/config/ota", "/config/diagnostics"};
  const char *labels[] = {"Home", "Network", "Matrix display", "OTA",
                          "Diagnostics"};
  const char *pages[] = {"home", "network", "display", "ota", "diagnostics"};
  String html = F("<nav class='page-nav' aria-label='Page navigation'>");
  for (uint8_t i = 0; i < 5; ++i) {
    bool current = String(activePage) == pages[i];
    html += F("<a class='nav-btn");
    if (current)
      html += F(" current");
    html += F("' href='");
    html += paths[i];
    html += '\'';
    if (current)
      html += F(" aria-current='page'");
    html += '>';
    html += labels[i];
    html += F("</a>");
  }
  html += F("</nav>");
  return html;
}

// Shared page header (doctype, style, title, heading, and nav bar) used by
// the home page and every configuration sub-page so the menu is identical
// everywhere. The heading is uniformly "C4-MATRIX - <menuItemName>",
// centered, matching the active nav item's label.
static String pageStart(const char *menuItemName, const char *activePage) {
  String heading = F("C4-MATRIX - ");
  heading += menuItemName;
  String html = F("<!doctype html><html><head><meta charset='utf-8'><meta "
                  "name='viewport' content='width=device-width,initial-scale=1'>"
                  "<title>");
  html += heading;
  html += F("</title>");
  html += FPSTR(MATRIX_PAGE_STYLE);
  html += F("</head><body><h1>");
  html += heading;
  html += F("</h1>");
  html += pageNavigation(activePage);
  return html;
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
  String html = pageStart("Home", "home");
  html += F("<form id='text-form'><label>Text to "
            "display<input id='text' name='text' maxlength='128' "
            "autocomplete='off' required></label><button type='submit'>"
            "Display text</button></form><fieldset><legend>All LEDs</legend>"
            "<button type='button' data-state='on'>All ON</button>"
            "<button type='button' data-state='off'>All OFF</button>"
            "<button type='button' data-state='clock'>Show Clock</button>"
            "<button type='button' data-color='red'>Red</button>"
            "<button type='button' data-color='green'>Green</button>"
            "<button type='button' data-color='blue'>Blue</button>"
            "<form id='color-form'><label>Custom RGB color"
            "<input type='color' id='fill-color' value='");
  html += colorHex(cfg.fillColor);
  html += F("'></label><button type='submit'>Apply</button></form>"
            "<form id='count-form'><label>Light LEDs 1-N (N from 0 to ");
  html += String(displayLedCount());
  html += F(")<input type='number' id='led-count' name='ledCount' min='0' max='");
  html += String(displayLedCount());
  html += F("' value='");
  html += String(cfg.ledCount);
  html += F("'></label><button type='submit'>Light LEDs 1-N</button></form>"
            "</fieldset>"
            "<p id='message' role='status'></p>");
  html += FPSTR(HOME_PAGE_SCRIPT);
  html += F("</body></html>");
  server.send(200, "text/html; charset=utf-8", html);
}

void handleConfig() {
  if (!ensureAuthorized())
    return;
  String html = pageStart("Configuration", "");
  html += F("<p>Choose a settings page:</p><ul>"
            "<li><a href='/config/network'>Network</a> &ndash; hostname, "
            "Wi-Fi, and MAC address</li>"
            "<li><a href='/config/display'>Matrix display</a> &ndash; output "
            "pin, matrix size, brightness, color, and scrolling</li>"
            "<li><a href='/config/ota'>OTA</a> &ndash; ArduinoOTA and web "
            "firmware update</li>"
            "<li><a href='/config/diagnostics'>Diagnostics</a> &ndash; "
            "status, debug logging, and factory reset</li></ul>"
            "</body></html>");
  server.send(200, "text/html; charset=utf-8", html);
}

void handleConfigNetwork() {
  if (!ensureAuthorized())
    return;
  String html = pageStart("Network", "network");
  html += F("<form id='network-form' method='post' action='/config/network'>"
            "<fieldset><legend>Network</legend><label>Hostname<input "
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
  html += F("'></label></fieldset><fieldset><legend>Timezone</legend><label>"
            "UTC offset (minutes, -720 to 840)<input type='number' "
            "name='utcOffsetMinutes' min='-720' max='840' step='15' "
            "value='");
  html += String(cfg.utcOffsetMinutes);
  html += F("'></label></fieldset><button type='submit'>Apply</button>"
            "</form><p id='message' role='status'></p>");
  html += FPSTR(CONFIG_FORM_SCRIPT);
  html += F("<script>wireAutoApplyForm('network-form','/config/network');"
            "</script></body></html>");
  server.send(200, "text/html; charset=utf-8", html);
}

void handleConfigNetworkSave() {
  if (!ensureAuthorized())
    return;
  if (!server.hasArg("hostname") || !server.hasArg("staSsid") ||
      !server.hasArg("wifiPower")) {
    server.send(400, "text/plain", "Missing configuration field");
    return;
  }
  String hostname = server.arg("hostname");
  String ssid = server.arg("staSsid");
  String password = server.arg("staPassword");
  String mac = server.arg("mac");
  float power = 0;
  uint8_t parsedMac[6];
  const bool customMac = server.hasArg("useCustomMac");
  long utcOffsetValue = cfg.utcOffsetMinutes;
  if (server.hasArg("utcOffsetMinutes") &&
      !parseIntegerArg(server.arg("utcOffsetMinutes"), utcOffsetValue)) {
    server.send(400, "text/plain", "Invalid configuration value");
    return;
  }
  const bool validUtcOffset = utcOffsetValue >= -720 && utcOffsetValue <= 840;
  if (!parseFloatArg(server.arg("wifiPower"), power) ||
      !validHostname(hostname) || ssid.length() == 0 || ssid.length() > 32 ||
      password.length() > 64 || power < 5.0f || power > 20.5f ||
      (!mac.isEmpty() && !parseMac(mac, parsedMac)) ||
      (customMac && mac.isEmpty()) ||
      (password.length() > 0 && password.length() < 8) || !validUtcOffset) {
    server.send(400, "text/plain", "Invalid configuration value");
    return;
  }
  // The SoftAP identity (SSID) is derived from the MAC address, so only a
  // MAC/custom-MAC change requires restarting the SoftAP radio. Restarting
  // the SoftAP live (e.g. via applyWifiSettings()) while a client is
  // attached to it -- including the browser making this very request -- can
  // drop connectivity until the device is power-cycled, so that path is
  // only taken via a full, safe reboot below.
  const bool macChanged =
      customMac != cfg.useCustomMac || (customMac && mac != cfg.mac);
  const bool hostnameChanged = hostname != cfg.hostname;
  const bool powerChanged = fabsf(power - cfg.wifiPower) > 0.01f;
  const bool staChanged =
      ssid != cfg.staSsid || (!password.isEmpty() && password != cfg.staPassword);
  cfg.hostname = hostname;
  cfg.staSsid = ssid;
  if (!password.isEmpty())
    cfg.staPassword = password;
  if (!mac.isEmpty())
    cfg.mac = mac;
  cfg.useCustomMac = customMac;
  cfg.wifiPower = power;
  cfg.utcOffsetMinutes = static_cast<int16_t>(utcOffsetValue);
  if (!saveConfig()) {
    server.send(500, "text/plain", "Could not save configuration");
    return;
  }
  if (macChanged) {
    // Changing the MAC address changes the SoftAP SSID; apply it via a
    // full, safe reboot instead of a live SoftAP restart.
    server.send(200, "text/plain",
                "MAC address change saved. Rebooting to apply...");
    server.client().flush();
    delay(250);
    ESP.restart();
    return;
  }
  // Apply the remaining settings live, without touching the SoftAP.
  if (hostnameChanged)
    WiFi.hostname(cfg.hostname);
  if (powerChanged)
    WiFi.setOutputPower(cfg.wifiPower);
  if (staChanged) {
    WiFi.disconnect(false);
    WiFi.begin(cfg.staSsid.c_str(), cfg.staPassword.c_str());
  }
  server.send(200, "text/plain", "Network settings applied.");
}

void handleConfigDisplay() {
  if (!ensureAuthorized())
    return;
  String html = pageStart("Matrix Display", "display");
  html += F("<form id='display-form' method='post' action='/config/display'>"
            "<fieldset><legend>Display</legend>"
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
  html += F("</select></label><label>Matrix width in LEDs (1-");
  html += String(MATRIX_MAX_WIDTH);
  html += F(")<input type='number' name='matrixWidth' min='1' max='");
  html += String(MATRIX_MAX_WIDTH);
  html += F("' value='");
  html += String(cfg.matrixWidth);
  html += F("'></label><label>Matrix height in LEDs (1-");
  html += String(MATRIX_MAX_HEIGHT);
  html += F(")<input type='number' name='matrixHeight' min='1' max='");
  html += String(MATRIX_MAX_HEIGHT);
  html += F("' value='");
  html += String(cfg.matrixHeight);
  html += F("'></label><label>Brightness (0-255)<input "
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
  html += F("'></label></fieldset><button type='submit'>Apply</button>"
            "</form><p id='message' role='status'></p>");
  html += FPSTR(CONFIG_FORM_SCRIPT);
  html += F("<script>wireAutoApplyForm('display-form','/config/display');"
            "</script></body></html>");
  server.send(200, "text/html; charset=utf-8", html);
}

void handleConfigDisplaySave() {
  if (!ensureAuthorized())
    return;
  if (!server.hasArg("displayPin") || !server.hasArg("brightness") ||
      !server.hasArg("textColor") || !server.hasArg("scrollSpeed") ||
      !server.hasArg("direction") || !server.hasArg("matrixWidth") ||
      !server.hasArg("matrixHeight")) {
    server.send(400, "text/plain", "Missing configuration field");
    return;
  }
  long pinValue = -1, brightnessValue = -1, speedValue = -1;
  long matrixWidthValue = -1, matrixHeightValue = -1;
  bool parsedNumbers = parseIntegerArg(server.arg("displayPin"), pinValue) &&
                       parseIntegerArg(server.arg("brightness"),
                                       brightnessValue) &&
                       parseIntegerArg(server.arg("scrollSpeed"), speedValue) &&
                       parseIntegerArg(server.arg("matrixWidth"),
                                       matrixWidthValue) &&
                       parseIntegerArg(server.arg("matrixHeight"),
                                       matrixHeightValue);
  uint32_t color;
  const bool validPin = pinValue == 16 || pinValue == 5 || pinValue == 4 ||
                        pinValue == 0 || pinValue == 2 || pinValue == 14 ||
                        pinValue == 12 || pinValue == 13 || pinValue == 15;
  const bool validMatrixSize =
      matrixWidthValue >= 1 && matrixWidthValue <= MATRIX_MAX_WIDTH &&
      matrixHeightValue >= 1 && matrixHeightValue <= MATRIX_MAX_HEIGHT;
  if (!parsedNumbers || !validPin || brightnessValue < 0 ||
      brightnessValue > 255 || speedValue < 10 || speedValue > 1000 ||
      !validMatrixSize ||
      (server.arg("direction") != "left" &&
       server.arg("direction") != "right") ||
      !parseColor(server.arg("textColor"), color)) {
    server.send(400, "text/plain", "Invalid configuration value");
    return;
  }
  // A strip re-pin/resize is only needed (and briefly blanks the matrix)
  // when the pin or matrix dimensions actually change.
  bool reinitStrip = cfg.displayPin != static_cast<int8_t>(pinValue) ||
                     cfg.matrixWidth != static_cast<uint16_t>(matrixWidthValue) ||
                     cfg.matrixHeight !=
                         static_cast<uint16_t>(matrixHeightValue);
  // scrollSetDirection()/scrollStart() reset the scroll position, so only
  // call them when the direction or enabled state actually changed --
  // otherwise an unrelated field change (e.g. color) would restart the
  // scroll from the beginning instead of letting it continue smoothly.
  bool directionChanged = (server.arg("direction") == "right") != cfg.scrollRight;
  bool scrollEnabledChanged =
      server.hasArg("scrollEnabled") != cfg.scrollEnabled;
  cfg.displayPin = static_cast<int8_t>(pinValue);
  cfg.brightness = static_cast<uint8_t>(brightnessValue);
  cfg.textColor = color;
  cfg.matrixWidth = static_cast<uint16_t>(matrixWidthValue);
  cfg.matrixHeight = static_cast<uint16_t>(matrixHeightValue);
  uint16_t configuredLedCount = cfg.matrixWidth * cfg.matrixHeight;
  if (cfg.ledCount > configuredLedCount)
    cfg.ledCount = configuredLedCount;
  cfg.serpentine = server.hasArg("serpentine");
  cfg.flipHorizontal = server.hasArg("flipHorizontal");
  cfg.scrollSpeed = static_cast<uint16_t>(speedValue);
  cfg.scrollRight = server.arg("direction") == "right";
  cfg.scrollEnabled = server.hasArg("scrollEnabled");
  if (!saveConfig()) {
    server.send(500, "text/plain", "Could not save configuration");
    return;
  }
  // Apply live, without a reboot: re-pin/resize the strip only if needed,
  // then re-apply brightness, color, orientation, and scrolling.
  if (reinitStrip)
    displayBegin(cfg.displayPin);
  displaySetBrightness(cfg.brightness);
  displaySetColor(cfg.textColor);
  displaySetSerpentine(cfg.serpentine);
  displaySetOrientation(cfg.flipHorizontal);
  if (directionChanged)
    scrollSetDirection(cfg.scrollRight ? ScrollDirection::Right
                                       : ScrollDirection::Left);
  scrollSetSpeed(cfg.scrollSpeed);
  if (scrollEnabledChanged) {
    if (cfg.scrollEnabled)
      scrollStart();
    else
      scrollStop();
  }
  server.send(200, "text/plain", "Display settings applied.");
}

void handleConfigOta() {
  if (!ensureAuthorized())
    return;
  String html = pageStart("OTA", "ota");
  html += F("<form id='ota-form' method='post' action='/config/ota'>"
            "<fieldset><legend>ArduinoOTA</legend><label>ArduinoOTA enabled ");
  html += checkbox("arduinoOtaEnabled", cfg.arduinoOtaEnabled);
  html += F("</label></fieldset><button type='submit'>Apply</button>"
            "</form><p id='message' role='status'></p>"
            "<p><a href='/update'>Web firmware update</a>"
            "</p></body></html>");
  server.send(200, "text/html; charset=utf-8", html);
}

void handleConfigOtaSave() {
  if (!ensureAuthorized())
    return;
  cfg.arduinoOtaEnabled = server.hasArg("arduinoOtaEnabled");
  if (!saveConfig()) {
    server.send(500, "text/plain", "Could not save configuration");
    return;
  }
  // Apply live: starts or stops the ArduinoOTA listener immediately.
  applyArduinoOtaSettings();
  server.send(200, "text/plain", "OTA settings applied.");
}

void handleConfigDiagnostics() {
  if (!ensureAuthorized())
    return;
  String html = pageStart("Diagnostics", "diagnostics");
  html += F("<fieldset><legend>Status</legend><div id='status' class='status'>"
            "Loading...</div></fieldset>"
            "<form id='diagnostics-form' method='post' "
            "action='/config/diagnostics'>"
            "<fieldset><legend>Diagnostics</legend><label>Serial debug "
            "logging ");
  html += checkbox("debugSerial", cfg.debugSerial);
  html += F("</label></fieldset><button type='submit'>Apply</button>"
            "</form><p id='message' role='status'></p>"
            "<form method='post' "
            "action='/config/factory-reset' onsubmit=\"return "
            "confirm('Restore factory defaults and reboot?')\">"
            "<button type='submit'>Factory reset</button></form>");
  html += FPSTR(CONFIG_FORM_SCRIPT);
  html += F("<script>wireAutoApplyForm('diagnostics-form',"
            "'/config/diagnostics');"
            "async function refreshStatus(){try{let r=await fetch('/api/status');"
            "let s=await r.json();document.querySelector('#status').textContent="
            "`Version: ${s.version}\\nIP: ${s.ip}\\nMAC: ${s.mac}\\nHeap: "
            "${s.heap} bytes\\nUptime: ${s.uptime} s\\nLED mode: ${s.mode}"
            "\\nFill color: ${s.fillColor}\\nText: ${s.text}\\nScroll: "
            "${s.scrollEnabled?'on':'off'} ${s.direction}, ${s.speed} ms/column"
            "\\nClock: ${s.clockSynced?'synced':'not synced'} (${s.clockTime})`}"
            "}catch(e){document.querySelector('#status').textContent='Status "
            "unavailable.'}}refreshStatus();</script></body></html>");
  server.send(200, "text/html; charset=utf-8", html);
}

void handleConfigDiagnosticsSave() {
  if (!ensureAuthorized())
    return;
  cfg.debugSerial = server.hasArg("debugSerial");
  if (!saveConfig()) {
    server.send(500, "text/plain", "Could not save configuration");
    return;
  }
  server.send(200, "text/plain", "Diagnostics settings applied.");
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
  json += F(",\"mode\":\"");
  json += displayModeName(cfg.mode);
  json += F("\",\"fillColor\":\"");
  json += colorHex(cfg.fillColor);
  json += F("\",\"ledCount\":");
  json += String(cfg.ledCount);
  json += F(",\"maxLedCount\":");
  json += String(displayLedCount());
  json += F(",\"clockTime\":\"");
  json += ntpTimeString();
  json += F("\",\"clockSynced\":");
  json += ntpTimeValid() ? F("true") : F("false");
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

void handleLedsPost() {
  String payload = server.arg("plain");
  if (payload.length() > 512) {
    server.send(413, "application/json",
                F("{\"error\":\"Request body is too large\"}"));
    return;
  }
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload);
  JsonObjectConst command = doc.as<JsonObjectConst>();
  DisplayMode mode = DisplayMode::Fill;
  uint32_t color = cfg.fillColor;
  uint16_t ledCount = cfg.ledCount;
  bool valid = false;
  bool explicitColorChosen = false;
  if (!error && command.size() == 1 && command["state"].is<String>()) {
    String state = command["state"].as<String>();
    valid = state == "on" || state == "off" || state == "clock";
    mode = state == "off"     ? DisplayMode::Off
           : state == "clock" ? DisplayMode::Clock
                               : DisplayMode::Fill;
  } else if (!error && command.size() == 1 &&
             command["color"].is<String>()) {
    String name = command["color"].as<String>();
    valid = name == "red" || name == "green" || name == "blue";
    color = name == "red" ? 0xFF0000UL
                         : name == "green" ? 0x00FF00UL : 0x0000FFUL;
    explicitColorChosen = valid;
  } else if (!error && command.size() == 3 &&
             command["r"].is<int>() && command["g"].is<int>() &&
             command["b"].is<int>()) {
    int r = command["r"].as<int>();
    int g = command["g"].as<int>();
    int b = command["b"].as<int>();
    valid = r >= 0 && r <= 255 && g >= 0 && g <= 255 && b >= 0 && b <= 255;
    if (valid) {
      color = (static_cast<uint32_t>(r) << 16) |
              (static_cast<uint32_t>(g) << 8) | static_cast<uint32_t>(b);
      explicitColorChosen = true;
    }
  } else if (!error && command.size() >= 1 && command["count"].is<int>() &&
             (command.size() == 1 ||
              (command.size() == 4 && command["r"].is<int>() &&
               command["g"].is<int>() && command["b"].is<int>()))) {
    int count = command["count"].as<int>();
    valid = count >= 0 && count <= displayLedCount();
    mode = DisplayMode::Count;
    if (valid && command.size() == 4) {
      int r = command["r"].as<int>();
      int g = command["g"].as<int>();
      int b = command["b"].as<int>();
      valid = r >= 0 && r <= 255 && g >= 0 && g <= 255 && b >= 0 && b <= 255;
      if (valid) {
        color = (static_cast<uint32_t>(r) << 16) |
                (static_cast<uint32_t>(g) << 8) | static_cast<uint32_t>(b);
        explicitColorChosen = true;
      }
    }
    if (valid)
      ledCount = static_cast<uint16_t>(count);
  }
  if (!valid) {
    DBG("Rejected invalid LED command");
    server.send(400, "application/json",
                F("{\"error\":\"Expected state on/off/clock, color "
                  "red/green/blue, "
                  "integer r, g, b from 0 to 255, or integer count from 0 to "
                  "the LED total (optionally with r, g, b)\"}"));
    return;
  }
  if (mode == DisplayMode::Count)
    displaySetLedCount(color, ledCount);
  else
    displaySetLeds(mode, color);
  if (explicitColorChosen)
    displaySetColor(color);
  if (!saveConfig()) {
    server.send(500, "application/json",
                F("{\"error\":\"Could not save LED settings\"}"));
    return;
  }
  server.send(200, "application/json", F("{\"ok\":true}"));
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}
}
