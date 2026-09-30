#include "WebServer.h"
#include "Actions.h"
#include "BackupRestore.h"
#include "BootDiagnostics.h"
#include "Config.h"
#include "Debug.h"
#include "MqttClient.h"
#include "Outputs.h"
#include "Reservations.h"
#include "Runtime.h"
#include "Stickserver.h"
#include "Version.h"
#include "WebAssets.h"
#include "WebUpdate.h"
#include "WifiManager.h"
#include "../libraries/EmcommCommon/src/EmcommCommon/Web.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP8266WebServer.h>

namespace vibrant {

ESP8266WebServer server(80);

void registerWebRoutes() {
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
  server.on("/config/import", HTTP_POST, handleConfigImportDone,
            handleConfigImportUpload);
  server.on("/config/factory-reset", HTTP_POST, handleFactoryReset);
  server.on("/firmware/update", HTTP_GET, handleFirmwareUpdatePage);
  server.on("/firmware/update", HTTP_POST, handleFirmwareUpdateDone,
            handleFirmwareUpdateUpload);
  server.onNotFound(handleNotFound);
  server.begin();
  logStatus(F("HTTP server started on port 80."));
}

String pinOption(int selectedPin, const PinMapping &mapping) {
  String selected = (selectedPin == mapping.gpio) ? " selected" : "";
  return "<option value=\"" + String(mapping.gpio) + "\"" + selected + ">" +
         String(mapping.label) + "</option>";
}

bool parsePinValue(const String &raw, int &pin) {
  if (raw == "-1") {
    pin = -1;
    return true;
  }
  if (raw.isEmpty())
    return false;
  for (size_t i = 0; i < raw.length(); ++i) {
    if (raw[i] < '0' || raw[i] > '9')
      return false;
  }
  pin = raw.toInt();
  return isValidOutputPin(pin);
}

bool parseIndexValue(const String &raw, int &value) {
  if (raw.isEmpty())
    return false;
  for (size_t i = 0; i < raw.length(); ++i) {
    if (raw[i] < '0' || raw[i] > '9')
      return false;
  }
  value = raw.toInt();
  return true;
}

bool parseFloatValue(const String &raw, float &value) {
  if (raw.isEmpty())
    return false;
  char *endPtr = nullptr;
  value = strtof(raw.c_str(), &endPtr);
  return endPtr != raw.c_str() && *endPtr == '\0';
}

bool usingFactoryPassword() {
  return cfg.apPassword == DEFAULT_AP_PASSWORD ||
         cfg.staPassword == DEFAULT_STA_PASSWORD;
}

String passwordWarningHtml() {
  return F("<p style='color:#b00020;'><strong>Warning:</strong> Factory "
           "default Wi-Fi password is active. "
           "Change it now for security.</p>");
}

bool ensureAuthorized() {
  if (server.authenticate("admin", cfg.apPassword.c_str()))
    return true;
  server.requestAuthentication();
  return false;
}

void handleHome() {
  bool actionRunning = isActionRunning();
  String html = FPSTR(HOME_PAGE_HEADER);
  html += SOFTWARE_VERSION;
  html += F("</p><p><a href='/settings'>Settings</a></p>");
  if (usingFactoryPassword()) {
    html += passwordWarningHtml();
  }
  // Initial action-status banner rendered server-side; JS polling keeps it
  // updated. The phase-detail string is also formatted by the JS updater; they
  // share the same visual format but run in different contexts (C++/server vs
  // JS/browser).
  html += "<div id='action-status'>";
  if (actionRunning) {
    String phaseDetail = isInCyclePhase()
                             ? String(F(" (cycles remaining: ")) +
                                   String(bgAction.cyclesRemaining) + ")"
                             : "";
    String target = bgAction.allOutputs
                        ? String(F("all outputs"))
                        : String(F("output ")) + String(bgAction.deviceIdx + 1);
    html += "<div class='action-banner'><strong>Action running on " + target +
            ": " + actionPhaseName() + phaseDetail +
            "</strong>"
            " &nbsp; <form method='post' action='/action/cancel' "
            "style='display:inline;'>"
            "<button type='submit'>Cancel</button></form></div>";
  }
  html += "</div>";

  // Global bulk-action buttons
  const char *bulkDisabled = actionRunning ? " disabled" : "";
  html +=
      String(F("<div style='margin:10px 0;'>")) +
      "<form method='post' action='/action/all-on' "
      "style='display:inline;margin:0;'>"
      "<button type='submit'" +
      bulkDisabled + ">Turn ON all</button></form>" +
      "<form method='post' action='/action/all-off' "
      "style='display:inline;margin:0;'>"
      "<button type='submit'" +
      bulkDisabled + ">Turn OFF all</button></form>" +
      "<form method='post' action='/action/leave-mesh-all' "
      "style='display:inline;margin:0;'"
      " onsubmit=\"return confirm('Run leave mesh signal on ALL outputs?');\">"
      "<button type='submit'" +
      bulkDisabled + ">Leave Mesh All</button></form>" +
      "<form method='post' action='/action/factory-reset-all' "
      "style='display:inline;margin:0;'"
      " onsubmit=\"return confirm('Run factory reset signal on ALL "
      "outputs?');\">"
      "<button type='submit'" +
      bulkDisabled + ">Factory Reset All</button></form>" + "</div>";

  html += F("<table><tr><th>#</th><th>Model</th><th>Name</th><th>Status</"
            "th><th>Output</th><th>Actions</th></tr>");

  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    const DeviceEntry &d = cfg.devices[i];
    bool mapped = isValidOutputPin(d.pin);
    String status = mapped ? (d.state ? String(F("ON")) : String(F("OFF")))
                           : String(F("Unassigned"));
    bool thisActionRunning = actionOwnsOutput(i);
    bool otherActionRunning = actionRunning && !thisActionRunning;

    html += "<tr><td>" + String(i + 1) + "</td><td>" + htmlEscape(d.model) +
            "</td><td>" + htmlEscape(d.name) + "</td><td>" + status +
            "</td><td>";

    if (mapped) {
      html += "<form method='post' action='/toggle' style='margin:0;'>"
              "<input type='hidden' name='idx' value='" +
              String(i) +
              "'>"
              "<input type='hidden' name='state' id='state_" +
              String(i) + "' value='" + String(d.state ? 1 : 0) +
              "'>"
              "<input type='checkbox'";
      if (d.state)
        html += " checked";
      html += " onchange=\"document.getElementById('state_" + String(i) +
              "').value=this.checked?1:0;this.form.submit();\">"
              "</form>";
    } else {
      html += F("(none)");
    }

    html += "</td><td>";
    if (mapped) {
      if (thisActionRunning) {
        html += F("<em>Running...</em>");
      } else {
        const char *disabledAttr = otherActionRunning ? " disabled" : "";
        html += "<form method='post' action='/action' "
                "style='display:inline;margin:0;'>"
                "<input type='hidden' name='idx' value='" +
                String(i) +
                "'>"
                "<input type='hidden' name='cmd' value='power_on'>"
                "<button type='submit'" +
                disabledAttr +
                ">On</button></form>"
                "<form method='post' action='/action' "
                "style='display:inline;margin:0;'>"
                "<input type='hidden' name='idx' value='" +
                String(i) +
                "'>"
                "<input type='hidden' name='cmd' value='power_off'>"
                "<button type='submit'" +
                disabledAttr +
                ">Off</button></form>"
                "<form method='post' action='/action' "
                "style='display:inline;margin:0;'>"
                "<input type='hidden' name='idx' value='" +
                String(i) +
                "'>"
                "<input type='hidden' name='cmd' value='leave_mesh'>"
                "<button type='submit'" +
                disabledAttr +
                ">Leave Mesh</button></form>"
                "<form method='post' action='/action' "
                "style='display:inline;margin:0;'>"
                "<input type='hidden' name='idx' value='" +
                String(i) +
                "'>"
                "<input type='hidden' name='cmd' value='factory_reset'>"
                "<button type='submit'" +
                disabledAttr + ">Factory Reset</button></form>";
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
  if (!ensureAuthorized())
    return;

  if (!server.hasArg("idx") || !server.hasArg("state")) {
    logError(F("Toggle request missing idx or state."));
    server.send(400, "text/plain", "Missing idx or state");
    return;
  }

  int idx = -1;
  if (!parseIndexValue(server.arg("idx"), idx) || idx < 0 ||
      idx >= cfg.numOutputs) {
    logError(F("Toggle request contained invalid device index."));
    server.send(400, "text/plain", "Invalid device index");
    return;
  }
  if (server.arg("state") != "0" && server.arg("state") != "1") {
    logError(F("Toggle request contained invalid state value."));
    server.send(400, "text/plain", "Invalid state value");
    return;
  }

  DeviceEntry &d = cfg.devices[idx];
  if (!isValidOutputPin(d.pin)) {
    logError(String(F("Toggle request for unmapped output: ")) + d.name);
    server.sendHeader("Location", "/");
    server.send(303);
    return;
  }

  // Abort any running sequence action on this output
  if (actionOwnsOutput(static_cast<uint8_t>(idx))) {
    cancelAction();
  }

  setOutputDirect(static_cast<uint8_t>(idx), server.arg("state") == "1");
  if (!outputsActivated)
    applyOutputsWhenSafe();
  mqttPublishOutputState(static_cast<uint8_t>(idx));
  Serial.print(F("[INFO] Output toggled: "));
  Serial.print(d.name);
  Serial.print(F(" -> "));
  Serial.println(d.state ? F("ON") : F("OFF"));
  // Runtime state changes are not persisted to flash by design.
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleAction() {
  if (!ensureAuthorized())
    return;
  if (!server.hasArg("idx") || !server.hasArg("cmd")) {
    server.send(400, "text/plain", "Missing idx or cmd");
    return;
  }
  int idx = -1;
  if (!parseIndexValue(server.arg("idx"), idx) || idx < 0 ||
      idx >= cfg.numOutputs) {
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
  if (!ensureAuthorized())
    return;
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
    json += ",\"idx\":" + String(bgAction.deviceIdx);
    json += ",\"allOutputs\":" + String(bgAction.allOutputs ? "true" : "false");
    json += ",\"phase\":\"" + actionPhaseName() + "\"";
    json += ",\"cyclesRemaining\":" +
            String(isInCyclePhase() ? bgAction.cyclesRemaining : 0);
  }
  json += "}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handleAllOn() {
  if (!ensureAuthorized())
    return;
  if (isActionRunning()) {
    server.send(409, "text/plain", "Another action is already running");
    return;
  }
  logStatus(F("Turn ON all outputs requested."));
  bool first = true;
  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    if (!isManagedOutput(i))
      continue;
    if (!first)
      delay(50);
    setOutputDirect(i, true);
    mqttPublishOutputState(i);
    first = false;
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleAllOff() {
  if (!ensureAuthorized())
    return;
  if (isActionRunning()) {
    server.send(409, "text/plain", "Another action is already running");
    return;
  }
  logStatus(F("Turn OFF all outputs requested."));
  bool first = true;
  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    if (!isManagedOutput(i))
      continue;
    if (!first)
      delay(50);
    setOutputDirect(i, false);
    mqttPublishOutputState(i);
    first = false;
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleFactoryResetAll() {
  if (!ensureAuthorized())
    return;
  if (isActionRunning()) {
    server.send(409, "text/plain", "Another action is already running");
    return;
  }
  logStatus(F("Factory reset ALL outputs requested."));
  if (managedOutputCount())
    startSequence(PROFILE_FACTORY_RESET, 0, true);
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleLeaveMeshAll() {
  if (!ensureAuthorized())
    return;
  if (isActionRunning()) {
    server.send(409, "text/plain", "Another action is already running");
    return;
  }
  logStatus(F("Leave mesh ALL outputs requested."));
  if (managedOutputCount())
    startSequence(PROFILE_LEAVE_MESH, 0, true);
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleSettingsGet() {
  if (!ensureAuthorized())
    return;

  String html = FPSTR(SETTINGS_PAGE_HEADER);
  html += FPSTR(SETTINGS_PAGE_SCRIPT);
  html += F("</head><body><h1>Settings</h1>"
            "<p><a href='/'>Back to main page</a></p><form method='post' "
            "action='/settings'>");
  if (usingFactoryPassword()) {
    html += passwordWarningHtml();
  }

  html += "<fieldset><legend>Station network</legend>"
          "<label>Station SSID <input name='staSsid' value='" +
          htmlEscape(cfg.staSsid) +
          "'></label>"
          "<label for='staPassword'>Station password</label><input "
          "id='staPassword' name='staPassword' type='password' value='' "
          "placeholder='Leave empty to keep current station password'>"
          "</fieldset>";

  html += "<fieldset><legend>Access point</legend>"
          "<label>SoftAP SSID <input value='" +
          htmlEscape(defaultSoftApSsidFromMac(cfg.mac)) +
          "' readonly></label>"
          "<label for='apPassword'>SoftAP password</label><input "
          "id='apPassword' name='apPassword' type='password' value='' "
          "placeholder='Leave empty to keep current AP password'>"
          "</fieldset>";

  html += "<fieldset><legend>Network device settings</legend>"
          "<label>MAC address <input name='mac' value='" +
          htmlEscape(cfg.mac) +
          "' maxlength='17'></label>"
          "<label><input type='checkbox' name='useCustomMac' value='1'" +
          String(cfg.useCustomMac ? " checked" : "") +
          "> Use custom MAC address</label>"
          "<label>Hostname for DHCP <input name='hostname' value='" +
          htmlEscape(cfg.hostname) +
          "' maxlength='32' pattern='[A-Za-z0-9-]*'></label>"
          "<label>Wi-Fi power (5.0 - 20.5 dBm) <input name='wifiPower' "
          "type='number' min='5' max='20.5' step='0.1' value='" +
          String(cfg.wifiPower, 1) +
          "'></label>"
          "</fieldset>";

  html += "<fieldset><legend>Devices</legend>"
          "<label>Number of outputs (1 - 16) <input name='numOutputs' "
          "type='number' min='1' max='16' step='1' value='" +
          String(cfg.numOutputs) +
          "'></label>"
          "<p>Each active output must use a different GPIO. TX/RX disable "
          "serial communication; GPIO0 (FLASH) and GPIO15 affect boot.</p>"
          "<div class='bulk-actions'><button type='button' "
          "onclick='copyFirstManufacturerToAll()'>Use first Manufacturer for "
          "all</button>"
          "<button type='button' onclick='copyFirstModelToAll()'>Use first "
          "Model for all</button>"
          "<button type='button' onclick='copyFirstNameToAll()'>Use first Name "
          "for all</button>"
          "<button type='button' onclick='clearAllFieldsExceptOutput()'>Clear "
          "all fields</button>"
          "<button type='button' onclick='reverseGpioAssignments()'>Reverse "
          "GPIO assignments</button>"
          "<span>If the first Name contains #5, copy keeps the first row at #5 "
          "and fills later rows as #6, #7, and so on.</span></div>"
          "<table><tr><th>#</th><th>Manufacturer</th><th>Model</th><th>Name</"
          "th><th>Control output</th></tr>";
  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    html += "<tr><td>" + String(i + 1) +
            "</td>"
            "<td><input name='manufacturer_" +
            String(i) + "' value='" + htmlEscape(cfg.devices[i].manufacturer) +
            "'></td>"
            "<td><input name='model_" +
            String(i) + "' value='" + htmlEscape(cfg.devices[i].model) +
            "'></td>"
            "<td><input name='name_" +
            String(i) + "' value='" + htmlEscape(cfg.devices[i].name) +
            "'></td>"
            "<td><select name='pin_" +
            String(i) + "'>";

    html += (cfg.devices[i].pin < 0)
                ? "<option value='-1' selected>none</option>"
                : "<option value='-1'>none</option>";
    for (size_t pinIndex = 0; pinIndex < OUTPUT_PIN_MAPPING_COUNT; ++pinIndex) {
      html += pinOption(cfg.devices[i].pin, OUTPUT_PIN_MAPPINGS[pinIndex]);
    }
    html += "</select></td></tr>";
  }
  html += "</table></fieldset>";

  html +=
      "<fieldset><legend>MQTT</legend>"
      "<label><input type='checkbox' name='mqttEnabled' value='1'" +
      String(cfg.mqttEnabled ? " checked" : "") +
      "> Enable MQTT</label>"
      "<label for='mqttHost'>MQTT server host</label>"
      "<input id='mqttHost' name='mqttHost' value='" +
      htmlEscape(cfg.mqttHost) +
      "' placeholder='e.g. 192.168.1.6'>"
      "<label for='mqttPort'>MQTT port</label>"
      "<input id='mqttPort' name='mqttPort' type='number' min='1' max='65535' "
      "value='" +
      String(cfg.mqttPort) +
      "'>"
      "<label for='mqttUser'>MQTT user (optional)</label>"
      "<input id='mqttUser' name='mqttUser' value='" +
      htmlEscape(cfg.mqttUser) +
      "'>"
      "<label for='mqttPassword'>MQTT password (optional)</label>"
      "<input id='mqttPassword' name='mqttPassword' type='password' value='' "
      "placeholder='Leave empty to keep current'"
      " oninput=\"document.getElementById('mqttPasswordClear').checked = "
      "false;\">"
      "<label><input type='checkbox' id='mqttPasswordClear' "
      "name='mqttPasswordClear' value='1'"
      " onchange=\"if(this.checked)document.getElementById('mqttPassword')."
      "value='';\"> Clear MQTT password (remove broker authentication)</label>"
      "<p style='font-size:0.9em;color:#555;'>Topics (N = zero-based output "
      "index, e.g. 0 = Output 1): "
      "<code>vibrant/" +
      htmlEscape(cfg.hostname) +
      "/out/&lt;N&gt;/set</code> (ON/OFF) &amp; "
      "<code>vibrant/" +
      htmlEscape(cfg.hostname) +
      "/out/&lt;N&gt;/action</code> (power_on / power_off / leave_mesh / "
      "factory_reset). "
      "Stickserver compatibility subscribes to <code>" +
      htmlEscape(String(STICKSERVER_ROOT_TOPIC)) +
      "</code> and replies on the device topic.</p>"
      "</fieldset>";

  html += "<fieldset><legend>Diagnostics</legend>"
          "<label><input type='checkbox' name='arduinoOtaEnabled' value='1'" +
          String(cfg.arduinoOtaEnabled ? " checked" : "") +
          "> Enable ArduinoOTA service (developer OTA via IDE/tools; uses "
          "admin password for auth)</label>"
          "<p style='font-size:0.9em;color:#555;'>When enabled, ArduinoOTA "
          "uses hostname <code>" +
          htmlEscape(cfg.hostname) +
          "</code> and requires the current admin password.</p>"
          "<label><input type='checkbox' name='debugSerial' value='1'" +
          String(cfg.debugSerial ? " checked" : "") +
          "> Enable verbose serial debug logging (euid parsing, idx "
          "resolution, etc.)</label>"
          "</fieldset>";

  html += F("<button type='submit'>Save settings</button></form>");

  html +=
      F("<h2>Configuration maintenance</h2>"
        "<p><a href='/config/export'>Download configuration backup</a></p>");
  html += "<p>Hold the FLASH button during power-on (during the first " +
          String(FLASH_BOOT_DETECTION_WINDOW_MS) +
          " milliseconds of boot) to trigger factory reset and restart.</p>"
          "<form method='post' action='/config/factory-reset' "
          "onsubmit=\"return confirm('Factory reset?');\">"
          "<button type='submit'>Factory reset</button></form>"
          "<form method='post' action='/config/import' "
          "enctype='multipart/form-data'>"
          "<label>Import backup JSON <input type='file' name='config' "
          "accept='application/json' required></label>"
          "<button type='submit'>Upload and restore</button></form>"
          "<h2>Firmware update</h2>"
          "<p>Upload a compiled <code>.bin</code> to update firmware over the "
          "network. "
          "The device reboots automatically after a successful flash.</p>"
          "<p><a href='/firmware/update'>Open firmware update page</a></p>";

  html += F("</body></html>");
  server.send(200, "text/html", html);
}

void handleSettingsPost() {
  if (!ensureAuthorized())
    return;

  String macValue = server.arg("mac");
  macValue.toUpperCase();
  uint8_t macBytes[6] = {0};
  if (!parseMac(macValue, macBytes)) {
    logError(F("Settings save rejected due to invalid MAC address."));
    server.send(400, "text/plain",
                "Invalid MAC address format. Use AA:BB:CC:DD:EE:FF");
    return;
  }

  String hostname = server.arg("hostname");
  if (hostname.length() > 32) {
    server.send(400, "text/plain",
                "Hostname must be at most 32 characters (letters, digits, "
                "hyphens only)");
    return;
  }
  for (size_t i = 0; i < hostname.length(); ++i) {
    char c = hostname[i];
    if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
          (c >= '0' && c <= '9') || c == '-')) {
      server.send(400, "text/plain",
                  "Hostname must contain only letters, digits, and hyphens");
      return;
    }
  }

  int parsedOutputs = -1;
  if (!parseIndexValue(server.arg("numOutputs"), parsedOutputs) ||
      parsedOutputs < 1 || parsedOutputs > MAX_DEVICES) {
    logError(F(
        "Settings save rejected: number of outputs must be between 1 and 16."));
    server.send(400, "text/plain", "Number of outputs must be 1-16");
    return;
  }
  bool usedPins[17] = {false};
  for (int i = 0; i < parsedOutputs; ++i) {
    int pin = -1;
    String pinArgName = "pin_" + String(i);
    if (server.hasArg(pinArgName) &&
        parsePinValue(server.arg(pinArgName), pin) && pin >= 0) {
      if (usedPins[pin]) {
        server.send(400, "text/plain",
                    "Duplicate GPIO assignment among active outputs");
        return;
      }
      usedPins[pin] = true;
    }
  }

  float parsedPower = 0.0f;
  if (!parseFloatValue(server.arg("wifiPower"), parsedPower)) {
    logError(F("Settings save rejected due to invalid Wi-Fi power value."));
    server.send(400, "text/plain", "Invalid Wi-Fi power value");
    return;
  }

  cfg.mac = macValue;
  cfg.useCustomMac =
      server.hasArg("useCustomMac") && server.arg("useCustomMac") == "1";
  cfg.hostname = hostname;
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

  if (cfg.staSsid.isEmpty() || cfg.staPassword.isEmpty() ||
      cfg.apPassword.isEmpty()) {
    logError(F("Settings save rejected because station SSID or passwords were "
               "empty."));
    server.send(
        400, "text/plain",
        "Station SSID, station password, and AP password must not be empty");
    return;
  }
  if (cfg.hostname.isEmpty())
    cfg.hostname = defaultHostnameFromMac(cfg.mac);

  cfg.numOutputs = static_cast<uint8_t>(parsedOutputs);

  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    cfg.devices[i].manufacturer = server.arg("manufacturer_" + String(i));
    cfg.devices[i].model = server.arg("model_" + String(i));
    String manufacturer = cfg.devices[i].manufacturer;
    String model = cfg.devices[i].model;
    manufacturer.trim();
    model.trim();
    if (manufacturer.isEmpty())
      cfg.devices[i].manufacturer = DEFAULT_DEVICE_MANUFACTURER;
    if (model.isEmpty())
      cfg.devices[i].model = DEFAULT_DEVICE_MODEL;
    cfg.devices[i].name = server.arg("name_" + String(i));
    int pin = -1;
    String pinArgName = "pin_" + String(i);
    if (!server.hasArg(pinArgName) ||
        !parsePinValue(server.arg(pinArgName), pin)) {
      pin = -1;
    }
    cfg.devices[i].pin = pin;
    if (!isValidOutputPin(cfg.devices[i].pin)) {
      cfg.devices[i].state = false;
    }
  }

  cfg.mqttEnabled =
      server.hasArg("mqttEnabled") && server.arg("mqttEnabled") == "1";
  cfg.mqttHost = server.arg("mqttHost");
  {
    int parsedPort = -1;
    if (parseIndexValue(server.arg("mqttPort"), parsedPort) &&
        parsedPort >= 1 && parsedPort <= 65535) {
      cfg.mqttPort = static_cast<uint16_t>(parsedPort);
    } else {
      cfg.mqttPort = DEFAULT_MQTT_PORT;
    }
  }
  cfg.mqttUser = server.arg("mqttUser");
  bool clearMqttPassword = server.hasArg("mqttPasswordClear") &&
                           server.arg("mqttPasswordClear") == "1";
  if (clearMqttPassword) {
    cfg.mqttPassword = "";
  } else {
    String newMqttPassword = server.arg("mqttPassword");
    String legacyMqttPassword = server.arg("mqttPass");
    if (newMqttPassword.isEmpty()) {
      newMqttPassword = legacyMqttPassword;
    } else if (!legacyMqttPassword.isEmpty() &&
               legacyMqttPassword != newMqttPassword) {
      logWarning(F("Settings POST contained both mqttPassword and legacy "
                   "mqttPass; applying mqttPassword."));
    }
    if (!newMqttPassword.isEmpty()) {
      cfg.mqttPassword = newMqttPassword;
    }
  }

  cfg.arduinoOtaEnabled = server.hasArg("arduinoOtaEnabled") &&
                          server.arg("arduinoOtaEnabled") == "1";
  cfg.debugSerial =
      server.hasArg("debugSerial") && server.arg("debugSerial") == "1";
  if (cfg.debugSerial) {
    Serial.println(F("[DEBUG] Settings manufacturer fields updated:"));
    for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
      Serial.print(F("[DEBUG] manufacturer_"));
      Serial.print(i);
      Serial.print(F("='"));
      Serial.print(cfg.devices[i].manufacturer);
      Serial.println(F("'"));
    }
  }

  logStatus(F("Settings updated from web UI."));
  if (!saveConfig()) {
    restartDevice(F("Failed to persist updated settings."));
  }
  applyRuntimeSettings();

  server.sendHeader("Location", "/settings");
  server.send(303);
}

void handleNotFound() { server.send(404, "text/plain", "Not found"); }

} // namespace vibrant
