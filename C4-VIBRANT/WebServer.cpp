#include "WebServer.h"
#include "Actions.h"
#include "BackupRestore.h"
#include "BootDiagnostics.h"
#include "Config.h"
#include "Debug.h"
#include "MqttClient.h"
#include "OtaService.h"
#include "Outputs.h"
#include "Reservations.h"
#include "Runtime.h"
#include "Stickserver.h"
#include "Version.h"
#include "WebAssets.h"
#include "WebUpdate.h"
#include "WifiManager.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <ESP8266WebServer.h>

namespace vibrant {

ESP8266WebServer server(80);

void registerWebRoutes() {
  logStatus(F("Registering web routes..."));
  server.on("/", HTTP_GET, handleHome);
  server.on("/partial", HTTP_GET, handleHomePartial);
  server.on("/toggle", HTTP_POST, handleToggle);
  server.on("/action", HTTP_POST, handleAction);
  server.on("/action/cancel", HTTP_POST, handleCancelAction);
  server.on("/action/status", HTTP_GET, handleActionStatus);
  server.on("/action/all-on", HTTP_POST, handleAllOn);
  server.on("/action/all-off", HTTP_POST, handleAllOff);
  server.on("/action/leave-mesh-all", HTTP_POST, handleLeaveMeshAll);
  server.on("/action/factory-reset-all", HTTP_POST, handleFactoryResetAll);
  server.on("/settings", HTTP_GET, handleSettingsGet);
  server.on("/settings/network", HTTP_GET, handleNetworkSettingsGet);
  server.on("/settings/network", HTTP_POST, handleNetworkSettingsPost);
  server.on("/settings/devices", HTTP_GET, handleDeviceSettingsGet);
  server.on("/settings/devices", HTTP_POST, handleDeviceSettingsPost);
  server.on("/settings/diagnostics", HTTP_GET, handleDiagnosticsGet);
  server.on("/settings/diagnostics/data", HTTP_GET, handleDiagnosticsData);
  server.on("/settings/diagnostics/heaplog", HTTP_GET, handleDiagnosticsHeapLog);
  server.on("/settings/diagnostics", HTTP_POST, handleDiagnosticsPost);
  server.on("/device/reboot", HTTP_POST, handleRebootDevice);
  server.on("/reservation/release", HTTP_POST, handleReleaseReservation);
  server.on("/reservation/reserve", HTTP_POST, handleGuiReserveOutput);
  server.on("/fleet", HTTP_GET, handleStickserverFleetGet);
  server.on("/fleet/data", HTTP_GET, handleStickserverFleetData);
  server.on("/fleet/toggle", HTTP_POST, handleFleetOutputToggle);
  server.on("/fleet/bulk", HTTP_POST, handleFleetBulkAction);
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

String htmlEscape(const String &value) {
  String out;
  out.reserve(value.length() + 16);
  for (size_t i = 0; i < value.length(); ++i) {
    char c = value[i];
    if (c == '&')
      out += F("&amp;");
    else if (c == '<')
      out += F("&lt;");
    else if (c == '>')
      out += F("&gt;");
    else if (c == '"')
      out += F("&quot;");
    else if (c == '\'')
      out += F("&#39;");
    else
      out += c;
  }
  return out;
}

// Escapes a string for safe embedding as a JSON string value (used when
// streaming JSON by hand instead of via ArduinoJson, e.g. /fleet/data).
String jsonEscape(const String &value) {
  String out;
  out.reserve(value.length() + 8);
  for (size_t i = 0; i < value.length(); ++i) {
    unsigned char c = (unsigned char)value[i];
    if (c == '"' || c == '\\') {
      out += '\\';
      out += (char)c;
    } else if (c == '\n')
      out += F("\\n");
    else if (c == '\r')
      out += F("\\r");
    else if (c == '\t')
      out += F("\\t");
    else if (c < 0x20) {
      char buf[8];
      snprintf(buf, sizeof(buf), "\\u%04x", c);
      out += buf;
    } else {
      out += (char)c;
    }
  }
  return out;
}

String formatUptimeHHMMSS(unsigned long totalSeconds) {
  unsigned long hours = totalSeconds / 3600UL;
  unsigned long minutes = (totalSeconds % 3600UL) / 60UL;
  unsigned long seconds = totalSeconds % 60UL;
  char buf[16];
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", hours, minutes, seconds);
  return String(buf);
}

// Unified identity header shown at the top of every page: the device's
// hostname (derived from its MAC address) plus live firmware version and
// uptime, centered above the page's content/table. This keeps the device's
// identity visible and consistent no matter which page is open, and ticks
// the uptime locally via JS so it stays live without extra server round
// trips.
static String pageHeaderHtml() {
  String html = F("<div class='page-header'><h1>");
  html += defaultHostnameFromMac(cfg.mac);
  html += F(" Output Control</h1><p>FW: ");
  html += SOFTWARE_VERSION;
  html += F(", Uptime: <span id='uptime-value'>");
  html += formatUptimeHHMMSS(millis() / 1000UL);
  html += F("</span></p></div><script>(function(){var s=");
  html += String(millis() / 1000UL);
  html += F(";function pad(n){return (n<10?'0':'')+n;}function "
            "fmt(t){var h=Math.floor(t/3600);var "
            "m=Math.floor((t%3600)/60);var sec=t%60;return "
            "pad(h)+':'+pad(m)+':'+pad(sec);}function tick(){var "
            "el=document.getElementById('uptime-value');if(el)"
            "el.textContent=fmt(s);s++;}tick();setInterval(tick,1000);})();"
            "</script>");
  return html;
}

// Forward declaration: the shared nav bar (with "Main output control" plus
// Network/Devices/Diagnostics/All Outputs) is defined further down near the
// settings page helpers, but handleHome() needs it too so every page shows
// the exact same navigation, including a highlighted "current" button for
// whichever page is active.
static String pageNavigation(const char *activePage);

namespace {
// Streaming helpers: callers build and send one small piece of HTML at a
// time (e.g. one table row) instead of accumulating a whole page in RAM, so
// each piece becomes its own independently-flushed HTTP chunk. Only one
// request is handled at a time by ESP8266WebServer, so this file-local state
// is safe.
bool gChunkedActive = false;
String gChunkedFallback;
const char *gChunkedContentType = "text/html";

// Writes exactly `len` bytes to `client`, retrying (with a short delay) as
// long as the socket is connected and progress is still possible. This is
// stronger than relying on ESP8266WebServer::sendContent(), which declares
// an HTTP chunk-size header up front and then simply logs (without
// resynchronizing or retrying) if the underlying write falls short under
// momentary WiFi/TCP congestion -- that mismatch between the declared and
// actually-sent byte count is what corrupted the chunked-transfer framing
// and produced the randomly missing/merged "All Outputs" table cells.
void writeFully(WiFiClient &client, const uint8_t *data, size_t len) {
  size_t written = 0;
  unsigned long lastProgressMs = millis();
  while (written < len && client.connected()) {
    size_t w = client.write(data + written, len - written);
    if (w > 0) {
      written += w;
      lastProgressMs = millis();
    } else if (millis() - lastProgressMs > 15000) {
      break; // genuinely stuck/disconnected; give up rather than hang forever
    } else {
      // Give ArduinoOTA a chance to process its UDP invitation/TCP transfer
      // even while a slow client stalls this write -- otherwise a page load
      // under backpressure could starve an in-flight OTA upload of CPU time
      // for seconds, which was making uploads fail and need 2-3 retries.
      if (arduinoOtaActive) {
        ArduinoOTA.handle();
      }
      delay(1);
    }
  }
}

void writeRaw(const char *data, size_t len) {
  if (len == 0) {
    return;
  }
  // Build the HTTP chunk ourselves (size header + CRLF + body + CRLF) and
  // write every part with writeFully(), so the declared chunk size always
  // matches what was actually delivered -- no partial/short writes can
  // desync the chunked-encoding stream the way sendContent() could.
  WiFiClient &client = server.client();
  char header[16];
  int headerLen = snprintf(header, sizeof(header), "%zx\r\n", len);
  writeFully(client, reinterpret_cast<const uint8_t *>(header), headerLen);
  writeFully(client, reinterpret_cast<const uint8_t *>(data), len);
  writeFully(client, reinterpret_cast<const uint8_t *>("\r\n"), 2);
}
} // namespace

// Starts a chunked response. Must be paired with one or more calls to
// writeChunk() followed by endChunkedHtml(). contentType defaults to
// "text/html"; pass "application/json" for streamed JSON endpoints (e.g.
// /fleet/data) so a large payload never needs to be held in RAM at once
// either.
void beginChunkedHtml(int code, const char *contentType = "text/html") {
  gChunkedFallback = "";
  gChunkedContentType = contentType;
  // ESP8266WebServer's default 1s write timeout is too tight for a busy page
  // with many sequential chunk writes (e.g. the "All Outputs" table); under
  // momentary congestion a write can time out short without retrying,
  // corrupting the chunked-encoding framing. Give writes more time, and
  // disable Nagle to reduce the chance of buffer backlog in the first place.
  server.client().setTimeout(8000);
  server.client().setNoDelay(true);
  // Without this, browsers can serve a stale cached copy of a GET page
  // (most importantly /partial and /fleet/data) instead of re-fetching, so
  // an action that changed server-side state (e.g. "Turn OFF all") silently
  // appears to do nothing on the next periodic refresh.
  server.sendHeader("Cache-Control", "no-store");
  gChunkedActive = server.chunkedResponseModeStart(code, contentType);
}

// Sends one piece of HTML immediately as its own independent chunk. If the
// client only supports HTTP/1.0 (chunked mode unavailable), the piece is
// buffered instead and sent as a single plain response by endChunkedHtml().
void writeChunk(const String &piece) {
  if (gChunkedActive) {
    writeRaw(piece.c_str(), piece.length());
  } else {
    gChunkedFallback += piece;
  }
}

void endChunkedHtml() {
  if (gChunkedActive) {
    server.chunkedResponseFinalize();
    gChunkedActive = false;
  } else {
    server.send(200, gChunkedContentType, gChunkedFallback);
    gChunkedFallback = "";
  }
}

// Sends an already-fully-built HTML string as a single chunked response.
// Prefer beginChunkedHtml() / writeChunk() / endChunkedHtml() for pages
// built incrementally (e.g. table rows), so the whole page never needs to
// exist in RAM at once.
void sendChunkedHtml(int code, const String &html) {
  beginChunkedHtml(code);
  writeChunk(html);
  endChunkedHtml();
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

// Renders only the dynamic, auto-refreshed part of the main page (the
// action-status banner, bulk-action buttons, and the output table) by
// writing chunks directly. Shared by the full-page handler (initial load)
// and the lightweight /partial endpoint that the page polls periodically so
// only this section needs to be refreshed, instead of the whole page.
static void renderHomeContent() {
  bool actionRunning = isActionRunning();

  // Action-status banner rendered server-side on every refresh (full page
  // load or partial poll) so it always reflects current state.
  String piece = "<div id='action-status'>";
  if (actionRunning) {
    String phaseDetail = isInCyclePhase()
                             ? String(F(" (cycles remaining: ")) +
                                   String(bgAction.cyclesRemaining) + ")"
                             : "";
    String target = bgAction.allOutputs
                        ? String(F("all outputs"))
                        : String(F("output ")) + String(bgAction.deviceIdx + 1);
    piece += "<div class='action-banner'><strong>Action running on " +
             target + ": " + actionPhaseName() + phaseDetail +
             "</strong>"
             " &nbsp; <form method='post' action='/action/cancel' "
             "style='display:inline;'>"
             "<button type='submit'>Cancel</button></form></div>";
  }
  piece += "</div>";

  // Global bulk-action buttons
  bool allOutputsReserved =
      managedOutputCount() > 0 && availableManagedOutputCount() == 0;
  const char *bulkDisabled =
      (actionRunning || allOutputsReserved) ? " disabled" : "";
  piece +=
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

  piece += F("<table><colgroup><col style='width:4%'><col style='width:16%'>"
             "<col style='width:16%'><col style='width:16%'><col "
             "style='width:10%'><col style='width:12%'><col "
             "style='width:26%'></colgroup><tr><th>#</th><th>DUT "
             "Manufacturer</th><th>DUT Model</th><th>DUT Name</"
             "th><th>Status</th><th>Reservation</th><th>Actions</th></tr>");
  writeChunk(piece);

  // Each output row is built and flushed independently so the whole table
  // never needs to be held in RAM at once.
  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    const DeviceEntry &d = cfg.devices[i];
    bool mapped = isValidOutputPin(d.pin);
    bool thisActionRunning = actionOwnsOutput(i);
    bool otherActionRunning = actionRunning && !thisActionRunning;
    bool reserved = outputReservations[i].reserved;

    piece = "<tr><td>" + String(i + 1) + "</td><td>" +
            htmlEscape(d.manufacturer) + "</td><td>" + htmlEscape(d.model) +
            "</td><td>" + htmlEscape(d.name) + "</td><td>";

    if (mapped) {
      const char *toggleDisabledAttr = reserved ? " disabled" : "";
      piece += "<form method='post' action='/toggle' style='margin:0;'>"
               "<input type='hidden' name='idx' value='" +
               String(i) +
               "'><input type='hidden' name='state' value='" +
               String(d.state ? 0 : 1) + "'><button type='submit' "
               "class='output-toggle " +
               String(d.state ? "output-on" : "output-off") + "'" +
               toggleDisabledAttr + " aria-label='Output " + String(i + 1) +
               " is " + String(d.state ? "ON" : "OFF") + "; turn " +
               String(d.state ? "off" : "on") + "' aria-pressed='" +
               String(d.state ? "true" : "false") + "'>" +
               String(d.state ? "ON" : "OFF") + "</button></form>";
    } else {
      piece += F("(none)");
    }

    piece += "</td><td>";
    if (mapped) {
      String reservationLabel;
      String reservationAction;
      const char *reservationButtonClass;
      if (reserved) {
        reservationLabel = outputReservations[i].owner.isEmpty()
                                ? String(F("Reserved"))
                                : htmlEscape(outputReservations[i].owner);
        reservationAction = F("/reservation/release");
        reservationButtonClass = "output-on";
      } else {
        // Not reserved by anything yet: show "n/a" and let the button
        // itself reserve this output for the GUI, which blocks any MQTT
        // "reserve" request from picking it (that code path already skips
        // outputs that are already reserved, regardless of owner).
        reservationLabel = F("n/a");
        reservationAction = F("/reservation/reserve");
        reservationButtonClass = "output-off";
      }
      piece += "<form method='post' action='" + reservationAction +
               "' style='margin:0;'><input type='hidden' name='idx' value='" +
               String(i) + "'><button type='submit' class='output-toggle " +
               String(reservationButtonClass) + "'>" + reservationLabel +
               "</button></form>";
    } else {
      piece += F("(none)");
    }

    piece += "</td><td>";
    if (mapped) {
      if (thisActionRunning) {
        piece += F("<em>Running...</em>");
      } else {
        const char *disabledAttr =
            (otherActionRunning || reserved) ? " disabled" : "";
        piece += "<form method='post' action='/action' "
                 "style='display:inline;margin:0;'>"
                 "<input type='hidden' name='idx' value='" +
                 String(i) +
                 "'>"
                 "<input type='hidden' name='cmd' value='leave_mesh'>"
                 "<button type='submit' class='output-toggle'" +
                 disabledAttr +
                 ">Leave Mesh</button></form>"
                 "<form method='post' action='/action' "
                 "style='display:inline;margin:0;'>"
                 "<input type='hidden' name='idx' value='" +
                 String(i) +
                 "'>"
                 "<input type='hidden' name='cmd' value='factory_reset'>"
                 "<button type='submit' class='output-toggle'" +
                 disabledAttr + ">Factory Reset</button></form>";
      }
    } else {
      piece += F("(none)");
    }
    piece += "</td></tr>";
    writeChunk(piece);
  }

  writeChunk(F("</table>"));
}

void handleHome() {
  beginChunkedHtml(200);

  String piece = FPSTR(HOME_PAGE_HEADER);
  piece += pageHeaderHtml();
  piece += pageNavigation("home");
  if (usingFactoryPassword()) {
    piece += passwordWarningHtml();
  }
  // This page is streamed as many small HTTP chunks; on a slow/weak WiFi
  // link the browser can render a table with table-layout:fixed
  // progressively as chunks arrive, so a user looking at it mid-load would
  // see rows/cells that simply haven't arrived yet -- which looks exactly
  // like missing/corrupted data but is actually just an incomplete page
  // load. Keep the content hidden behind a loading message until the whole
  // thing has arrived, then reveal it with a trailing inline script so the
  // user only ever sees the complete content. This only matters for this
  // direct, streamed initial load -- the periodic /partial refresh below is
  // fetched in full by the browser before it is ever shown, so it doesn't
  // need the same treatment.
  piece += F("<p id='home-loading'>Loading output status&hellip;</p>"
             "<div id='home-content' style='display:none'>");
  writeChunk(piece);
  renderHomeContent();
  writeChunk(F(
      "</div>"
      "<script>"
      "document.getElementById('home-loading').style.display='none';"
      "document.getElementById('home-content').style.display='';"
      // Guarded against overlapping fetches: on a slow/weak link a refresh
      // can still be in flight when the next interval tick fires, and a
      // second concurrent connection opened mid-transfer has been observed
      // to crash the device (extra connection memory on top of an
      // already-large in-progress response). This single flag now guards
      // BOTH the periodic /partial refresh AND button-submitted actions
      // (previously only refresh-vs-refresh was guarded; an action POST
      // could still fire while a refresh GET was in flight, opening a
      // second simultaneous connection that could crash the device --
      // seen in practice as the browser's fetch() aborting mid-request
      // with 'TypeError: Failed to fetch').
      "var homeRequestInFlight=false;"
      "function refreshHomeContent(){"
      "if(homeRequestInFlight)return;"
      "homeRequestInFlight=true;"
      "fetch('/partial').then(function(r){return r.text();})"
      ".then(function(html){"
      "document.getElementById('home-content').innerHTML=html;"
      "}).catch(function(){})"
      ".then(function(){homeRequestInFlight=false;});"
      "}"
      "setInterval(refreshHomeContent,3000);"
      // Buttons are re-created on every refresh, so submits are intercepted
      // via delegation on a stable ancestor rather than binding to the
      // buttons themselves. Submitting via fetch (instead of a normal
      // navigation) lets the page stay in place and schedule a refresh 1
      // second later, giving the action/MQTT command time to take effect
      // before the section re-fetches. If an inline onsubmit confirm()
      // dialog already cancelled the submission (e.g. bulk Leave Mesh/
      // Factory Reset), defaultPrevented is already true and this is
      // skipped.
      "document.getElementById('home-content').addEventListener('submit',"
      "function(e){"
      "if(e.defaultPrevented)return;"
      "e.preventDefault();"
      "if(homeRequestInFlight){"
      "alert('Busy refreshing, please try again in a moment.');return;}"
      "homeRequestInFlight=true;"
      "fetch(e.target.action,{method:'POST',body:new FormData(e.target)})"
      // fetch() only rejects on a network failure; an HTTP error status
      // (e.g. 409 when another action is already running, or 400 for a
      // bad request) still resolves normally and was previously swallowed
      // here, so a rejected bulk/per-output action silently appeared to do
      // nothing. Surface it instead.
      ".then(function(r){if(!r.ok){r.text().then(function(t){"
      "alert('Action failed (HTTP '+r.status+'): '+(t||r.statusText));"
      "});}})"
      ".catch(function(err){alert('Action failed: '+err);})"
      ".then(function(){homeRequestInFlight=false;"
      "setTimeout(refreshHomeContent,1000);});"
      "});"
      "</script></body></html>"));
  endChunkedHtml();
}

// Lightweight endpoint returning only the dynamic action-status/output-table
// HTML fragment (no page chrome), polled periodically by the main page's
// inline script so only that section needs to refresh, not the whole page.
// Matches the main page's own access level (no auth required to view).
void handleHomePartial() {
  beginChunkedHtml(200);
  renderHomeContent();
  endChunkedHtml();
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
  maybePersistOutputState();
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
  maybePersistOutputState();
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
  maybePersistOutputState();
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

static String pageNavigation(const char *activePage) {
  String html = F("<nav class='settings-nav' aria-label='Page navigation'>");
  const char *paths[] = {"/", "/settings/network", "/settings/devices",
                         "/settings/diagnostics", "/fleet"};
  const char *labels[] = {"Main output control", "Network, Wi-Fi & MQTT",
                          "Devices & outputs", "Diagnostics & OTA",
                          "All Outputs"};
  const char *pages[] = {"home", "network", "devices", "diagnostics",
                         "fleet"};
  for (uint8_t i = 0; i < 5; ++i) {
    html += "<a class='nav-btn";
    if (String(activePage) == pages[i])
      html += " current";
    html += "' href='" + String(paths[i]) + "'";
    if (String(activePage) == pages[i])
      html += " aria-current='page'";
    html += ">" + String(labels[i]) + "</a>";
  }
  html += F("</nav>");
  return html;
}

static String settingsPageStart(const char *activePage) {
  String html = FPSTR(SETTINGS_PAGE_HEADER);
  html += "</head><body>";
  html += pageHeaderHtml();
  html += pageNavigation(activePage);
  if (usingFactoryPassword())
    html += passwordWarningHtml();
  return html;
}

static void finishSettingsSave(const char *logMessage, const char *redirect) {
  logStatus(logMessage);
  if (!saveConfig())
    restartDevice(F("Failed to persist updated settings."));
  applyRuntimeSettings();
  server.sendHeader("Location", redirect);
  server.send(303);
}

static void logSettingsDebug(const char *pageName) {
  if (cfg.debugSerial) {
    Serial.print(F("[DEBUG] Settings page saved: "));
    Serial.println(pageName);
  }
}

void handleSettingsGet() {
  if (!ensureAuthorized())
    return;
  // Streamed in independent pieces (rather than one large String built in
  // full before sending) to keep peak heap usage low on this memory
  // constrained device; see beginChunkedHtml() for why that matters.
  beginChunkedHtml(200);
  writeChunk(settingsPageStart(""));
  writeChunk(F("<p class='page-intro'>Choose a settings page to configure the "
               "network, outputs, or diagnostics and firmware updates.</p>"
               "<ul><li><a href='/settings/network'>Network, Wi-Fi &amp; "
               "MQTT</a></li><li><a href='/settings/devices'>Devices &amp; "
               "outputs</a></li><li><a href='/settings/diagnostics'>Diagnostics "
               "&amp; OTA</a></li></ul></body></html>"));
  endChunkedHtml();
}

void handleNetworkSettingsGet() {
  if (!ensureAuthorized())
    return;
  // Streamed as several independent pieces (rather than one large String
  // built in full before sending) to keep peak heap usage low on this
  // memory constrained device; see beginChunkedHtml() for why that matters.
  beginChunkedHtml(200);
  writeChunk(settingsPageStart("network"));
  writeChunk(
      "<p class='page-intro'>Network and broker changes are saved without "
      "altering device or diagnostics settings.</p>"
      "<form method='post' action='/settings/network'>"
      "<fieldset><legend>Station network</legend>"
      "<label>Station SSID <input name='staSsid' value='" +
      htmlEscape(cfg.staSsid) +
      "'></label>"
      "<label for='staPassword'>Station password</label><input "
      "id='staPassword' name='staPassword' type='password' value='' "
      "placeholder='Leave empty to keep current station password'>"
      "</fieldset>");
  writeChunk(
      "<fieldset><legend>Access point</legend>"
      "<label>SoftAP SSID <input value='" +
      htmlEscape(defaultSoftApSsidFromMac(cfg.mac)) +
      "' readonly></label>"
      "<label for='apPassword'>SoftAP password</label><input "
      "id='apPassword' name='apPassword' type='password' value='' "
      "placeholder='Leave empty to keep current AP password'>"
      "</fieldset>"
      "<fieldset><legend>Network device settings</legend>"
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
      "'></label></fieldset>");
  writeChunk(
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
      "value='';\"> Clear MQTT password (remove broker authentication)</label>");
  writeChunk(
      "<label><input type='checkbox' name='stickserverRespondEnabled' "
      "value='1'" +
      String(cfg.stickserverRespondEnabled ? " checked" : "") +
      "> Respond to stickserver hello/list requests (lets peers discover "
      "and control this device)</label>"
      "<label><input type='checkbox' name='stickserverQueryEnabled' "
      "value='1'" +
      String(cfg.stickserverQueryEnabled ? " checked" : "") +
      "> Query for stickservers (broadcast hello/list requests to actively "
      "discover other stickservers and their outputs)</label>"
      "<label><input type='checkbox' "
      "name='stickserverPassiveDiscoveryEnabled' value='1'" +
      String(cfg.stickserverPassiveDiscoveryEnabled ? " checked" : "") +
      "> Passively parse hello/list responses observed on the bus "
      "(required for the All Outputs page)</label>"
      "<label><input type='checkbox' name='restoreOutputStateOnBoot' "
      "value='1'" +
      String(cfg.restoreOutputStateOnBoot ? " checked" : "") +
      "> Restore output ON/OFF state after boot/reboot (otherwise all "
      "outputs always start OFF)</label>"
      "<label><input type='checkbox' name='restoreReservationsOnBoot' "
      "value='1'" +
      String(cfg.restoreReservationsOnBoot ? " checked" : "") +
      "> Restore output reservations after boot/reboot (otherwise all "
      "reservations are cleared)</label>");
  writeChunk(
      "<p style='font-size:0.9em;color:#555;'>Device control is handled "
      "exclusively via the Stickserver protocol, which subscribes to "
      "<code>" +
      htmlEscape(String(STICKSERVER_ROOT_TOPIC)) +
      "</code> and replies on the device topic (hello / list / reserve / "
      "release / status / join / leave / power_on / power_off / "
      "factory_reset / reboot).</p></fieldset>"
      "<button type='submit'>Save network settings</button></form>"
      "</body></html>");
  endChunkedHtml();
}

void handleNetworkSettingsPost() {
  if (!ensureAuthorized())
    return;
  if (!server.hasArg("staSsid") || !server.hasArg("mac") ||
      !server.hasArg("hostname") || !server.hasArg("wifiPower") ||
      !server.hasArg("mqttHost") || !server.hasArg("mqttPort") ||
      !server.hasArg("mqttUser")) {
    logError(F("Network settings save rejected: required field missing."));
    server.send(400, "text/plain", "Missing required network setting");
    return;
  }

  String macValue = server.arg("mac");
  macValue.toUpperCase();
  uint8_t macBytes[6] = {0};
  if (!parseMac(macValue, macBytes)) {
    logError(F("Network settings save rejected due to invalid MAC address."));
    server.send(400, "text/plain",
                "Invalid MAC address format. Use AA:BB:CC:DD:EE:FF");
    return;
  }
  String hostname = server.arg("hostname");
  if (hostname.length() > 32) {
    logError(F("Network settings save rejected due to invalid hostname."));
    server.send(400, "text/plain",
                "Hostname must be at most 32 characters (letters, digits, "
                "hyphens only)");
    return;
  }
  for (size_t i = 0; i < hostname.length(); ++i) {
    char c = hostname[i];
    if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
          (c >= '0' && c <= '9') || c == '-')) {
      logError(F("Network settings save rejected due to invalid hostname."));
      server.send(400, "text/plain",
                  "Hostname must contain only letters, digits, and hyphens");
      return;
    }
  }
  float parsedPower = 0.0f;
  if (!parseFloatValue(server.arg("wifiPower"), parsedPower)) {
    logError(F("Network settings save rejected due to invalid Wi-Fi power."));
    server.send(400, "text/plain", "Invalid Wi-Fi power value");
    return;
  }
  String stationSsid = server.arg("staSsid");
  String stationPassword = cfg.staPassword;
  if (!server.arg("staPassword").isEmpty())
    stationPassword = server.arg("staPassword");
  String apPassword = cfg.apPassword;
  if (!server.arg("apPassword").isEmpty())
    apPassword = server.arg("apPassword");
  if (stationSsid.isEmpty() || stationPassword.isEmpty() ||
      apPassword.isEmpty()) {
    logError(F("Network settings save rejected because required credentials "
               "were empty."));
    server.send(400, "text/plain",
                "Station SSID, station password, and AP password must not be "
                "empty");
    return;
  }

  cfg.mac = macValue;
  cfg.useCustomMac =
      server.hasArg("useCustomMac") && server.arg("useCustomMac") == "1";
  cfg.hostname =
      hostname.isEmpty() ? defaultHostnameFromMac(cfg.mac) : hostname;
  cfg.staSsid = stationSsid;
  cfg.staPassword = stationPassword;
  cfg.apPassword = apPassword;
  cfg.wifiPower = constrain(parsedPower, MIN_WIFI_POWER, MAX_WIFI_POWER);
  cfg.mqttEnabled =
      server.hasArg("mqttEnabled") && server.arg("mqttEnabled") == "1";
  cfg.mqttHost = server.arg("mqttHost");
  int parsedPort = -1;
  if (parseIndexValue(server.arg("mqttPort"), parsedPort) && parsedPort >= 1 &&
      parsedPort <= 65535) {
    cfg.mqttPort = static_cast<uint16_t>(parsedPort);
  } else {
    cfg.mqttPort = DEFAULT_MQTT_PORT;
  }
  cfg.mqttUser = server.arg("mqttUser");
  if (server.hasArg("mqttPasswordClear") &&
      server.arg("mqttPasswordClear") == "1") {
    cfg.mqttPassword = "";
  } else {
    String newMqttPassword = server.arg("mqttPassword");
    String legacyMqttPassword = server.arg("mqttPass");
    if (newMqttPassword.isEmpty())
      newMqttPassword = legacyMqttPassword;
    else if (!legacyMqttPassword.isEmpty() &&
             legacyMqttPassword != newMqttPassword)
      logWarning(F("Network settings POST contained conflicting MQTT "
                   "password fields; applying mqttPassword."));
    if (!newMqttPassword.isEmpty())
      cfg.mqttPassword = newMqttPassword;
  }
  cfg.stickserverRespondEnabled =
      server.hasArg("stickserverRespondEnabled") &&
      server.arg("stickserverRespondEnabled") == "1";
  cfg.stickserverQueryEnabled =
      server.hasArg("stickserverQueryEnabled") &&
      server.arg("stickserverQueryEnabled") == "1";
  cfg.stickserverPassiveDiscoveryEnabled =
      server.hasArg("stickserverPassiveDiscoveryEnabled") &&
      server.arg("stickserverPassiveDiscoveryEnabled") == "1";
  cfg.restoreOutputStateOnBoot =
      server.hasArg("restoreOutputStateOnBoot") &&
      server.arg("restoreOutputStateOnBoot") == "1";
  cfg.restoreReservationsOnBoot =
      server.hasArg("restoreReservationsOnBoot") &&
      server.arg("restoreReservationsOnBoot") == "1";

  logSettingsDebug("network");
  finishSettingsSave("Network settings updated from web UI.",
                     "/settings/network");
}

void handleDeviceSettingsGet() {
  if (!ensureAuthorized())
    return;
  beginChunkedHtml(200);
  String piece = settingsPageStart("devices");
  piece += FPSTR(SETTINGS_PAGE_SCRIPT);
  piece += "<p>Each active output must use a different GPIO. TX/RX disable "
           "serial communication; GPIO0 (FLASH) and GPIO15 affect boot.</p>"
           "<form id='device-form' method='post' action='/settings/devices'>"
           "<label>Number of outputs (1 - 16) <input name='numOutputs' "
           "type='number' min='1' max='16' step='1' value='" +
           String(cfg.numOutputs) +
           "'></label>"
           "<div class='bulk-actions'><button type='button' "
           "onclick='copyFirstManufacturerToAll()'>Use first Manufacturer for "
           "all</button>"
           "<button type='button' onclick='copyFirstModelToAll()'>Use first "
           "Model for all</button>"
           "<button type='button' onclick='copyFirstNameToAll()'>Use first "
           "Name for all</button>"
           "<button type='button' onclick='clearAllFieldsExceptOutput()'>"
           "Clear all fields</button>"
           "<button type='button' onclick='reverseGpioAssignments()'>Reverse "
           "GPIO assignments</button>"
           "<span>If the first Name contains #5, copy keeps the first row at "
           "#5 and fills later rows as #6, #7, and so on.</span></div>"
           "<table><tr><th>#</th><th>DUT Manufacturer</th><th>DUT Model</"
           "th><th>DUT Name</th><th>Control output</th></tr>";
  writeChunk(piece);

  // Each output row is flushed independently so the whole form table never
  // needs to be held in RAM at once.
  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    piece = "<tr><td>" + String(i + 1) +
            "</td><td><input name='manufacturer_" + String(i) + "' value='" +
            htmlEscape(cfg.devices[i].manufacturer) +
            "'></td><td><input name='model_" + String(i) + "' value='" +
            htmlEscape(cfg.devices[i].model) +
            "'></td><td><input name='name_" + String(i) + "' value='" +
            htmlEscape(cfg.devices[i].name) +
            "'></td><td><select name='pin_" + String(i) + "'>";
    piece += (cfg.devices[i].pin < 0)
                 ? "<option value='-1' selected>none</option>"
                 : "<option value='-1'>none</option>";
    for (size_t pinIndex = 0; pinIndex < OUTPUT_PIN_MAPPING_COUNT; ++pinIndex)
      piece += pinOption(cfg.devices[i].pin, OUTPUT_PIN_MAPPINGS[pinIndex]);
    piece += F("</select></td></tr>");
    writeChunk(piece);
  }
  writeChunk(F("</table><button type='submit'>Save device settings</button>"
               "</form></body></html>"));
  endChunkedHtml();
}

void handleDeviceSettingsPost() {
  if (!ensureAuthorized())
    return;
  int parsedOutputs = -1;
  if (!server.hasArg("numOutputs") ||
      !parseIndexValue(server.arg("numOutputs"), parsedOutputs) ||
      parsedOutputs < 1 || parsedOutputs > MAX_DEVICES) {
    logError(F("Device settings save rejected: output count must be 1-16."));
    server.send(400, "text/plain", "Number of outputs must be 1-16");
    return;
  }
  bool usedPins[17] = {false};
  for (int i = 0; i < parsedOutputs; ++i) {
    String pinArgName = "pin_" + String(i);
    if (!server.hasArg(pinArgName)) {
      if (i < cfg.numOutputs) {
        logError(F("Device settings save rejected: output field missing."));
        server.send(400, "text/plain", "Missing device output setting");
        return;
      }
      continue;
    }
    int pin = -1;
    if (!parsePinValue(server.arg(pinArgName), pin)) {
      logError(F("Device settings save rejected: invalid output pin."));
      server.send(400, "text/plain", "Invalid output pin");
      return;
    }
    if (pin >= 0) {
      if (usedPins[pin]) {
        logError(F("Device settings save rejected: duplicate GPIO."));
        server.send(400, "text/plain",
                    "Duplicate GPIO assignment among active outputs");
        return;
      }
      usedPins[pin] = true;
    }
  }
  for (int i = 0; i < parsedOutputs; ++i) {
    String index = String(i);
    if (i < cfg.numOutputs &&
        (!server.hasArg("manufacturer_" + index) ||
         !server.hasArg("model_" + index) || !server.hasArg("name_" + index))) {
      logError(F("Device settings save rejected: device field missing."));
      server.send(400, "text/plain", "Missing device setting");
      return;
    }
  }

  cfg.numOutputs = static_cast<uint8_t>(parsedOutputs);
  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    String index = String(i);
    cfg.devices[i].manufacturer =
        server.hasArg("manufacturer_" + index)
            ? server.arg("manufacturer_" + index)
            : String(DEFAULT_DEVICE_MANUFACTURER);
    cfg.devices[i].model = server.hasArg("model_" + index)
                               ? server.arg("model_" + index)
                               : String(DEFAULT_DEVICE_MODEL);
    String manufacturer = cfg.devices[i].manufacturer;
    String model = cfg.devices[i].model;
    manufacturer.trim();
    model.trim();
    if (manufacturer.isEmpty())
      cfg.devices[i].manufacturer = DEFAULT_DEVICE_MANUFACTURER;
    if (model.isEmpty())
      cfg.devices[i].model = DEFAULT_DEVICE_MODEL;
    cfg.devices[i].name =
        server.hasArg("name_" + index) ? server.arg("name_" + index) : "";
    int pin = -1;
    String pinArgName = "pin_" + index;
    if (server.hasArg(pinArgName))
      parsePinValue(server.arg(pinArgName), pin);
    cfg.devices[i].pin = pin;
    if (!isValidOutputPin(cfg.devices[i].pin))
      cfg.devices[i].state = false;
  }
  logSettingsDebug("devices");
  finishSettingsSave("Device settings updated from web UI.",
                     "/settings/devices");
}

void handleDiagnosticsGet() {
  if (!ensureAuthorized())
    return;
  // Streamed as several independent pieces (rather than one large String
  // built in full before sending) to keep peak heap usage low on this
  // memory constrained device; see beginChunkedHtml() for why that matters.
  beginChunkedHtml(200);
  writeChunk(settingsPageStart("diagnostics"));
  writeChunk(
      "<p class='page-intro'>Control verbose serial diagnostics and update "
      "the firmware.</p><form method='post' "
      "action='/settings/diagnostics'><fieldset><legend>Diagnostics and "
      "ArduinoOTA</legend>"
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
      "> Enable verbose serial debug logging</label></fieldset>"
      "<button type='submit'>Save diagnostics settings</button></form>");
  writeChunk(
      "<h2>Device control</h2>"
      "<p>Current uptime: <span id='diag-uptime'>" +
      formatUptimeHHMMSS(millis() / 1000UL) +
      "</span></p>"
      "<p>Free heap: <span id='diag-heap'>" + String(ESP.getFreeHeap()) +
      " bytes (fragmentation: " + String(ESP.getHeapFragmentation()) +
      "%, largest free block: " + String(ESP.getMaxFreeBlockSize()) +
      " bytes)</span></p>"
      "<form method='post' action='/device/reboot' "
      "onsubmit=\"return confirm('Reboot the device now?');\">"
      "<button type='submit'>Reboot device</button></form>"
      "<script>(function(){function refresh(){"
      "fetch('/settings/diagnostics/data').then(function(r){return "
      "r.json();}).then(function(d){"
      "var u=document.getElementById('diag-uptime');if(u)u.textContent=d."
      "uptime;"
      "var h=document.getElementById('diag-heap');if(h)h.textContent=d."
      "freeHeap+' bytes (fragmentation: '+d.heapFragmentation+'%, largest "
      "free block: '+d.maxFreeBlockSize+' bytes)';"
      "}).catch(function(){});}"
      "refresh();setInterval(refresh,5000);})();</script>");
  writeChunk(
      "<h2>Configuration maintenance</h2>"
      "<p><a href='/config/export'>Download configuration backup</a></p>"
      "<p>Hold the FLASH button during power-on (during the first " +
      String(FLASH_BOOT_DETECTION_WINDOW_MS) +
      " milliseconds of boot) to trigger factory reset and restart.</p>"
      "<form method='post' action='/config/factory-reset' "
      "onsubmit=\"return confirm('Factory reset?');\">"
      "<button type='submit'>Factory reset</button></form>"
      "<form method='post' action='/config/import' "
      "enctype='multipart/form-data'>"
      "<label>Import backup JSON <input type='file' name='config' "
      "accept='application/json' required></label>"
      "<button type='submit'>Upload and restore</button></form>");
  writeChunk(F("<h2>Firmware update</h2>"
               "<p>Upload a compiled <code>.bin</code> to update firmware over "
               "the network. The device reboots automatically after a "
               "successful flash.</p><p><a href='/firmware/update'>Open "
               "firmware update page</a></p></body></html>"));
  endChunkedHtml();
}

// Lightweight REST endpoint returning the diagnostics page's live values
// (uptime, heap, OTA/debug flags) as JSON, so the diagnostics page can poll
// and refresh just these values in place instead of re-fetching/re-parsing
// the whole HTML page.
void handleDiagnosticsData() {
  if (!ensureAuthorized())
    return;
  String json = "{\"version\":\"" + jsonEscape(String(SOFTWARE_VERSION)) +
                "\",\"hostname\":\"" + jsonEscape(cfg.hostname) +
                "\",\"uptimeSeconds\":" + String(millis() / 1000UL) +
                ",\"uptime\":\"" + formatUptimeHHMMSS(millis() / 1000UL) +
                "\",\"freeHeap\":" + String(ESP.getFreeHeap()) +
                ",\"heapFragmentation\":" + String(ESP.getHeapFragmentation()) +
                ",\"maxFreeBlockSize\":" + String(ESP.getMaxFreeBlockSize()) +
                ",\"arduinoOtaEnabled\":" +
                String(cfg.arduinoOtaEnabled ? "true" : "false") +
                ",\"debugSerial\":" +
                String(cfg.debugSerial ? "true" : "false") + "}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

// Exposes the fixed-size heap-fragmentation trace (see
// handleStickserverMessage()/recordHeapTraceBase() in Stickserver.cpp) over
// HTTP, so it can be inspected without a physical serial connection. Only
// populated while cfg.debugSerial is enabled.
void handleDiagnosticsHeapLog() {
  if (!ensureAuthorized())
    return;
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", renderHeapTraceJson());
}

void handleDiagnosticsPost() {
  if (!ensureAuthorized())
    return;
  cfg.arduinoOtaEnabled = server.hasArg("arduinoOtaEnabled") &&
                          server.arg("arduinoOtaEnabled") == "1";
  cfg.debugSerial =
      server.hasArg("debugSerial") && server.arg("debugSerial") == "1";
  logSettingsDebug("diagnostics");
  finishSettingsSave("Diagnostics and OTA settings updated from web UI.",
                     "/settings/diagnostics");
}

void handleReleaseReservation() {
  if (!ensureAuthorized())
    return;
  int idx = -1;
  if (!server.hasArg("idx") || !parseIndexValue(server.arg("idx"), idx) ||
      idx < 0 || idx >= cfg.numOutputs ||
      !isValidOutputPin(cfg.devices[idx].pin)) {
    server.send(400, "text/plain", "Invalid output index");
    return;
  }
  logStatus(String(F("Releasing reservation for output ")) +
            String(idx + 1) + F(" via web UI."));
  JsonDocument req;
  req["cmd"] = "release";
  req["ver"] = STICKSERVER_PROTOCOL_VERSION;
  req["mid"] = String(F("web-release-")) + String(millis());
  JsonArray euids = req["euids"].to<JsonArray>();
  euids.add(stickserverOutputEuid(static_cast<uint8_t>(idx)));
  String payload;
  serializeJson(req, payload);
  // Processed in-process (not round-tripped through the broker) so the
  // release takes effect immediately; the resulting response is still
  // published over MQTT like any other stickserver command.
  handleStickserverMessage(stickserverInstanceTopic(), payload);
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleGuiReserveOutput() {
  if (!ensureAuthorized())
    return;
  int idx = -1;
  if (!server.hasArg("idx") || !parseIndexValue(server.arg("idx"), idx) ||
      idx < 0 || idx >= cfg.numOutputs ||
      !isValidOutputPin(cfg.devices[idx].pin)) {
    server.send(400, "text/plain", "Invalid output index");
    return;
  }
  // Unlike handleReleaseReservation(), this is a local, GUI-only
  // reservation: it is not routed through the stickserver protocol since
  // that command picks any output matching a model/count request rather
  // than this specific one. Setting outputReservations[idx] directly here
  // is enough to block MQTT "reserve" requests from ever selecting this
  // output, since that code path already skips any output that is already
  // reserved -- regardless of owner.
  if (!outputReservations[idx].reserved) {
    outputReservations[idx].reserved = true;
    outputReservations[idx].owner = "GUI";
    logStatus(String(F("Reserved output ")) + String(idx + 1) +
              F(" for the GUI via web UI (blocks MQTT reservation)."));
    maybePersistReservations();
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleRebootDevice() {
  if (!ensureAuthorized())
    return;
  logStatus(F("Device reboot requested from web UI."));
  server.send(200, "text/html",
              "<!doctype html><html><head><meta charset='utf-8'><title>"
              "Rebooting</title></head><body><p>Rebooting device...</p>"
              "<script>setTimeout(function(){window.location='/';},8000);"
              "</script></body></html>");
  server.client().flush();
  delay(200);
  ESP.restart();
}

// Streams the full "All Outputs" state as JSON: which stickservers are
// discovered, their outputs, and each output's current on/off state. The
// /fleet page is a static shell (built once) whose JS fetches this endpoint
// on load and on every periodic refresh, building/rebuilding the output
// table entirely client-side from this data -- this replaced an earlier
// design where the ESP8266 itself re-rendered the whole HTML table (with a
// <form> per output button) on every refresh, which was repeatedly
// implicated in reboots under heap/WiFi pressure on large tables. Shipping
// plain data instead of markup means the device does much less work per
// refresh, and the browser (not the ESP8266) is responsible for
// consistently redrawing the table every time.
static void renderFleetDataJson() {
  if (!cfg.mqttEnabled || !mqttClient.connected()) {
    writeChunk(F("{\"ok\":false,\"reason\":\"mqtt\"}"));
    return;
  }
  if (!cfg.stickserverQueryEnabled && !cfg.stickserverPassiveDiscoveryEnabled) {
    writeChunk(F("{\"ok\":false,\"reason\":\"disabled\"}"));
    return;
  }

  // Collect active discovered servers (this device's own instance is
  // discovered the same way as any other, via its own hello/list replies).
  uint8_t activeIdx[MAX_DISCOVERED_SERVERS];
  uint8_t activeCount = 0;
  for (uint8_t i = 0; i < MAX_DISCOVERED_SERVERS; ++i) {
    if (!discoveredServers[i].active)
      continue;
    activeIdx[activeCount++] = i;
  }

  if (activeCount == 0) {
    writeChunk(F("{\"ok\":true,\"servers\":[]}"));
    return;
  }

  // Sort columns by ascending IP address (numeric, not lexicographic, so
  // e.g. .9 sorts before .10). Servers with no known IP (third-party
  // stickservers that don't report one) sort after all known-IP servers,
  // falling back to hostname order among themselves.
  for (uint8_t a = 0; a + 1 < activeCount; ++a) {
    for (uint8_t b = 0; b + 1 < activeCount - a; ++b) {
      const DiscoveredServer &sa = discoveredServers[activeIdx[b]];
      const DiscoveredServer &sb = discoveredServers[activeIdx[b + 1]];
      IPAddress ipa, ipb;
      bool haveA = ipa.fromString(sa.ipAddress);
      bool haveB = ipb.fromString(sb.ipAddress);
      bool swapNeeded = false;
      if (haveA && haveB) {
        uint32_t keyA = ((uint32_t)ipa[0] << 24) | ((uint32_t)ipa[1] << 16) |
                        ((uint32_t)ipa[2] << 8) | ipa[3];
        uint32_t keyB = ((uint32_t)ipb[0] << 24) | ((uint32_t)ipb[1] << 16) |
                        ((uint32_t)ipb[2] << 8) | ipb[3];
        swapNeeded = keyA > keyB;
      } else if (haveA != haveB) {
        swapNeeded = !haveA; // known IP sorts before unknown IP
      } else {
        // Neither has a known IP: fall back to hostname ordering.
        String labelA = sa.hostname.isEmpty() ? sa.instanceTopic : sa.hostname;
        String labelB = sb.hostname.isEmpty() ? sb.instanceTopic : sb.hostname;
        swapNeeded = labelA > labelB;
      }
      if (swapNeeded) {
        uint8_t tmp = activeIdx[b];
        activeIdx[b] = activeIdx[b + 1];
        activeIdx[b + 1] = tmp;
      }
    }
  }

  writeChunk(F("{\"ok\":true,\"servers\":["));
  for (uint8_t c = 0; c < activeCount; ++c) {
    const DiscoveredServer &s = discoveredServers[activeIdx[c]];
    String label = s.hostname.isEmpty() ? s.instanceTopic : s.hostname;
    // Hostnames follow the "C4-VIBRANT-<MAC suffix>" convention; strip the
    // common prefix so the column header shows just the distinguishing MAC
    // suffix. The instanceTopic fallback is a different format and is left
    // untouched.
    if (!s.hostname.isEmpty() && label.startsWith(F("C4-VIBRANT-")))
      label.remove(0, 11);

    String piece = c == 0 ? "{" : ",{";
    piece.reserve(300);
    piece += "\"label\":\"" + jsonEscape(label) + "\",\"ip\":\"" +
             jsonEscape(s.ipAddress) + "\",\"topic\":\"" +
             jsonEscape(s.instanceTopic) + "\",\"outputs\":[";
    writeChunk(piece);

    for (uint8_t row = 0; row < s.outputCount; ++row) {
      const DiscoveredOutputEntry &o = s.outputs[row];
      if (!o.valid)
        continue;
      String label2 = o.name.isEmpty() ? String(F("Output ")) + String(row + 1)
                                        : o.name;
      piece = row == 0 ? "{" : ",{";
      piece.reserve(200);
      piece += "\"euid\":\"" + jsonEscape(o.euid) + "\",\"name\":\"" +
               jsonEscape(label2) + "\",\"on\":" +
               String(o.state ? "true" : "false") + "}";
      writeChunk(piece);
      // Feed the watchdog and let the WiFi/TCP stack run between outputs.
      // With many discovered stickservers (each with up to MAX_DEVICES
      // outputs) this can be hundreds of entries, each its own blocking
      // network write; without yielding here a slow/congested link can
      // starve background WiFi servicing long enough to trip the watchdog
      // and reboot the device mid-response.
      yield();
    }
    writeChunk(F("]}"));
  }
  writeChunk(F("]}"));
}

void handleStickserverFleetGet() {
  if (!ensureAuthorized())
    return;
  String piece = settingsPageStart("fleet");
  piece += F(
      "<p class='page-intro'>Discovered stickserver instances and their "
      "outputs, gathered passively over MQTT (hello/list). Each column is "
      "one stickserver; each row is one output slot. Buttons reflect live "
      "MQTT state: gray = off, yellow = on. Click a button to toggle that "
      "output over MQTT. This section refreshes itself automatically.</p>"
      "<div class='bulk-actions'>"
      "<button type='button' data-cmd='power_on'>Turn On</button>"
      "<button type='button' data-cmd='power_off'>Turn Off</button>"
      "<button type='button' data-cmd='leave_mesh' data-confirm='Run leave "
      "mesh signal on ALL discovered outputs?'>Leave Mesh</button>"
      "<button type='button' data-cmd='factory_reset' data-confirm='Run "
      "factory reset signal on ALL discovered outputs?'>Factory "
      "Reset</button>"
      "</div>"
      "<div id='fleet-content'><p>Loading discovered outputs&hellip;</p>"
      "</div>"
      "<script>(function(){"
      "var contentEl=document.getElementById('fleet-content');"
      "var refreshInFlight=false;"
      "function esc(s){return String(s).replace(/[&<>\"']/g,function(c){"
      "return "
      "{'&':'&amp;','<':'&lt;','>':'&gt;','\"':'&quot;',\"'\":'&#39;'}[c];"
      "});}"
      "function renderTable(servers){"
      "if(servers.length===0){"
      "contentEl.innerHTML='<p>No stickservers discovered yet. This page "
      "refreshes automatically in the background; wait a few "
      "seconds.</p>';return;}"
      "var maxRows=0;"
      "servers.forEach(function(s){if(s.outputs.length>maxRows)"
      "maxRows=s.outputs.length;});"
      "var html=\"<table id='fleet-table'><tr>\";"
      "servers.forEach(function(s){"
      "if(s.ip){html+=\"<th><a href='http://\"+esc(s.ip)+\"' "
      "target='_blank' rel='noopener'>\"+esc(s.label)+'</a></th>';}"
      "else{html+='<th>'+esc(s.label)+'</th>';}"
      "});"
      "html+='</tr>';"
      "for(var r=0;r<maxRows;r++){"
      "html+='<tr>';"
      "servers.forEach(function(s){"
      "var o=s.outputs[r];"
      "html+='<td>';"
      "if(o){"
      "html+=\"<button type='button' class='output-toggle "
      "\"+(o.on?'output-on':'output-off')+\"' data-topic='\"+esc(s.topic)+"
      "\"' data-euid='\"+esc(o.euid)+\"' "
      "data-cmd='\"+(o.on?'power_off':'power_on')+\"'>\"+esc(o.name)+"
      "'</button>';"
      "}"
      "html+='</td>';"
      "});"
      "html+='</tr>';"
      "}"
      "html+='</table>';"
      "contentEl.innerHTML=html;"
      "}"
      "function applyData(data){"
      "if(!data.ok){"
      "if(data.reason==='mqtt'){"
      "contentEl.innerHTML=\"<p style='color:#b00020;'><strong>MQTT is not "
      "connected.</strong> Enable and configure MQTT on the Network "
      "settings page to discover stickservers.</p>\";"
      "}else{"
      "contentEl.innerHTML=\"<p style='color:#b00020;'><strong>Stickserver "
      "discovery is disabled.</strong> Enable &quot;Query for "
      "stickservers&quot; or &quot;Passively parse hello/list "
      "responses&quot; on the Network settings page to populate this "
      "page.</p>\";"
      "}"
      "return;"
      "}"
      "renderTable(data.servers);"
      "}"
      // Guarded against overlapping fetches: on a slow/weak link a refresh
      // can still be in flight when the next interval tick fires, and a
      // second concurrent connection opened mid-transfer has been observed
      // to crash the device. This flag now also guards toggle/bulk-action
      // POSTs (previously unguarded against the periodic refresh, so a
      // click could open a second simultaneous connection while a refresh
      // was in flight and crash the device -- seen as the browser's
      // fetch() aborting mid-request with 'TypeError: Failed to fetch').
      "function refreshFleetContent(){"
      "if(refreshInFlight)return;"
      "refreshInFlight=true;"
      "fetch('/fleet/data').then(function(r){return r.json();})"
      ".then(applyData).catch(function(){})"
      ".then(function(){refreshInFlight=false;});"
      "}"
      "refreshFleetContent();"
      "setInterval(refreshFleetContent,5000);"
      // Per-output toggle buttons are rebuilt from scratch on every
      // refresh, so clicks are handled via delegation on the stable
      // container instead of binding to each button.
      "contentEl.addEventListener('click',function(e){"
      "var btn=e.target.closest('button.output-toggle');"
      "if(!btn)return;"
      "if(refreshInFlight){"
      "alert('Busy refreshing, please try again in a moment.');return;}"
      "refreshInFlight=true;"
      "var body=new URLSearchParams();"
      "body.set('topic',btn.getAttribute('data-topic'));"
      "body.set('euid',btn.getAttribute('data-euid'));"
      "body.set('cmd',btn.getAttribute('data-cmd'));"
      "fetch('/fleet/toggle',{method:'POST',body:body})"
      // fetch() only rejects on a network failure; an HTTP error status
      // (e.g. 400/503) still resolves normally and was previously
      // swallowed here, silently appearing to do nothing. Surface it.
      ".then(function(r){if(!r.ok){r.text().then(function(t){"
      "alert('Action failed (HTTP '+r.status+'): '+(t||r.statusText));"
      "});}})"
      ".catch(function(err){alert('Action failed: '+err);})"
      ".then(function(){refreshInFlight=false;"
      "setTimeout(refreshFleetContent,1000);});"
      "});"
      "document.querySelectorAll('.bulk-actions "
      "button').forEach(function(btn){"
      "btn.addEventListener('click',function(){"
      "var confirmMsg=btn.getAttribute('data-confirm');"
      "if(confirmMsg&&!confirm(confirmMsg))return;"
      "if(refreshInFlight){"
      "alert('Busy refreshing, please try again in a moment.');return;}"
      "refreshInFlight=true;"
      "var body=new URLSearchParams();"
      "body.set('cmd',btn.getAttribute('data-cmd'));"
      "fetch('/fleet/bulk',{method:'POST',body:body})"
      ".then(function(r){if(!r.ok){r.text().then(function(t){"
      "alert('Action failed (HTTP '+r.status+'): '+(t||r.statusText));"
      "});}})"
      ".catch(function(err){alert('Action failed: '+err);})"
      ".then(function(){refreshInFlight=false;"
      "setTimeout(refreshFleetContent,1000);});"
      "});"
      "});"
      "})();</script></body></html>");
  sendChunkedHtml(200, piece);
}

// Lightweight REST endpoint returning the full "All Outputs" state as JSON
// (see renderFleetDataJson()). Polled periodically by the /fleet page's
// inline script, which builds/rebuilds the output table client-side from
// this data so the device never has to re-render HTML markup per refresh.
void handleStickserverFleetData() {
  if (!ensureAuthorized())
    return;
  beginChunkedHtml(200, "application/json");
  renderFleetDataJson();
  endChunkedHtml();
}

void handleFleetOutputToggle() {
  if (!ensureAuthorized())
    return;
  if (!server.hasArg("topic") || !server.hasArg("euid") ||
      !server.hasArg("cmd")) {
    server.send(400, "text/plain", "Missing topic/euid/cmd");
    return;
  }
  String topic = server.arg("topic");
  String euid = server.arg("euid");
  String cmd = server.arg("cmd");
  if (topic.isEmpty() || euid.isEmpty() ||
      (cmd != "power_on" && cmd != "power_off")) {
    server.send(400, "text/plain", "Invalid toggle request");
    return;
  }
  if (!cfg.mqttEnabled || !mqttClient.connected()) {
    server.send(503, "text/plain", "MQTT is not connected");
    return;
  }
  logStatus(String(F("Fleet output toggle (")) + cmd + F(") requested for "
            "euid ") + euid + F(" on topic ") + topic + F(" via web UI."));
  JsonDocument req;
  req["cmd"] = cmd;
  req["ver"] = STICKSERVER_PROTOCOL_VERSION;
  req["mid"] = String(F("fleet-toggle-")) + String(millis());
  req["euid"] = euid;
  String payload;
  serializeJson(req, payload);
  mqttClient.publish(topic.c_str(), payload.c_str());
  // Called via fetch() from the "All Outputs" page's JS, not a native form
  // submission, so a plain ack is returned instead of a redirect -- a 303
  // to /fleet would otherwise be followed silently by fetch() and waste a
  // full extra request/response for a body the caller never looks at.
  server.send(200, "text/plain", "ok");
}

void handleFleetBulkAction() {
  if (!ensureAuthorized())
    return;
  if (!server.hasArg("cmd")) {
    server.send(400, "text/plain", "Missing cmd");
    return;
  }
  String cmd = server.arg("cmd");
  if (cmd != "power_on" && cmd != "power_off" && cmd != "leave_mesh" &&
      cmd != "factory_reset") {
    server.send(400, "text/plain", "Invalid cmd");
    return;
  }
  if (!cfg.mqttEnabled || !mqttClient.connected()) {
    server.send(503, "text/plain", "MQTT is not connected");
    return;
  }
  logStatus(String(F("Fleet bulk action (")) + cmd +
            F(") requested for all discovered outputs via web UI."));
  // Send one command per discovered output, across every discovered
  // stickserver (not just this device's own outputs). Each stickserver
  // only needs the single command published to its own instance topic;
  // yield() between publishes keeps the loop responsive if there turn out
  // to be many discovered outputs.
  uint16_t sent = 0;
  for (uint8_t i = 0; i < MAX_DISCOVERED_SERVERS; ++i) {
    if (!discoveredServers[i].active)
      continue;
    const DiscoveredServer &s = discoveredServers[i];
    for (uint8_t j = 0; j < s.outputCount; ++j) {
      if (!s.outputs[j].valid)
        continue;
      JsonDocument req;
      req["cmd"] = cmd;
      req["ver"] = STICKSERVER_PROTOCOL_VERSION;
      req["mid"] =
          String(F("fleet-bulk-")) + String(millis()) + "-" + String(sent);
      req["euid"] = s.outputs[j].euid;
      String payload;
      serializeJson(req, payload);
      mqttClient.publish(s.instanceTopic.c_str(), payload.c_str());
      ++sent;
      yield();
    }
  }
  // Called via fetch() from the "All Outputs" page's JS, not a native form
  // submission, so a plain ack is returned instead of a redirect (see
  // handleFleetOutputToggle() for why).
  server.send(200, "text/plain", "ok");
}

void handleNotFound() { server.send(404, "text/plain", "Not found"); }

} // namespace vibrant
