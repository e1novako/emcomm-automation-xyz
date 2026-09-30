#include "WebUpdate.h"
#include "Config.h"
#include "Debug.h"
#include "WebServer.h"
#include <ESP8266WebServer.h>
#include <Updater.h>

namespace vibrant {

bool otaUpdateFailed = false;
String otaUpdateError;

void handleFirmwareUpdatePage() {
  if (!ensureAuthorized())
    return;

  String html = F(
      "<!doctype html><html><head><meta charset='utf-8'><title>Firmware "
      "Update</title>"
      "<style>body{font-family:Arial,sans-serif;margin:20px;}fieldset{margin-"
      "bottom:16px;}"
      "label{display:block;margin:6px 0;}button{padding:8px "
      "10px;margin-right:8px;}"
      ".warn{color:#b00020;font-weight:bold;}</style></head><body>"
      "<h1>Firmware Update</h1>"
      "<p><a href='/settings'>Back to Settings</a></p>"
      "<p>Upload a compiled <code>.bin</code> firmware file to update the "
      "device. "
      "The device will reboot automatically after a successful update.</p>"
      "<p class='warn'>Warning: Do not power off the device during an update. "
      "Interrupted updates may require USB reflashing to recover.</p>"
      "<form method='post' action='/firmware/update' "
      "enctype='multipart/form-data'>"
      "<label>Firmware file (.bin) <input type='file' name='firmware' "
      "accept='.bin' required></label>"
      "<button type='submit'>Upload and flash</button></form>"
      "</body></html>");
  server.send(200, "text/html", html);
}

void handleFirmwareUpdateUpload() {
  if (!server.authenticate("admin", cfg.apPassword.c_str())) {
    server.requestAuthentication();
    return;
  }

  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    otaUpdateFailed = false;
    otaUpdateError = "";
    logStatus(String(F("OTA firmware update upload started: ")) +
              upload.filename);
    // Reserve flash space for the new sketch; subtract 4 KB (0x1000) as safety
    // margin and align down to a 4 KB flash sector boundary (0xFFFFF000 mask).
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
        logStatus(String(F("OTA firmware update upload complete: ")) +
                  String(upload.totalSize) + F(" bytes written."));
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
  if (!ensureAuthorized())
    return;
  if (otaUpdateFailed) {
    logError(String(F("OTA firmware update failed: ")) + otaUpdateError);
    String html = F("<!doctype html><html><head><meta "
                    "charset='utf-8'><title>Firmware Update Failed</title>"
                    "<style>body{font-family:Arial,sans-serif;margin:20px;}."
                    "err{color:#b00020;}</style></head><body>"
                    "<h1 class='err'>Firmware Update Failed</h1><p>");
    html += htmlEscape(otaUpdateError);
    html += F("</p><p><a href='/firmware/update'>Try again</a> | <a "
              "href='/settings'>Settings</a></p>"
              "</body></html>");
    server.send(500, "text/html", html);
    return;
  }
  logStatus(F("OTA firmware update succeeded. Rebooting..."));
  server.send(
      200, "text/html",
      F("<!doctype html><html><head><meta charset='utf-8'>"
        "<meta http-equiv='refresh' content='15;url=/'>"
        "<title>Update OK</title>"
        "<style>body{font-family:Arial,sans-serif;margin:20px;}</style></"
        "head><body>"
        "<h1>Firmware update successful</h1>"
        "<p>The device is rebooting. This page will reload in 15 seconds.</p>"
        "</body></html>"));
  // Flush the TCP send buffer and allow time for the HTTP response to reach the
  // client.
  server.client().flush();
  delay(200);
  ESP.restart();
}

} // namespace vibrant
