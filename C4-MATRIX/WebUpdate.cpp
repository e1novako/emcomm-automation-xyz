#include "WebUpdate.h"
#include "Config.h"
#include "Debug.h"
#include "Display.h"
#include "OtaService.h"
#include "WebServer.h"
#include <ESP8266WebServer.h>
#include <Updater.h>

namespace c4matrix {
static bool updateFailed = false;
static String updateError;

void handleFirmwareUpdatePage() {
  if (!ensureAuthorized())
    return;
  server.send(
      200, "text/html; charset=utf-8",
      F("<!doctype html><html><head><meta charset='utf-8'><meta "
        "name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>C4-MATRIX - Firmware Update</title></head><body "
        "style='font:16px Arial,sans-serif;margin:24px auto;max-width:700px;"
        "padding:0 14px'><h1 style='text-align:center'>C4-MATRIX - Firmware "
        "Update</h1><p><a href='/config/ota'>"
        "OTA settings</a> | <a href='/'>Display</a></p><p>Upload a compiled "
        "ESP8266 firmware .bin file. The device will reboot after a successful "
        "update. Do not power off during the transfer.</p><form method='post' "
        "action='/update' enctype='multipart/form-data'><input type='file' "
        "name='firmware' accept='.bin' required><button type='submit'>"
        "Upload and flash</button></form></body></html>"));
}

void handleFirmwareUpdateUpload() {
  if (!server.authenticate("admin", cfg.apPassword.c_str())) {
    server.requestAuthentication();
    return;
  }
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    updateFailed = false;
    updateError = "";
    logStatus(String(F("Web OTA upload started: ")) + upload.filename);
    uint32_t freeSpace = ESP.getFreeSketchSpace();
    uint32_t maxSketchSize =
        freeSpace > 0x1000 ? (freeSpace - 0x1000) & 0xFFFFF000UL : 0;
    if (maxSketchSize == 0 || !Update.begin(maxSketchSize)) {
      updateFailed = true;
      updateError = Update.getErrorString();
      logError(String(F("Web OTA could not begin: ")) + updateError);
    } else {
      otaTransferInProgress = true;
      displayShowOtaIcon();
    }
  } else if (upload.status == UPLOAD_FILE_WRITE && !updateFailed) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      updateFailed = true;
      updateError = Update.getErrorString();
      logError(String(F("Web OTA write failed: ")) + updateError);
    }
    DBG("Web OTA chunk received: %u bytes", upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END && !updateFailed) {
    if (!Update.end(true)) {
      updateFailed = true;
      updateError = Update.getErrorString();
      logError(String(F("Web OTA finalize failed: ")) + updateError);
    } else {
      logStatus(String(F("Web OTA complete: ")) + String(upload.totalSize) +
                F(" bytes."));
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    updateFailed = true;
    updateError = F("Upload aborted by client.");
    Update.end(false);
    logWarning(F("Web OTA upload aborted."));
  }
  // The device reboots automatically after a successful upload, so the icon
  // only needs to be cleared when the transfer failed and execution will
  // continue running the current firmware.
  if (updateFailed && otaTransferInProgress) {
    otaTransferInProgress = false;
    displayForceRedraw();
  }
}

void handleFirmwareUpdateDone() {
  if (!ensureAuthorized())
    return;
  if (updateFailed) {
    String response = F("<!doctype html><html><body><h1>Update failed</h1><p>");
    response += htmlEscape(updateError);
    response += F("</p><p><a href='/update'>Try again</a></p></body></html>");
    server.send(500, "text/html; charset=utf-8", response);
    return;
  }
  server.send(200, "text/html; charset=utf-8",
              F("<!doctype html><html><head><meta charset='utf-8'>"
                "<meta http-equiv='refresh' content='15;url=/'>"
                "</head><body><h1>Update successful</h1><p>Rebooting...</p>"
                "</body></html>"));
  server.client().flush();
  delay(200);
  ESP.restart();
}
}
