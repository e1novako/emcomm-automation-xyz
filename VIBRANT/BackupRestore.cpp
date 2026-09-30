#include "BackupRestore.h"
#include "Config.h"
#include "Debug.h"
#include "Runtime.h"
#include "WebServer.h"
#include <ArduinoJson.h>

namespace vibrant {

File importFile;
bool importFailed = false;

void handleConfigExport() {
  if (!ensureAuthorized())
    return;

  if (!LittleFS.exists(CONFIG_PATH)) {
    logError(
        F("Config export requested but configuration file was not found."));
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

  HTTPUpload &upload = server.upload();
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
      if (importFile.write(upload.buf, upload.currentSize) !=
          upload.currentSize) {
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
  if (!ensureAuthorized())
    return;
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
    logError(String(F("Uploaded configuration JSON is invalid: ")) +
             verifyError.c_str());
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
    restartDevice(
        F("Imported configuration could not be loaded after replace."));
  }
  if (!saveConfig()) {
    restartDevice(F("Failed to normalize and save imported configuration."));
  }
  logStatus(F("Configuration import applied successfully."));
  applyRuntimeSettings();
  server.sendHeader("Location", "/settings");
  server.send(303);
}

void handleFactoryReset() {
  if (!ensureAuthorized())
    return;

  logStatus(F("Factory reset requested."));
  setFactoryDefaults();
  if (!saveConfig()) {
    restartDevice(F("Failed to persist factory reset configuration."));
  }
  applyRuntimeSettings();
  server.sendHeader("Location", "/settings");
  server.send(303);
}

} // namespace vibrant
