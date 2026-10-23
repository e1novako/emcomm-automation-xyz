#include "BackupRestore.h"
#include "Actions.h"
#include "Config.h"
#include "Debug.h"
#include "Reservations.h"
#include "Runtime.h"
#include "WebServer.h"
#include <ArduinoJson.h>

namespace vibrant {

File importFile;
bool importFailed = false;
size_t importBytes = 0;
// A real backup is a few KB; the cap stops an oversized upload from filling
// flash or exhausting heap when it is parsed.
constexpr size_t MAX_IMPORT_BYTES = 16384;

void handleConfigExport() {
  if (!ensureAuthorized())
    return;

  // Ensure the exported backup reflects each output's current reservation,
  // not just whatever was last written to flash (reservation changes are
  // only auto-saved when restoreReservationsOnBoot is enabled). An explicit
  // backup export always captures the live reservation state.
  syncReservationsIntoConfig();
  saveConfig();

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
    importBytes = 0;
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
    importBytes += upload.currentSize;
    if (importBytes > MAX_IMPORT_BYTES) {
      if (!importFailed)
        logError(F("Configuration import rejected: file too large."));
      importFailed = true;
    } else if (importFile) {
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

// Compares the device entries actually restored into cfg/outputReservations
// after an import against the values present in the uploaded backup JSON,
// logging a warning for every output whose manufacturer/model/name/pin,
// reservation, or (when restoreOutputStateOnBoot is enabled) ON/OFF state
// did not come through the restore exactly as uploaded. Returns true only
// if every output present in the backup matched.
static bool verifyImportedConfig(JsonDocument &importedDoc) {
  JsonArray devices = importedDoc["devices"].as<JsonArray>();
  if (devices.isNull() || devices.size() == 0) {
    logWarning(F("Import verification skipped: backup file contained no "
                 "devices array."));
    return true;
  }
  bool allMatched = true;
  for (uint8_t i = 0; i < MAX_DEVICES && i < devices.size(); ++i) {
    JsonObject d = devices[i];
    String expManufacturer =
        d["manufacturer"] | String(DEFAULT_DEVICE_MANUFACTURER);
    String expModel = d["model"] | String(DEFAULT_DEVICE_MODEL);
    expManufacturer.trim();
    expModel.trim();
    if (expManufacturer.isEmpty())
      expManufacturer = DEFAULT_DEVICE_MANUFACTURER;
    if (expModel.isEmpty())
      expModel = DEFAULT_DEVICE_MODEL;
    String expName = d["name"] | (String(F("Output ")) + String(i + 1));
    int8_t expPin = static_cast<int8_t>(d["pin"] | -1);
    DeviceModelType expDeviceModel = getDeviceModel(d["deviceModel"] | 0).type;
    int expTop = d["topPin"] | -1;
    int expBottom = d["bottomPin"] | -1;
    bool expReserved = d["reserved"] | false;
    String expOwner = d["reservedOwner"] | String("");

    bool mismatch = false;
    if (cfg.devices[i].manufacturer != expManufacturer ||
        cfg.devices[i].model != expModel || cfg.devices[i].name != expName ||
        cfg.devices[i].pin != expPin ||
        cfg.devices[i].deviceModel != expDeviceModel ||
        cfg.devices[i].topPin != expTop ||
        cfg.devices[i].bottomPin != expBottom) {
      mismatch = true;
    }
    if (outputReservations[i].reserved != expReserved ||
        (expReserved && outputReservations[i].owner != expOwner)) {
      mismatch = true;
    }
    if (cfg.restoreOutputStateOnBoot) {
      bool expState = d["state"] | false;
      if (cfg.devices[i].state != expState)
        mismatch = true;
    }
    if (mismatch) {
      allMatched = false;
      logWarning(String(F("Import verification mismatch for output ")) +
                 String(i + 1) +
                 F(": restored data does not match the backup file."));
    }
  }
  return allMatched;
}

void handleConfigImportDone() {
  if (!ensureAuthorized())
    return;
  if (isActionRunning())
    cancelAction();
  if (importFailed) {
    if (LittleFS.exists(IMPORT_CONFIG_PATH))
      LittleFS.remove(IMPORT_CONFIG_PATH);
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
  // An explicit backup restore always restores each output's reservation,
  // independent of restoreReservationsOnBoot (which only governs normal
  // power-cycle boot behavior, not a deliberate admin-initiated restore).
  for (uint8_t i = 0; i < MAX_DEVICES; ++i) {
    outputReservations[i].reserved = cfg.devices[i].reserved;
    outputReservations[i].owner = cfg.devices[i].reservedOwner;
  }
  if (!saveConfig()) {
    restartDevice(F("Failed to normalize and save imported configuration."));
  }
  if (verifyImportedConfig(verifyDoc)) {
    logStatus(F("Configuration import verified: all restored output data "
                "matches the backup file."));
  } else {
    logWarning(F("Configuration import completed, but some restored output "
                 "data did not match the backup file (see warnings above)."));
  }
  logStatus(F("Configuration import applied successfully."));
  applyRuntimeSettings();
  server.sendHeader("Location", "/settings/diagnostics");
  server.send(303);
}

void handleFactoryReset() {
  if (!ensureAuthorized())
    return;
  if (isActionRunning())
    cancelAction();

  logStatus(F("Factory reset requested."));
  setFactoryDefaults();
  if (!saveConfig()) {
    restartDevice(F("Failed to persist factory reset configuration."));
  }
  applyRuntimeSettings();
  server.sendHeader("Location", "/settings/diagnostics");
  server.send(303);
}

} // namespace vibrant
