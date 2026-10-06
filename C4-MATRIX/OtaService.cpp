#include "OtaService.h"
#include "Config.h"
#include "Debug.h"
#include <ArduinoOTA.h>

namespace c4matrix {
bool arduinoOtaActive = false;
bool otaTransferInProgress = false;
static bool callbacksConfigured = false;

void applyArduinoOtaSettings() {
  if (!cfg.arduinoOtaEnabled) {
    arduinoOtaActive = false;
    logStatus(F("ArduinoOTA disabled by configuration."));
    return;
  }
  ArduinoOTA.setHostname(cfg.hostname.c_str());
  ArduinoOTA.setPassword(cfg.apPassword.c_str());
  if (!callbacksConfigured) {
    ArduinoOTA.onStart([]() {
      otaTransferInProgress = true;
      logStatus(F("ArduinoOTA transfer started."));
    });
    ArduinoOTA.onEnd([]() {
      otaTransferInProgress = false;
      logStatus(F("ArduinoOTA transfer complete."));
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      if (cfg.debugSerial && total) {
        DBG("ArduinoOTA progress: %u%%", progress * 100U / total);
      }
    });
    ArduinoOTA.onError([](ota_error_t error) {
      otaTransferInProgress = false;
      logError(String(F("ArduinoOTA error #")) +
               String(static_cast<int>(error)));
    });
    callbacksConfigured = true;
  }
  ArduinoOTA.begin();
  arduinoOtaActive = true;
  logStatus(String(F("ArduinoOTA enabled: ")) + cfg.hostname);
}

void maintainArduinoOta() {
  if (arduinoOtaActive)
    ArduinoOTA.handle();
}
}
