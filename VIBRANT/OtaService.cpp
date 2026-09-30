#include "OtaService.h"
#include "Config.h"
#include "Debug.h"
#include "../libraries/EmcommCommon/src/EmcommCommon/ArduinoOta.h"
#include <ArduinoOTA.h>

namespace vibrant {

bool arduinoOtaActive = false, arduinoOtaCallbacksConfigured = false;

void applyArduinoOtaSettings() {
  if (!cfg.arduinoOtaEnabled) {
    if (arduinoOtaActive) {
      logStatus(F("ArduinoOTA disabled in settings. OTA request handling is "
                  "now paused."));
    } else {
      logStatus(F("ArduinoOTA is disabled."));
    }
    arduinoOtaActive = false;
    return;
  }

  if (!arduinoOtaCallbacksConfigured) {
    emcomm::setArduinoOtaCallbacks(
        ArduinoOTA,
        []() {
          String mode = (ArduinoOTA.getCommand() == U_FLASH) ? F("firmware")
                                                             : F("filesystem");
          logStatus(String(F("ArduinoOTA start (")) + mode + F(")."));
          DBG("ArduinoOTA host: %s", cfg.hostname.c_str());
        },
        []() {
          logStatus(F("ArduinoOTA completed."));
          DBG("ArduinoOTA transfer finished successfully.");
        },
        [](unsigned int progress, unsigned int total) {
          if (cfg.debugSerial) {
            unsigned int percent =
                (total == 0U) ? 0U : (progress * 100U) / total;
            DBG("ArduinoOTA progress: %u%%", percent);
          }
        },
        [](ota_error_t error) {
          logError(String(F("ArduinoOTA error #")) +
                   String(static_cast<int>(error)));
          DBG("ArduinoOTA transfer aborted due to error.");
        });
    arduinoOtaCallbacksConfigured = true;
  }

  emcomm::startArduinoOta(ArduinoOTA, cfg.hostname.c_str(),
                          cfg.apPassword.c_str());
  arduinoOtaActive = true;
  logStatus(String(F("ArduinoOTA enabled on hostname: ")) + cfg.hostname);
}

} // namespace vibrant
