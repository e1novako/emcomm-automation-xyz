#include "BootDiagnostics.h"
#include "Config.h"
#include "Debug.h"
#include "Display.h"
#include "NtpClock.h"
#include "OtaService.h"
#include "Version.h"
#include "WebServer.h"
#include "WifiManager.h"
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <LittleFS.h>

using namespace c4matrix;

// GPIO0 is the NodeMCU "FLASH"/BOOT button, pulled up externally. Holding it
// low for FACTORY_RESET_HOLD_MS while the board starts up (power-on or
// reset) restores factory defaults -- useful for recovering a device whose
// saved Wi-Fi credentials are wrong and unreachable over the network.
static const uint8_t FACTORY_RESET_BUTTON_PIN = 0;
static const unsigned long FACTORY_RESET_HOLD_MS = 3000;

static void checkFactoryResetButton() {
  pinMode(FACTORY_RESET_BUTTON_PIN, INPUT_PULLUP);
  delay(20); // let the pin settle before sampling it
  if (digitalRead(FACTORY_RESET_BUTTON_PIN) != LOW)
    return;
  logWarning(F("Boot button held at startup; keep holding for 3s to factory "
               "reset..."));
  unsigned long start = millis();
  while (digitalRead(FACTORY_RESET_BUTTON_PIN) == LOW) {
    if (millis() - start >= FACTORY_RESET_HOLD_MS) {
      performFactoryResetAndRestart(
          F("Factory reset requested via boot button hold."));
      return; // unreachable: performFactoryResetAndRestart() restarts
    }
    delay(20);
  }
  logStatus(F("Boot button released before hold threshold; continuing "
              "normal boot."));
}

void setup() {
  Serial.begin(115200);
  bootStartMillis = millis();
  delay(100);
  Serial.println();
  logStatus(F("C4-MATRIX boot starting."));
  logStatus(String(F("Firmware version: ")) + SOFTWARE_VERSION);

  WiFi.mode(WIFI_AP_STA);
  if (!LittleFS.begin()) {
    logError(F("LittleFS mount failed; formatting configuration filesystem."));
    if (!LittleFS.format() || !LittleFS.begin())
      restartDevice(F("Could not initialize LittleFS."));
  }
  checkFactoryResetButton();
  if (!loadConfig()) {
    setFactoryDefaults();
    if (!saveConfig())
      restartDevice(F("Could not save factory configuration."));
  }
  applyWifiSettings();
  ntpBegin();
  displayBegin(cfg.displayPin);
  displaySetBrightness(cfg.brightness);
  displaySetColor(cfg.textColor);
  displaySetSerpentine(cfg.serpentine);
  displaySetOrientation(cfg.flipHorizontal);
  if (cfg.scrollEnabled)
    scrollStart();
  applyArduinoOtaSettings();
  registerWebRoutes();
  logBootDiagnostics();
  logStatus(F("C4-MATRIX boot complete."));
}

void loop() {
  maintainArduinoOta();
  if (otaTransferInProgress)
    return;
  server.handleClient();
  maintainWifiConnection();
  ntpMaintain();
  maintainDisplay();
  yield();
}
