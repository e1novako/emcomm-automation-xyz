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

// GPIO0 is the NodeMCU "FLASH"/BOOT button, pulled up externally. If it is
// pressed (held low) while the board starts up (power-on or reset), the
// device performs a factory reset -- useful for recovering a device whose
// saved Wi-Fi credentials are wrong and unreachable over the network.
static const uint8_t FACTORY_RESET_BUTTON_PIN = 0;

static void checkFactoryResetButton() {
  pinMode(FACTORY_RESET_BUTTON_PIN, INPUT_PULLUP);
  delay(20); // let the pin settle before sampling it, debouncing noise
  if (digitalRead(FACTORY_RESET_BUTTON_PIN) != LOW)
    return;
  performFactoryResetAndRestart(
      F("Factory reset requested: FLASH/BOOT button pressed at startup."));
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
