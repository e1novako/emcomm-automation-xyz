#include "BootDiagnostics.h"
#include "Config.h"
#include "Debug.h"
#include "Display.h"
#include "OtaService.h"
#include "Version.h"
#include "WebServer.h"
#include "WifiManager.h"
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <LittleFS.h>

using namespace c4matrix;

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
  if (!loadConfig()) {
    setFactoryDefaults();
    if (!saveConfig())
      restartDevice(F("Could not save factory configuration."));
  }
  applyWifiSettings();
  displayBegin(cfg.displayPin);
  displaySetBrightness(cfg.brightness);
  displaySetColor(cfg.textColor);
  displaySetSerpentine(cfg.serpentine);
  displaySetOrientation(cfg.flipHorizontal);
  displaySetText(cfg.text);
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
  maintainDisplay();
  yield();
}
