#include "Actions.h"
#include "BackupRestore.h"
#include "BootDiagnostics.h"
#include "Config.h"
#include "Debug.h"
#include "MqttClient.h"
#include "OtaService.h"
#include "Outputs.h"
#include "Runtime.h"
#include "Stickserver.h"
#include "Version.h"
#include "WebServer.h"
#include "WifiManager.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <LittleFS.h>
#include <PubSubClient.h>
#include <Updater.h>
using namespace vibrant;

void setup() {
  Serial.begin(115200);
  bootStartMillis = millis();
  delay(100);
  Serial.println();
  Serial.println(F("[INFO] VIBRANT boot starting..."));
  Serial.print(F("[INFO] Software version: "));
  Serial.println(SOFTWARE_VERSION);

  pinMode(FLASH_BUTTON_PIN, INPUT_PULLUP);

  logStatus(F("Mounting LittleFS..."));
  if (!LittleFS.begin()) {
    logError(F("LittleFS mount failed. Formatting filesystem; saved "
               "configuration will be erased."));
    if (!LittleFS.format()) {
      restartDevice(F("LittleFS format failed after mount failure."));
    }
    if (!LittleFS.begin()) {
      restartDevice(F("LittleFS mount failed after format."));
    }
    logStatus(F("LittleFS mount succeeded after format."));
  } else {
    logStatus(F("LittleFS mounted successfully."));
  }

  WiFi.mode(WIFI_AP_STA);
  logStatus(F("Loading runtime configuration..."));
  if (!loadConfig()) {
    logError(F("Configuration load path failed; restoring factory defaults."));
    setFactoryDefaults();
    if (!saveConfig()) {
      restartDevice(F("Failed to save factory defaults during boot recovery."));
    }
  }

  checkFlashFactoryResetOnBoot();
  if (cfg.debugSerial)
    logLoadedWifiConfig();
  applyWifiSettings();
  if (cfg.debugSerial)
    logWifiScan();
  prepareOutputsForBootPhase();
  mqttEnsureConnected();

  registerWebRoutes();
  applyMqttSettings();
  applyArduinoOtaSettings();
  logWifiSummary(defaultSoftApSsidFromMac(cfg.mac));
  logStatus(F("Boot sequence complete."));
}

void loop() {
  if (arduinoOtaActive) {
    ArduinoOTA.handle();
  }
  // While an ArduinoOTA transfer is actively writing flash, skip everything
  // else: the web server's chunked-write retries, MQTT, and stickserver
  // discovery can each block for up to several seconds, which previously
  // starved ArduinoOTA.handle() of CPU time mid-transfer and made uploads
  // fail and need 2-3 retries. See OtaService.h for details.
  if (otaTransferInProgress) {
    return;
  }
  server.handleClient();
  if (!outputsActivated && outputActivationDelayElapsed()) {
    applyOutputsWhenSafe();
  }
  maintainWifiConnection();
  maintainMqtt();
  maintainStickserverDiscovery();
  maintainBackgroundAction();
}
