#include "Config.h"
#include "Debug.h"
#include "WifiManager.h"
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <LittleFS.h>
#include <math.h>
extern "C" {
#include "user_interface.h"
}

namespace c4matrix {
const char CONFIG_PATH[] = "/matrix_config.json";
const char DEFAULT_STA_SSID[] = "Z-Wave Automation";
const char DEFAULT_STA_PASSWORD[] = "Fiber714Cvet";
const char DEFAULT_AP_PASSWORD[] = "Fiber714Cvet";
DeviceConfig cfg;

bool parseMac(const String &mac, uint8_t out[6]) {
  if (mac.length() != 17)
    return false;
  for (uint8_t i = 0; i < 6; ++i) {
    if (i < 5 && mac[i * 3 + 2] != ':')
      return false;
    auto hex = [](char c) -> int {
      if (c >= '0' && c <= '9')
        return c - '0';
      if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
      if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
      return -1;
    };
    int high = hex(mac[i * 3]);
    int low = hex(mac[i * 3 + 1]);
    if (high < 0 || low < 0)
      return false;
    out[i] = static_cast<uint8_t>((high << 4) | low);
  }
  return true;
}

bool applyConfiguredMac() {
  uint8_t mac[6];
  if (!parseMac(cfg.mac, mac)) {
    logWarning(F("Invalid configured MAC; using hardware MAC."));
    return false;
  }
  bool stationOk = wifi_set_macaddr(STATION_IF, mac);
  bool accessPointOk = wifi_set_macaddr(SOFTAP_IF, mac);
  if (!stationOk || !accessPointOk) {
    logWarning(F("Could not apply custom MAC to both Wi-Fi interfaces."));
    return false;
  }
  logStatus(String(F("Applied custom MAC: ")) + cfg.mac);
  return true;
}

void setFactoryDefaults() {
  cfg.mac = WiFi.softAPmacAddress();
  cfg.useCustomMac = false;
  cfg.hostname = defaultHostnameFromMac(cfg.mac);
  cfg.staSsid = DEFAULT_STA_SSID;
  cfg.staPassword = DEFAULT_STA_PASSWORD;
  cfg.apPassword = DEFAULT_AP_PASSWORD;
  cfg.wifiPower = 20.5f;
  cfg.displayPin = 16;
  cfg.brightness = 40;
  cfg.textColor = 0x00FF00;
  cfg.matrixWidth = 32;
  cfg.matrixHeight = 8;
  cfg.mode = DisplayMode::Clock;
  cfg.fillColor = 0xFFFFFF;
  cfg.ledCount = 0;
  cfg.serpentine = true;
  cfg.flipHorizontal = false;
  cfg.scrollEnabled = true;
  cfg.scrollRight = false;
  cfg.scrollSpeed = 80;
  cfg.text = "";
  cfg.arduinoOtaEnabled = true;
  cfg.debugSerial = false;
  cfg.utcOffsetMinutes = 0;
  logStatus(F("Factory defaults loaded."));
}

bool saveConfig() {
  JsonDocument doc;
  doc["mac"] = cfg.mac;
  doc["useCustomMac"] = cfg.useCustomMac;
  doc["hostname"] = cfg.hostname;
  doc["staSsid"] = cfg.staSsid;
  doc["staPassword"] = cfg.staPassword;
  doc["apPassword"] = cfg.apPassword;
  doc["wifiPower"] = cfg.wifiPower;
  doc["displayPin"] = cfg.displayPin;
  doc["brightness"] = cfg.brightness;
  doc["textColor"] = cfg.textColor;
  doc["matrixWidth"] = cfg.matrixWidth;
  doc["matrixHeight"] = cfg.matrixHeight;
  doc["mode"] = displayModeName(cfg.mode);
  doc["fillColor"] = cfg.fillColor;
  doc["ledCount"] = cfg.ledCount;
  doc["serpentine"] = cfg.serpentine;
  doc["flipHorizontal"] = cfg.flipHorizontal;
  doc["scrollEnabled"] = cfg.scrollEnabled;
  doc["scrollRight"] = cfg.scrollRight;
  doc["scrollSpeed"] = cfg.scrollSpeed;
  doc["text"] = cfg.text;
  doc["arduinoOtaEnabled"] = cfg.arduinoOtaEnabled;
  doc["debugSerial"] = cfg.debugSerial;
  doc["utcOffsetMinutes"] = cfg.utcOffsetMinutes;
  File file = LittleFS.open(CONFIG_PATH, "w");
  if (!file) {
    logError(F("Could not open configuration file for writing."));
    return false;
  }
  bool ok = serializeJson(doc, file) > 0;
  file.close();
  if (!ok) {
    logError(F("Could not serialize configuration JSON."));
    return false;
  }
  logStatus(F("Configuration saved to LittleFS."));
  return true;
}

bool loadConfig() {
  if (!LittleFS.exists(CONFIG_PATH)) {
    logStatus(F("No configuration file; saving factory defaults."));
    setFactoryDefaults();
    return saveConfig();
  }
  File file = LittleFS.open(CONFIG_PATH, "r");
  if (!file) {
    logError(F("Could not open configuration; loading factory defaults."));
    setFactoryDefaults();
    return saveConfig();
  }
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, file);
  file.close();
  if (error) {
    logError(String(F("Invalid configuration JSON: ")) + error.c_str());
    setFactoryDefaults();
    return saveConfig();
  }

  cfg.mac = doc["mac"] | WiFi.softAPmacAddress();
  cfg.useCustomMac = doc["useCustomMac"] | false;
  cfg.hostname = doc["hostname"] | defaultHostnameFromMac(cfg.mac);
  cfg.staSsid = doc["staSsid"] | String(DEFAULT_STA_SSID);
  cfg.staPassword = doc["staPassword"] | String(DEFAULT_STA_PASSWORD);
  cfg.apPassword = doc["apPassword"] | String(DEFAULT_AP_PASSWORD);
  cfg.wifiPower = doc["wifiPower"] | 20.5f;
  int displayPin = doc["displayPin"] | 16;
  int brightness = doc["brightness"] | 40;
  cfg.textColor = (doc["textColor"] | 0x00FF00UL) & 0xFFFFFFUL;
  int matrixWidthValue = doc["matrixWidth"] | 32;
  int matrixHeightValue = doc["matrixHeight"] | 8;
  String mode = doc["mode"] | String("clock");
  cfg.mode = mode == "fill"    ? DisplayMode::Fill
             : mode == "off"   ? DisplayMode::Off
             : mode == "count" ? DisplayMode::Count
             : mode == "clock" ? DisplayMode::Clock
                                : DisplayMode::Text;
  cfg.fillColor = (doc["fillColor"] | 0xFFFFFFUL) & 0xFFFFFFUL;
  int ledCount = doc["ledCount"] | 0;
  cfg.serpentine = doc["serpentine"] | true;
  cfg.flipHorizontal = doc["flipHorizontal"] | false;
  cfg.scrollEnabled = doc["scrollEnabled"] | true;
  cfg.scrollRight = doc["scrollRight"] | false;
  int scrollSpeed = doc["scrollSpeed"] | 80;
  cfg.text = doc["text"] | String("");
  cfg.arduinoOtaEnabled = doc["arduinoOtaEnabled"] | true;
  cfg.debugSerial = doc["debugSerial"] | false;
  int utcOffsetMinutes = doc["utcOffsetMinutes"] | 0;
  cfg.utcOffsetMinutes = utcOffsetMinutes >= -720 && utcOffsetMinutes <= 840
                             ? static_cast<int16_t>(utcOffsetMinutes)
                             : 0;
  if (cfg.hostname.isEmpty())
    cfg.hostname = defaultHostnameFromMac(cfg.mac);
  if (cfg.staSsid.isEmpty())
    cfg.staSsid = DEFAULT_STA_SSID;
  if (cfg.staPassword.isEmpty())
    cfg.staPassword = DEFAULT_STA_PASSWORD;
  if (cfg.apPassword.length() < 8)
    cfg.apPassword = DEFAULT_AP_PASSWORD;
  if (!isfinite(cfg.wifiPower) || cfg.wifiPower < 5.0f ||
      cfg.wifiPower > 20.5f)
    cfg.wifiPower = 20.5f;
  if (displayPin != 16 && displayPin != 5 && displayPin != 4 &&
      displayPin != 0 && displayPin != 2 && displayPin != 14 &&
      displayPin != 12 && displayPin != 13 && displayPin != 15)
    displayPin = 16;
  cfg.displayPin = static_cast<int8_t>(displayPin);
  cfg.brightness = brightness >= 0 && brightness <= 255
                       ? static_cast<uint8_t>(brightness)
                       : 40;
  cfg.scrollSpeed = scrollSpeed >= 10 && scrollSpeed <= 1000
                        ? static_cast<uint16_t>(scrollSpeed)
                        : 80;
  bool validMatrixSize = matrixWidthValue >= 1 &&
                         matrixWidthValue <= MATRIX_MAX_WIDTH &&
                         matrixHeightValue >= 1 &&
                         matrixHeightValue <= MATRIX_MAX_HEIGHT;
  cfg.matrixWidth = validMatrixSize
                        ? static_cast<uint16_t>(matrixWidthValue)
                        : DEFAULT_MATRIX_WIDTH;
  cfg.matrixHeight = validMatrixSize
                         ? static_cast<uint16_t>(matrixHeightValue)
                         : DEFAULT_MATRIX_HEIGHT;
  uint16_t configuredLedCount =
      static_cast<uint16_t>(cfg.matrixWidth) * cfg.matrixHeight;
  cfg.ledCount = ledCount >= 0 && ledCount <= configuredLedCount
                     ? static_cast<uint16_t>(ledCount)
                     : 0;
  if (cfg.text.length() > 128)
    cfg.text.remove(128);
  if (cfg.useCustomMac) {
    uint8_t parsedMac[6];
    if (!parseMac(cfg.mac, parsedMac)) {
      cfg.mac = WiFi.softAPmacAddress();
      cfg.useCustomMac = false;
      logWarning(F("Invalid saved custom MAC; using hardware MAC."));
    }
  }
  logStatus(F("Configuration loaded."));
  return true;
}

void performFactoryResetAndRestart(const String &reason) {
  logWarning(reason);
  setFactoryDefaults();
  if (!saveConfig()) {
    logError(F("Factory reset could not be saved."));
    delay(1000);
    ESP.restart();
  }
  delay(300);
  ESP.restart();
}
}
