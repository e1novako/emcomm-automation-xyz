#pragma once
#include <Arduino.h>
#include "Display.h"

namespace c4matrix {
struct DeviceConfig {
  String mac;
  bool useCustomMac;
  String hostname;
  String staSsid, staPassword, apPassword;
  float wifiPower;
  int8_t displayPin;
  uint8_t brightness;
  uint32_t textColor;
  DisplayMode mode;
  uint32_t fillColor;
  bool serpentine, flipHorizontal;
  bool scrollEnabled;
  bool scrollRight;
  uint16_t scrollSpeed;
  String text;
  bool arduinoOtaEnabled, debugSerial;
};

extern const char CONFIG_PATH[];
extern const char DEFAULT_STA_SSID[];
extern const char DEFAULT_STA_PASSWORD[];
extern const char DEFAULT_AP_PASSWORD[];
extern DeviceConfig cfg;
void setFactoryDefaults();
bool saveConfig();
bool loadConfig();
bool parseMac(const String &, uint8_t out[6]);
bool applyConfiguredMac();
void performFactoryResetAndRestart(const String &);
}
