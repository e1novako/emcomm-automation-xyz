#pragma once
#include "Config.h"
#include <Arduino.h>
namespace vibrant {
#define DBG(...)                                                               \
  do {                                                                         \
    if (cfg.debugSerial) {                                                     \
      Serial.print(F("[DEBUG] "));                                             \
      Serial.printf(__VA_ARGS__);                                              \
      Serial.println();                                                        \
    }                                                                          \
  } while (0)
void logStatus(const String &);
void logWarning(const String &);
void logError(const String &);
void restartDevice(const String &);
void logDeviceSummary();
void logWifiSummary(const String &);
void logLoadedWifiConfig();
void logWifiScan();
} // namespace vibrant
