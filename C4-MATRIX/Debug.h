#pragma once
#include "Config.h"
#include <Arduino.h>

namespace c4matrix {
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
}
