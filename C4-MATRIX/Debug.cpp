#include "Debug.h"
#include <ESP8266WiFi.h>

namespace c4matrix {
void logStatus(const String &message) {
  Serial.print(F("[INFO] "));
  Serial.println(message);
}
void logWarning(const String &message) {
  Serial.print(F("[WARN] "));
  Serial.println(message);
}
void logError(const String &message) {
  Serial.print(F("[ERROR] "));
  Serial.println(message);
}
void restartDevice(const String &reason) {
  logError(String(F("Restarting: ")) + reason);
  delay(1500);
  ESP.restart();
}
}
