#include "BootDiagnostics.h"
#include "Config.h"
#include "Debug.h"
#include "Version.h"
#include <ESP8266WiFi.h>

namespace c4matrix {
unsigned long bootStartMillis = 0;
void logBootDiagnostics() {
  logStatus(String(F("Firmware version: ")) + SOFTWARE_VERSION);
  logStatus(String(F("Hardware MAC: ")) + WiFi.macAddress());
  logStatus(String(F("Configured hostname: ")) + cfg.hostname);
  logStatus(String(F("Free heap: ")) + String(ESP.getFreeHeap()) + F(" bytes"));
  logStatus(String(F("SoftAP address: ")) + WiFi.softAPIP().toString());
  if (WiFi.status() == WL_CONNECTED)
    logStatus(String(F("Station address: ")) + WiFi.localIP().toString());
  DBG("Boot diagnostics complete at %lu ms", millis() - bootStartMillis);
}
}
