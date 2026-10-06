#pragma once
#include <Arduino.h>
#include <ESP8266WiFi.h>

namespace c4matrix {
String macLastThreeOctets(const String &);
String defaultHostnameFromMac(const String &);
String defaultSoftApSsidFromMac(const String &);
void applyWifiSettings();
void maintainWifiConnection();
}
