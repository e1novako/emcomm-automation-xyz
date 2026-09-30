#pragma once
#include "Config.h"
#include <ESP8266WiFi.h>
namespace vibrant {
extern const unsigned long WIFI_RECONNECT_INTERVAL_MS,
    WIFI_CONNECT_LOG_INTERVAL_MS, WIFI_RECOVERY_WINDOW_MS;
extern const uint8_t MAX_WIFI_RECOVERY_ATTEMPTS;
extern wl_status_t lastWifiStatus;
extern unsigned long lastWifiReconnectAttemptMs, lastWifiConnectLogMs,
    wifiDisconnectSinceMs;
extern uint8_t wifiRecoveryAttempts;
const char *wifiStatusToString(wl_status_t);
String macLastThreeOctets(const String &);
String defaultHostnameFromMac(const String &);
String defaultSoftApSsidFromMac(const String &);
void resetWifiRecoveryState();
void applyWifiSettings();
void maintainWifiConnection();
} // namespace vibrant
