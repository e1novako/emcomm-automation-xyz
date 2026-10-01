#include "WifiManager.h"
#include "Config.h"
#include "Debug.h"
#include <ESP8266WiFi.h>

namespace vibrant {

wl_status_t lastWifiStatus = WL_IDLE_STATUS;
unsigned long lastWifiReconnectAttemptMs = 0, lastWifiConnectLogMs = 0,
              wifiDisconnectSinceMs = 0;
uint8_t wifiRecoveryAttempts = 0;

const char *wifiStatusToString(wl_status_t status) {
  switch (status) {
  case WL_CONNECTED:
    return "CONNECTED";
  case WL_NO_SSID_AVAIL:
    return "NO_SSID_AVAIL";
  case WL_CONNECT_FAILED:
    return "CONNECT_FAILED";
  case WL_WRONG_PASSWORD:
    return "WRONG_PASSWORD";
  case WL_IDLE_STATUS:
    return "IDLE";
  case WL_DISCONNECTED:
    return "DISCONNECTED";
  case WL_CONNECTION_LOST:
    return "CONNECTION_LOST";
  case WL_SCAN_COMPLETED:
    return "SCAN_COMPLETED";
  default:
    return "UNKNOWN";
  }
}

String macLastThreeOctets(const String &mac) {
  int i1 = mac.indexOf(':');
  if (i1 < 0)
    return "000000";
  int i2 = mac.indexOf(':', i1 + 1);
  if (i2 < 0)
    return "000000";
  int i3 = mac.indexOf(':', i2 + 1);
  if (i3 < 0)
    return "000000";
  String tail = mac.substring(i3 + 1);
  tail.replace(":", "");
  tail.toUpperCase();
  return tail;
}

String defaultHostnameFromMac(const String &mac) {
  return String(F("C4-VIBRANT-")) + macLastThreeOctets(mac);
}

String defaultSoftApSsidFromMac(const String &mac) {
  return defaultHostnameFromMac(mac);
}

void resetWifiRecoveryState() {
  wifiDisconnectSinceMs = 0;
  wifiRecoveryAttempts = 0;
}

void applyWifiSettings() {
  logStatus(F("Applying Wi-Fi settings..."));
  if (cfg.useCustomMac)
    applyConfiguredMac();
  cfg.wifiPower = constrain(cfg.wifiPower, MIN_WIFI_POWER, MAX_WIFI_POWER);

  const String softApSsid = defaultSoftApSsidFromMac(cfg.mac);

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP_STA);
  WiFi.hostname(cfg.hostname);
  WiFi.setAutoReconnect(true);
  WiFi.setOutputPower(cfg.wifiPower);

  bool apStarted = WiFi.softAP(softApSsid.c_str(), cfg.apPassword.c_str());
  if (!apStarted) {
    restartDevice(F("Failed to start SoftAP with configured credentials."));
  }

  logStatus(String(F("Starting station connection to SSID: ")) + cfg.staSsid);
  WiFi.begin(cfg.staSsid.c_str(), cfg.staPassword.c_str());
  resetWifiRecoveryState();
  lastWifiStatus = WiFi.status();
  logWifiSummary(softApSsid);
}

void maintainWifiConnection() {
  wl_status_t status = WiFi.status();
  if (status != lastWifiStatus) {
    Serial.print(F("[INFO] Wi-Fi status changed: "));
    Serial.print(wifiStatusToString(lastWifiStatus));
    Serial.print(F(" -> "));
    Serial.println(wifiStatusToString(status));
    lastWifiStatus = status;
  }

  if (status == WL_CONNECTED) {
    if (wifiDisconnectSinceMs != 0) {
      logStatus(String(F("Wi-Fi reconnected. Station IP: ")) +
                WiFi.localIP().toString());
    }
    resetWifiRecoveryState();
    return;
  }

  unsigned long now = millis();
  if (wifiDisconnectSinceMs == 0) {
    wifiDisconnectSinceMs = now;
    lastWifiConnectLogMs = 0;
    logWarning(String(F("Wi-Fi disconnected. Status: ")) +
               wifiStatusToString(status));
  }

  if (now - wifiDisconnectSinceMs >= WIFI_RECOVERY_WINDOW_MS &&
      wifiRecoveryAttempts >= MAX_WIFI_RECOVERY_ATTEMPTS) {
    restartDevice(
        F("Wi-Fi could not be recovered within the configured window."));
  }

  if (lastWifiConnectLogMs == 0 ||
      now - lastWifiConnectLogMs >= WIFI_CONNECT_LOG_INTERVAL_MS) {
    Serial.print(F("[INFO] Waiting for Wi-Fi recovery. Status="));
    Serial.print(wifiStatusToString(status));
    Serial.print(F(" Attempts="));
    Serial.println(wifiRecoveryAttempts);
    lastWifiConnectLogMs = now;
  }

  if (now - lastWifiReconnectAttemptMs >= WIFI_RECONNECT_INTERVAL_MS) {
    lastWifiReconnectAttemptMs = now;
    ++wifiRecoveryAttempts;
    Serial.print(F("[INFO] Attempting Wi-Fi reconnect #"));
    Serial.println(wifiRecoveryAttempts);
    WiFi.disconnect(false);
    logStatus(String(F("Reconnecting to SSID: ")) + cfg.staSsid);
    WiFi.begin(cfg.staSsid.c_str(), cfg.staPassword.c_str());
  }
}

} // namespace vibrant
