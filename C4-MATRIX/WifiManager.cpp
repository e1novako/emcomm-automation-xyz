#include "WifiManager.h"
#include "Config.h"
#include "Debug.h"

namespace c4matrix {
String macLastThreeOctets(const String &mac) {
  if (mac.length() < 8)
    return "000000";
  String tail = mac.substring(mac.length() - 8);
  tail.replace(":", "");
  tail.toUpperCase();
  return tail;
}
String defaultHostnameFromMac(const String &mac) {
  return String(F("C4-MATRIX-")) + macLastThreeOctets(mac);
}
String defaultSoftApSsidFromMac(const String &mac) {
  return defaultHostnameFromMac(mac);
}
void applyWifiSettings() {
  logStatus(F("Applying Wi-Fi configuration."));
  if (cfg.useCustomMac)
    applyConfiguredMac();
  else
    cfg.mac = WiFi.softAPmacAddress();
  cfg.wifiPower = constrain(cfg.wifiPower, 5.0f, 20.5f);
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP_STA);
  WiFi.hostname(cfg.hostname);
  WiFi.setAutoReconnect(true);
  WiFi.setOutputPower(cfg.wifiPower);
  String apName = defaultSoftApSsidFromMac(cfg.mac);
  if (!WiFi.softAP(apName.c_str(), cfg.apPassword.c_str()))
    restartDevice(F("Could not start SoftAP."));
  logStatus(String(F("SoftAP started: ")) + apName);
  logStatus(String(F("Connecting to station SSID: ")) + cfg.staSsid);
  WiFi.begin(cfg.staSsid.c_str(), cfg.staPassword.c_str());
  DBG("Wi-Fi power %.1f dBm", cfg.wifiPower);
}
void maintainWifiConnection() {
  static unsigned long lastAttempt = 0;
  if (WiFi.status() == WL_CONNECTED)
    return;
  if (millis() - lastAttempt < 15000UL)
    return;
  lastAttempt = millis();
  logWarning(String(F("Wi-Fi disconnected; reconnecting to ")) + cfg.staSsid);
  WiFi.disconnect(false);
  WiFi.begin(cfg.staSsid.c_str(), cfg.staPassword.c_str());
}
}
