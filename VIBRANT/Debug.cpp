#include "Debug.h"
#include "Config.h"
#include "Outputs.h"
#include "Version.h"
#include "WifiManager.h"
#include <Arduino.h>
#include <ESP8266WiFi.h>

namespace vibrant {

void restartDevice(const String &reason) {
  Serial.println();
  Serial.println(F("[FATAL] Unrecoverable condition encountered."));
  Serial.print(F("[FATAL] Reason: "));
  Serial.println(reason);
  Serial.println(F("[FATAL] Restarting device in 2 seconds..."));
  delay(2000);
  ESP.restart();
}

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

void logWifiSummary(const String &softApSsid) {
  Serial.print(F("[INFO] SoftAP SSID: "));
  Serial.println(softApSsid);
  Serial.print(F("[INFO] Station target SSID: "));
  Serial.println(cfg.staSsid);
  Serial.print(F("[INFO] Hostname: "));
  Serial.println(cfg.hostname);
  Serial.print(F("[INFO] Software version: "));
  Serial.println(SOFTWARE_VERSION);
  Serial.print(F("[INFO] Wi-Fi power: "));
  Serial.print(cfg.wifiPower, 1);
  Serial.println(F(" dBm"));
  Serial.print(F("[INFO] SoftAP IP: "));
  Serial.println(WiFi.softAPIP());
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("[INFO] Station connected. IP: "));
    Serial.println(WiFi.localIP());
  } else {
    Serial.print(F("[INFO] Station status: "));
    Serial.println(wifiStatusToString(WiFi.status()));
  }
}

void logLoadedWifiConfig() {
  Serial.print(F("[INFO] [DIAG] Loaded station SSID: "));
  Serial.println(cfg.staSsid);
  Serial.print(F("[INFO] [DIAG] Loaded Wi-Fi power: "));
  Serial.print(cfg.wifiPower, 1);
  Serial.println(F(" dBm"));
}

void logWifiScan() {
  logStatus(F("[DIAG] Scanning for visible Wi-Fi networks (may take a few "
              "seconds)..."));
  int n = WiFi.scanNetworks();
  if (n <= 0) {
    logStatus(F("[DIAG] No Wi-Fi networks found during scan."));
  } else {
    Serial.print(F("[INFO] [DIAG] Found "));
    Serial.print(n);
    Serial.println(F(" network(s):"));
    for (int i = 0; i < n; ++i) {
      Serial.print(F("[INFO] [DIAG]   SSID: \""));
      Serial.print(WiFi.SSID(i));
      Serial.print(F("\"  RSSI: "));
      Serial.print(WiFi.RSSI(i));
      Serial.println(F(" dBm"));
    }
  }
  WiFi.scanDelete();
}

void logDeviceSummary() {
  Serial.println(F("[INFO] Configured outputs:"));
  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    Serial.print(F("  ["));
    Serial.print(i + 1);
    Serial.print(F("] Model='"));
    Serial.print(cfg.devices[i].model);
    Serial.print(F("' Name='"));
    Serial.print(cfg.devices[i].name);
    Serial.print(F("' Pin="));
    Serial.print(pinLabel(cfg.devices[i].pin));
    Serial.print(F(" State="));
    Serial.println(cfg.devices[i].state ? F("ON") : F("OFF"));
  }
}

} // namespace vibrant
