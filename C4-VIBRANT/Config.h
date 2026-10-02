#pragma once
#include <Arduino.h>
namespace vibrant {
enum : uint8_t { MAX_DEVICES = 16 };
struct DeviceEntry {
  String manufacturer, model, name;
  int8_t pin;
  bool state;
};
struct DeviceConfig {
  String mac;
  bool useCustomMac;
  String hostname, staSsid, staPassword, apPassword;
  float wifiPower;
  uint8_t numOutputs;
  DeviceEntry devices[MAX_DEVICES];
  bool mqttEnabled;
  String mqttHost;
  uint16_t mqttPort;
  String mqttUser, mqttPassword;
  bool arduinoOtaEnabled, debugSerial;
  // Stickserver fleet discovery (hello/list broadcasts + responding to
  // peers' discovery requests). Disabled by default: discovery is the
  // source of ongoing MQTT broadcast traffic across the fleet, and most
  // deployments don't need the "All Outputs" multi-device view.
  bool stickserverDiscoveryEnabled;
};
struct PinMapping {
  int8_t gpio;
  const char *label;
};
extern const char CONFIG_PATH[], IMPORT_CONFIG_PATH[], DEFAULT_STA_SSID[],
    DEFAULT_STA_PASSWORD[], DEFAULT_AP_PASSWORD[],
    DEFAULT_DEVICE_MANUFACTURER[], DEFAULT_DEVICE_MODEL[];
extern const uint8_t DEFAULT_NUM_OUTPUTS;
extern const float MIN_WIFI_POWER, MAX_WIFI_POWER;
extern const unsigned long OUTPUT_BOOT_ACTIVATION_DELAY_MS;
extern const uint16_t DEFAULT_MQTT_PORT;
extern const PinMapping OUTPUT_PIN_MAPPINGS[];
extern const size_t OUTPUT_PIN_MAPPING_COUNT;
extern DeviceConfig cfg;
void setFactoryDefaults();
bool saveConfig();
bool loadConfig();
bool parseMac(const String &, uint8_t out[6]);
bool applyConfiguredMac();
void performFactoryResetAndRestart(const String &);
} // namespace vibrant
