#pragma once
#include "Config.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
namespace vibrant {
extern const unsigned long MQTT_RECONNECT_INTERVAL_MS;
extern const uint16_t MQTT_PACKET_BUFFER_SIZE, MQTT_PAYLOAD_LOG_MAX_LEN;
extern WiFiClient mqttWifiClient;
extern PubSubClient mqttClient;
extern unsigned long lastMqttConnectAttemptMs;
// Native "vibrant/<hostname>/out/<idx>/..." topics have been removed; only
// the Stickserver protocol is used now. mqttPublishOutputState is kept as a
// no-op so existing call sites do not need to change.
void mqttPublishOutputState(uint8_t);
void mqttCallback(char *, byte *, unsigned int);
const char *mqttStateString(int);
bool mqttDoConnect();
void applyMqttSettings();
void maintainMqtt();
void mqttEnsureConnected();
bool deviceModelMatches(const String &, const String &);
} // namespace vibrant
