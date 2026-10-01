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
String mqttOutputStateTopic(uint8_t);
String mqttOutputSetTopic(uint8_t);
String mqttOutputActionTopic(uint8_t);
void mqttPublishOutputState(uint8_t);
void mqttPublishAllOutputStates();
void mqttCallback(char *, byte *, unsigned int);
const char *mqttStateString(int);
bool mqttDoConnect();
void applyMqttSettings();
void maintainMqtt();
void mqttEnsureConnected();
bool deviceModelMatches(const String &, const String &);
} // namespace vibrant
