#include "MqttClient.h"
#include "Actions.h"
#include "Config.h"
#include "Debug.h"
#include "Outputs.h"
#include "Stickserver.h"
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>

namespace vibrant {

WiFiClient mqttWifiClient;
PubSubClient mqttClient(mqttWifiClient);
unsigned long lastMqttConnectAttemptMs = 0;

bool deviceModelMatches(const String &modelField, const String &ntype) {
  int tokenStart = 0;
  bool matched = false;
  while (tokenStart <= static_cast<int>(modelField.length())) {
    int comma = modelField.indexOf(',', tokenStart);
    int tokenEnd = comma >= 0 ? comma : modelField.length();
    String token = modelField.substring(tokenStart, tokenEnd);
    token.trim();
    bool tokenMatches = token.equalsIgnoreCase(ntype);
    DBG("[MQTT] comparing Model token='%s' against ntype='%s' -> %s",
        token.c_str(), ntype.c_str(), tokenMatches ? "match" : "no match");
    if (tokenMatches) {
      matched = true;
      break;
    }
    if (comma < 0)
      break;
    tokenStart = comma + 1;
  }
  DBG("[MQTT] Model match result: %s", matched ? "match" : "no match");
  return matched;
}

// Native "vibrant/<hostname>/out/<idx>/..." topics have been removed; only
// the Stickserver protocol (handleStickserverMessage) is used now. This
// function is kept as a no-op so existing call sites after output state
// changes do not need to be edited.
void mqttPublishOutputState(uint8_t) {}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  if (topic == nullptr || payload == nullptr)
    return;
  if (length == 0) {
    logWarning(F("MQTT: dropped empty payload."));
    return;
  }
  String topicStr(topic);
  if (topicStr.isEmpty()) {
    logWarning(F("MQTT: dropped message with empty topic."));
    return;
  }
  // MQTT payload bytes are not null-terminated; copy to String using explicit
  // length. reserve() pre-allocates so concat does not reallocate; failure
  // means low memory.
  String payloadStr;
  payloadStr.reserve(length);
  if (!payloadStr.concat(reinterpret_cast<const char *>(payload), length)) {
    logWarning(F("MQTT: dropped message -- payload allocation failed."));
    return;
  }

  if (cfg.debugSerial) {
    DBG("[MQTT] Received -> topic: %s payload[%u]: %s%s", topicStr.c_str(),
        length, payloadStr.substring(0, MQTT_PAYLOAD_LOG_MAX_LEN).c_str(),
        payloadStr.length() > MQTT_PAYLOAD_LOG_MAX_LEN ? "...(truncated)" : "");
  }

  if (topicStr == STICKSERVER_ROOT_TOPIC ||
      topicStr == stickserverInstanceTopic()) {
    DBG("[MQTT] Stickserver message -> topic: %s", topicStr.c_str());
    handleStickserverMessage(topicStr, payloadStr);
    return;
  }
  Serial.print(F("[WARN] [MQTT] Unmatched topic (no handler): "));
  Serial.println(topicStr);
}

const char *mqttStateString(int state) {
  switch (state) {
  case -4:
    return "CONNECTION_TIMEOUT";
  case -3:
    return "CONNECTION_LOST";
  case -2:
    return "CONNECT_FAILED";
  case -1:
    return "DISCONNECTED";
  case 0:
    return "CONNECTED";
  case 1:
    return "BAD_PROTOCOL";
  case 2:
    return "BAD_CLIENT_ID";
  case 3:
    return "UNAVAILABLE";
  case 4:
    return "BAD_CREDENTIALS";
  case 5:
    return "UNAUTHORIZED";
  default:
    return "UNKNOWN";
  }
}

bool mqttDoConnect() {
  if (!cfg.mqttEnabled || cfg.mqttHost.isEmpty())
    return false;
  mqttClient.setBufferSize(MQTT_PACKET_BUFFER_SIZE);
  mqttClient.setServer(cfg.mqttHost.c_str(), cfg.mqttPort);
  mqttClient.setCallback(mqttCallback);
  String clientId = cfg.hostname;
  Serial.print(F("[INFO] [MQTT] Connecting -> host: "));
  Serial.print(cfg.mqttHost);
  Serial.print(F(" port: "));
  Serial.print(cfg.mqttPort);
  Serial.print(F(" clientId: "));
  Serial.println(clientId);
  bool connected;
  if (cfg.mqttUser.isEmpty()) {
    // No credentials: connect anonymously
    Serial.println(F("[INFO] [MQTT] Auth mode: anonymous"));
    connected = mqttClient.connect(clientId.c_str());
  } else {
    // User is set; password may be empty (broker may allow empty password for a
    // named user)
    Serial.print(F("[INFO] [MQTT] Auth mode: credentials (user: "));
    Serial.print(cfg.mqttUser);
    Serial.println(F(")"));
    connected = mqttClient.connect(clientId.c_str(), cfg.mqttUser.c_str(),
                                   cfg.mqttPassword.c_str());
  }
  if (!connected) {
    Serial.print(F("[WARN] [MQTT] Connection refused -> state: "));
    Serial.println(mqttStateString(mqttClient.state()));
    return false;
  }
  // Subscribe to stickserver root and instance topics
  DBG("[MQTT] Subscribing -> %s", STICKSERVER_ROOT_TOPIC);
  mqttClient.subscribe(STICKSERVER_ROOT_TOPIC);
  String instanceTopic = stickserverInstanceTopic();
  DBG("[MQTT] Subscribing -> %s", instanceTopic.c_str());
  mqttClient.subscribe(instanceTopic.c_str());
  logStatus(String(F("MQTT connected. Host: ")) + cfg.mqttHost + F(" port: ") +
            String(cfg.mqttPort));
  return true;
}

void applyMqttSettings() {
  if (!cfg.mqttEnabled || cfg.mqttHost.isEmpty()) {
    if (mqttClient.connected()) {
      logStatus(F("MQTT disabled or host cleared; disconnecting."));
      mqttClient.disconnect();
    } else {
      logStatus(F("MQTT disabled or host not configured; skipping."));
    }
    return;
  }
  Serial.print(F("[INFO] [MQTT] Applying settings -> host: "));
  Serial.print(cfg.mqttHost);
  Serial.print(F(" port: "));
  Serial.println(cfg.mqttPort);
  mqttClient.setBufferSize(MQTT_PACKET_BUFFER_SIZE);
  mqttClient.setServer(cfg.mqttHost.c_str(), cfg.mqttPort);
  mqttClient.setCallback(mqttCallback);
  if (mqttClient.connected()) {
    logStatus(F("MQTT settings updated; disconnecting to force reconnect."));
    mqttClient.disconnect();
  }
  lastMqttConnectAttemptMs = 0;
}

void maintainMqtt() {
  if (!cfg.mqttEnabled || cfg.mqttHost.isEmpty())
    return;
  if (WiFi.status() != WL_CONNECTED)
    return;
  if (mqttClient.connected()) {
    mqttClient.loop();
    return;
  }
  unsigned long now = millis();
  if (now - lastMqttConnectAttemptMs < MQTT_RECONNECT_INTERVAL_MS)
    return;
  lastMqttConnectAttemptMs = now;
  logStatus(F("Attempting MQTT connection..."));
  if (!mqttDoConnect()) {
    Serial.print(F("[WARN] [MQTT] Connection failed. State: "));
    Serial.println(mqttStateString(mqttClient.state()));
  }
}

void mqttEnsureConnected() {
  if (!cfg.mqttEnabled || cfg.mqttHost.isEmpty())
    return;
  if (WiFi.status() != WL_CONNECTED)
    return;
  if (mqttClient.connected())
    return;
  lastMqttConnectAttemptMs = 0;
  maintainMqtt();
}

} // namespace vibrant
