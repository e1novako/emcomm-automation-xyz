#include "MqttClient.h"
#include "Actions.h"
#include "Config.h"
#include "Debug.h"
#include "Outputs.h"
#include "Stickserver.h"
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <string.h>

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

static const size_t MQTT_MIN_HEADROOM_BYTES = 768;

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  if (topic == nullptr || payload == nullptr)
    return;
  if (length == 0) {
    logWarning(F("MQTT: dropped empty payload."));
    return;
  }
  // Response messages (including this device's own hello/list responses,
  // which the broker echoes back on our instance topic) are only useful for
  // discovery. Skip them before any allocation when discovery is off; they
  // can be ~1.7 KB each and previously exhausted the heap. Envelopes start
  // with {"rsp":..., so a short prefix scan is enough.
  if (!cfg.stickserverQueryEnabled && !cfg.stickserverPassiveDiscoveryEnabled) {
    static const char kRsp[] = "\"rsp\"";
    size_t scan = length < 40 ? length : 40;
    for (size_t i = 0; i + 5 <= scan; ++i) {
      if (memcmp(payload + i, kRsp, 5) == 0)
        return;
    }
  }
  String topicStr(topic);
  if (topicStr.isEmpty()) {
    logWarning(F("MQTT: dropped message with empty topic."));
    return;
  }
  if (cfg.debugSerial) {
    // Truncate into a fixed stack buffer rather than building a String: this
    // log line runs on the hottest path in the firmware (every MQTT message,
    // including the ~14-16-message hello/list bursts).
    char logBuf[MQTT_PAYLOAD_LOG_MAX_LEN + 1];
    size_t copyLen = length < MQTT_PAYLOAD_LOG_MAX_LEN
                         ? length
                         : MQTT_PAYLOAD_LOG_MAX_LEN;
    memcpy(logBuf, payload, copyLen);
    logBuf[copyLen] = '\0';
    DBG("[MQTT] Received -> topic: %s payload[%u]: %s%s", topicStr.c_str(),
        length, logBuf, length > MQTT_PAYLOAD_LOG_MAX_LEN ? "...(truncated)" : "");
  }

  // Hello/list responses (the bulk of bus traffic) are consumed directly from
  // the MQTT buffer, so the ~1.7 KB payload is never copied to the heap.
  if (handleStickserverFastPath(topicStr, payload, length))
    return;

  // MQTT payload bytes are not null-terminated; copy to String using explicit
  // length. reserve() pre-allocates so concat does not reallocate; failure
  // means low memory.
  // Drop before allocating when the largest free block cannot hold the
  // payload plus headroom for parsing; the warning is rate-limited so a
  // burst of dropped messages cannot flood the serial log and starve the
  // watchdog.
  static unsigned long lastDropWarnMs = 0;
  if (ESP.getMaxFreeBlockSize() < length + MQTT_MIN_HEADROOM_BYTES) {
    unsigned long now = millis();
    if (now - lastDropWarnMs >= 5000) {
      lastDropWarnMs = now;
      logWarning(String(F("MQTT: dropped message -- low heap (payload ")) +
                 String(length) + F(" B, max free block ") +
                 String(ESP.getMaxFreeBlockSize()) + F(" B)."));
    }
    return;
  }
  String payloadStr;
  payloadStr.reserve(length);
  if (!payloadStr.concat(reinterpret_cast<const char *>(payload), length)) {
    unsigned long now = millis();
    if (now - lastDropWarnMs >= 5000) {
      lastDropWarnMs = now;
      logWarning(F("MQTT: dropped message -- payload allocation failed."));
    }
    return;
  }

  // Subscribed via root topic + wildcard (root/#), so this also receives
  // other stickserver instances' requests/responses, used for discovery.
  // Checked via raw prefix comparison (strncmp), not
  // topicStr.startsWith(String(STICKSERVER_ROOT_TOPIC) + '/'), which would
  // heap-allocate two temporary Strings on every single message -- this
  // runs unconditionally (not just under debugSerial) for all bus traffic,
  // making it a prime fragmentation source during hello/list bursts.
  size_t rootLen = strlen(STICKSERVER_ROOT_TOPIC);
  bool isRootSubtopic = topicStr.length() > rootLen &&
                       strncmp(topicStr.c_str(), STICKSERVER_ROOT_TOPIC,
                               rootLen) == 0 &&
                       topicStr[rootLen] == '/';
  if (topicStr == STICKSERVER_ROOT_TOPIC || isRootSubtopic) {
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

// Ensures the PubSubClient internal TX/RX buffer is large enough for our
// JSON payloads (hello/list responses can exceed PubSubClient's 256-byte
// library default). setBufferSize() reallocates on the heap and can
// silently fail under memory pressure, leaving the client at its previous
// (too-small) buffer size -- which truncates large payloads and shows up on
// the receiving end as "invalid_json"/"IncompleteInput" errors. We only
// call it once the first time (or again if it did not take effect), rather
// than on every reconnect attempt, to avoid needless realloc churn that
// contributes to heap fragmentation.
void ensureMqttBufferSize() {
  if (mqttClient.getBufferSize() == MQTT_PACKET_BUFFER_SIZE)
    return;
  if (!mqttClient.setBufferSize(MQTT_PACKET_BUFFER_SIZE)) {
    Serial.print(F("[WARN] [MQTT] setBufferSize("));
    Serial.print(MQTT_PACKET_BUFFER_SIZE);
    Serial.print(F(") failed -- low heap? current buffer size: "));
    Serial.println(mqttClient.getBufferSize());
  }
}

bool mqttDoConnect() {
  if (!cfg.mqttEnabled || cfg.mqttHost.isEmpty())
    return false;
  ensureMqttBufferSize();
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
  // Subscribe to the stickserver root topic (broadcast commands, e.g. hello)
  // plus a wildcard on everything beneath it so this device also observes
  // other stickserver instances' requests/responses for discovery purposes.
  DBG("[MQTT] Subscribing -> %s", STICKSERVER_ROOT_TOPIC);
  mqttClient.subscribe(STICKSERVER_ROOT_TOPIC);
  // The wildcard delivers every peer's traffic (~1.7 KB hello/list bursts),
  // which exhausts the heap; it is only needed for discovery. Otherwise
  // subscribe to just our own instance topic.
  if (cfg.stickserverQueryEnabled || cfg.stickserverPassiveDiscoveryEnabled) {
    String wildcardTopic = String(STICKSERVER_ROOT_TOPIC) + F("/#");
    DBG("[MQTT] Subscribing -> %s", wildcardTopic.c_str());
    mqttClient.subscribe(wildcardTopic.c_str());
  } else {
    DBG("[MQTT] Subscribing -> %s", stickserverInstanceTopic().c_str());
    mqttClient.subscribe(stickserverInstanceTopic().c_str());
  }
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
  ensureMqttBufferSize();
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
