#pragma once
#include "Config.h"
#include <ArduinoJson.h>
namespace vibrant {
extern const char STICKSERVER_ROOT_TOPIC[], STICKSERVER_OUTPUT_TYPE[];
extern const uint8_t STICKSERVER_PROTOCOL_VERSION;

// Discovery of other stickserver instances on the MQTT network (used to
// render a fleet-wide output table). Populated passively from hello/list
// responses observed on the stickserver root wildcard subscription.
struct DiscoveredOutputEntry {
  String euid;
  String name;
  bool state = false;
  bool valid = false;
};
struct DiscoveredServer {
  bool active = false;
  String instanceTopic;
  String instanceId;
  String hostname;
  unsigned long lastSeenMs = 0;
  unsigned long lastListRequestMs = 0;
  DiscoveredOutputEntry outputs[MAX_DEVICES];
  uint8_t outputCount = 0;
};
constexpr uint8_t MAX_DISCOVERED_SERVERS = 16;
extern DiscoveredServer discoveredServers[MAX_DISCOVERED_SERVERS];
void maintainStickserverDiscovery();

String stickserverMacToken();
String stickserverIdToken();
String stickserverInstanceId();
String stickserverInstanceTopic();
String stickserverOutputEuid(uint8_t);
int findManagedOutputByEuid(const String &);
void populateStickserverDevice(JsonObject, uint8_t);
void buildStickserverEnvelope(JsonDocument &, const String &, int,
                              const String &, const char *);
bool publishStickserverResponse(JsonDocument &);
void publishStickserverFailure(const String &, int, const String &,
                               const String &, const String &, const String &);
bool extractStringArray(JsonVariantConst, String[], size_t &);
const char *aggregateStickserverStatus(size_t, size_t);
int resolveRequestedEuid(JsonVariantConst, String &);
void handleStickserverMessage(const String &, const String &);
} // namespace vibrant
