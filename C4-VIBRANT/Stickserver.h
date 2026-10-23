#pragma once
#include "Config.h"
#include <ArduinoJson.h>
namespace vibrant {
extern const char STICKSERVER_ROOT_TOPIC[], STICKSERVER_OUTPUT_TYPE[];
extern const uint8_t STICKSERVER_PROTOCOL_VERSION;

// Discovery of other stickserver instances on the MQTT network (used to
// render a fleet-wide output table). Populated passively from hello/list
// responses observed on the stickserver root wildcard subscription.
// Fixed-size char buffers (not String) so each entry is one flat block with
// no per-field heap allocations.
struct DiscoveredOutputEntry {
  char euid[40] = {0};
  char name[24] = {0};
  bool state = false;
  bool valid = false;
};
struct DiscoveredServer {
  bool active = false;
  String instanceTopic;
  String hostname;
  String ipAddress;
  unsigned long lastSeenMs = 0;
  unsigned long lastListRequestMs = 0;
  // Allocated on demand and grown in steps of OUTPUT_GROW_STEP: a table
  // sized for MAX_DEVICES in every slot cost ~7 KB even when most peers
  // have only a few outputs.
  DiscoveredOutputEntry *outputs = nullptr;
  uint8_t outputCount = 0;
  uint8_t outputCapacity = 0;
  DiscoveredServer() = default;
  DiscoveredServer(const DiscoveredServer &) = delete;
  DiscoveredServer &operator=(const DiscoveredServer &) = delete;
  ~DiscoveredServer() { reset(); }
  void reset() {
    delete[] outputs;
    outputs = nullptr;
    outputCount = 0;
    outputCapacity = 0;
    active = false;
    instanceTopic = String();
    hostname = String();
    ipAddress = String();
    lastSeenMs = 0;
    lastListRequestMs = 0;
  }
};
constexpr uint8_t MAX_DISCOVERED_SERVERS = 16;
extern DiscoveredServer discoveredServers[MAX_DISCOVERED_SERVERS];
void maintainStickserverDiscovery();
// Handles this firmware's own hello/list responses straight from the raw MQTT
// payload without copying it to the heap. Returns true if the message was
// consumed.
bool handleStickserverFastPath(const String &topic, const uint8_t *payload,
                               size_t len);

// Fixed-size (no heap allocation) ring buffer of recent per-message heap
// snapshots, recorded in handleStickserverMessage() when cfg.debugSerial is
// enabled. Exposed over HTTP (see WebServer.cpp's
// /settings/diagnostics/heaplog) so the heap-fragmentation trace can be
// inspected without a physical serial connection.
struct HeapTraceEntry {
  char topic[28] = {0};
  uint16_t payloadLen = 0;
  uint32_t freeBefore = 0;
  uint32_t blockBefore = 0;
  uint32_t freeAfterParse = 0;
  uint32_t blockAfterParse = 0;
  int32_t freeAfterDiscovery = -1; // -1 = not a discovery-response message
  uint32_t blockAfterDiscovery = 0;
  uint32_t atMs = 0;
};
constexpr uint8_t HEAP_TRACE_CAPACITY = 16;
extern HeapTraceEntry heapTraceLog[HEAP_TRACE_CAPACITY];
extern uint8_t heapTraceHead;
extern uint8_t heapTraceCount;
String renderHeapTraceJson();

String stickserverMacToken();
String stickserverIdToken();
String stickserverInstanceId();
const String &stickserverInstanceTopic();
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
