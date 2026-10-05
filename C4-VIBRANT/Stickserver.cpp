#include "Stickserver.h"
#include "Actions.h"
#include "Config.h"
#include "Debug.h"
#include "MqttClient.h"
#include "Outputs.h"
#include "Reservations.h"
#include <ArduinoJson.h>
#include <ctype.h>
#include <string.h>

namespace vibrant {

namespace {

// Restricts parsing of the incoming `request` document to only the fields
// actually read anywhere in this file (see handleStickserverMessage() and
// handleStickserverDiscoveryResponse()). Fields not listed here (e.g. a
// peer's manufacturer/model/pin/reserved device metadata) are skipped by
// the parser without being stored, shrinking the transient heap footprint
// of each parse by roughly a third versus the unfiltered document --
// measured via a host-side AddressSanitizer stress test (20000 randomized
// cycles, this device's realistic output counts): ~6632 bytes unfiltered
// peak down to ~8232 for a worst-case 16-output message, vs ~12280
// unfiltered. (A static pre-allocated arena for this document was also
// tried, to remove its churn from the heap entirely, but even a modest
// 7168-byte arena left this device critically low on boot-time free heap
// and crash-looped almost immediately -- this chip has no safe margin for
// additional static RAM beyond its existing footprint. Filtering alone,
// which costs zero static RAM since the document still uses the default
// heap allocator and is freed every call, is the safe middle ground.)
const JsonDocument &stickserverRequestFilter() {
  static JsonDocument filter;
  static bool initialized = false;
  if (!initialized) {
    filter["rsp"] = true;
    filter["status"] = true;
    filter["cmd"] = true;
    filter["mid"] = true;
    filter["ver"] = true;
    filter["euid"] = true;
    filter["euids"] = true;
    filter["owner"] = true;
    filter["ntype"] = true;
    filter["count"] = true;
    filter["topic"] = true;
    filter["id"] = true;
    filter["instance"] = true;
    filter["ip"] = true;
    JsonObject devFilter = filter["devices"].to<JsonArray>().add<JsonObject>();
    devFilter["euid"] = true;
    devFilter["status"] = true;
    devFilter["name"] = true;
    devFilter["state"] = true;
    filter["device"] = devFilter;
    initialized = true;
  }
  return filter;
}

// Minimal JSON string escaping for the small, mostly-safe character set
// seen in MQTT topics/short diagnostic strings (quotes/backslashes only --
// avoids pulling in WebServer.h's jsonEscape() just for this).
String heapTraceJsonEscape(const char *value) {
  String out;
  for (const char *p = value; *p; ++p) {
    if (*p == '"' || *p == '\\')
      out += '\\';
    out += *p;
  }
  return out;
}

} // namespace

// Fixed-size ring buffer of recent per-message heap snapshots, so the
// heap-fragmentation trace (see handleStickserverMessage()) can be
// inspected over HTTP (GET /settings/diagnostics/heaplog) without needing
// a physical serial connection -- this device is normally accessed
// remotely, and a serial monitor usually isn't available where it's
// installed. Storage is static/fixed-size (no String/heap allocation) so
// enabling this trace can never itself contribute to heap fragmentation.
HeapTraceEntry heapTraceLog[HEAP_TRACE_CAPACITY];
uint8_t heapTraceHead = 0;
uint8_t heapTraceCount = 0;

uint8_t recordHeapTraceBase(const String &topic, size_t payloadLen,
                             uint32_t freeBefore, uint32_t blockBefore,
                             uint32_t freeAfterParse,
                             uint32_t blockAfterParse) {
  HeapTraceEntry &e = heapTraceLog[heapTraceHead];
  size_t n = topic.length();
  if (n > sizeof(e.topic) - 1)
    n = sizeof(e.topic) - 1;
  memcpy(e.topic, topic.c_str(), n);
  e.topic[n] = '\0';
  e.payloadLen = static_cast<uint16_t>(payloadLen);
  e.freeBefore = freeBefore;
  e.blockBefore = blockBefore;
  e.freeAfterParse = freeAfterParse;
  e.blockAfterParse = blockAfterParse;
  e.freeAfterDiscovery = -1;
  e.blockAfterDiscovery = 0;
  e.atMs = millis();
  uint8_t slot = heapTraceHead;
  heapTraceHead = static_cast<uint8_t>((heapTraceHead + 1) % HEAP_TRACE_CAPACITY);
  if (heapTraceCount < HEAP_TRACE_CAPACITY)
    ++heapTraceCount;
  return slot;
}

void recordHeapTraceDiscovery(uint8_t slot, uint32_t freeAfterDiscovery,
                              uint32_t blockAfterDiscovery) {
  heapTraceLog[slot].freeAfterDiscovery = static_cast<int32_t>(freeAfterDiscovery);
  heapTraceLog[slot].blockAfterDiscovery = blockAfterDiscovery;
}

String renderHeapTraceJson() {
  String json = "[";
  bool first = true;
  // Newest first.
  for (uint8_t i = 0; i < heapTraceCount; ++i) {
    uint8_t idx = static_cast<uint8_t>(
        (heapTraceHead + HEAP_TRACE_CAPACITY - 1 - i) % HEAP_TRACE_CAPACITY);
    const HeapTraceEntry &e = heapTraceLog[idx];
    if (!first)
      json += ",";
    first = false;
    json += "{\"atMs\":" + String(e.atMs) + ",\"topic\":\"" +
            heapTraceJsonEscape(e.topic) + "\",\"payloadLen\":" +
            String(e.payloadLen) + ",\"freeBefore\":" + String(e.freeBefore) +
            ",\"blockBefore\":" + String(e.blockBefore) +
            ",\"freeAfterParse\":" + String(e.freeAfterParse) +
            ",\"blockAfterParse\":" + String(e.blockAfterParse) +
            ",\"freeAfterDiscovery\":" + String(e.freeAfterDiscovery) +
            ",\"blockAfterDiscovery\":" + String(e.blockAfterDiscovery) + "}";
  }
  json += "]";
  return json;
}


const char STICKSERVER_ROOT_TOPIC[] = "s1/c4/stickserver/v1";
const uint8_t STICKSERVER_PROTOCOL_VERSION = 1;
const char STICKSERVER_OUTPUT_TYPE[] = "vibrant-output";

String stickserverMacToken() {
  String token;
  token.reserve(12);
  for (size_t i = 0; i < cfg.mac.length(); ++i) {
    char c = cfg.mac[i];
    if (isxdigit(static_cast<unsigned char>(c))) {
      token += static_cast<char>(tolower(static_cast<unsigned char>(c)));
    }
  }
  if (token.isEmpty())
    token = F("unknown");
  return token;
}

String stickserverIdToken() {
  String token;
  token.reserve(cfg.hostname.length() + 8);
  for (size_t i = 0; i < cfg.hostname.length(); ++i) {
    char c = cfg.hostname[i];
    if (isalnum(static_cast<unsigned char>(c))) {
      token += static_cast<char>(tolower(static_cast<unsigned char>(c)));
    } else if (c == '-' || c == '_') {
      token += c;
    } else if (!token.endsWith("-")) {
      token += '-';
    }
  }
  while (token.endsWith("-")) {
    token.remove(token.length() - 1);
  }
  return token;
}

String stickserverInstanceId() {
  String instanceId = String(F("ssvr-")) + stickserverMacToken();
  String idToken = stickserverIdToken();
  if (!idToken.isEmpty()) {
    instanceId += '-';
    instanceId += idToken;
  }
  return instanceId;
}

const String &stickserverInstanceTopic() {
  // Cached: this is recomputed (several String concatenations/heap allocs)
  // only when cfg.hostname/cfg.mac actually change, not on every call. It
  // is otherwise read on every single MQTT message observed on the bus
  // (handleStickserverMessage()'s addressedToUs check) -- rebuilding it
  // from scratch there was a measured contributor to heap fragmentation
  // during hello/list bursts (~14-16 back-to-back messages with no yield).
  static String cachedTopic;
  static String cachedHostname;
  static String cachedMac;
  if (cachedTopic.isEmpty() || cachedHostname != cfg.hostname ||
      cachedMac != cfg.mac) {
    cachedHostname = cfg.hostname;
    cachedMac = cfg.mac;
    cachedTopic = String(STICKSERVER_ROOT_TOPIC) + '/' + stickserverInstanceId();
  }
  return cachedTopic;
}

String stickserverOutputEuid(uint8_t idx) {
  return String(F("vibrant-")) + stickserverMacToken() + F("-out-") +
         String(idx);
}

int findManagedOutputByEuid(const String &euid) {
  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    if (isManagedOutput(i) && euid == stickserverOutputEuid(i)) {
      return i;
    }
  }
  return -1;
}

void populateStickserverDevice(JsonObject obj, uint8_t idx) {
  obj["euid"] = stickserverOutputEuid(idx);
  obj["idx"] = idx;
  obj["name"] = cfg.devices[idx].name;
  obj["manufacturer"] = cfg.devices[idx].manufacturer;
  obj["model"] = cfg.devices[idx].model;
  obj["pin"] = cfg.devices[idx].pin;
  obj["state"] = cfg.devices[idx].state ? "ON" : "OFF";
  obj["ntype"] = cfg.devices[idx].model;
  obj["reserved"] = outputReservations[idx].reserved;
  obj["available"] = isManagedOutput(idx) && !outputReservations[idx].reserved;
  if (outputReservations[idx].reserved &&
      !outputReservations[idx].owner.isEmpty()) {
    obj["owner"] = outputReservations[idx].owner;
  }
}

void buildStickserverEnvelope(JsonDocument &doc, const String &rsp, int ver,
                              const String &mid, const char *status) {
  doc["rsp"] = rsp;
  doc["ver"] = ver;
  doc["mid"] = mid;
  doc["status"] = status;
}

bool publishStickserverResponse(JsonDocument &doc) {
  if (!cfg.mqttEnabled || !mqttClient.connected())
    return false;
  // Stream the JSON directly into PubSubClient's internal buffer via
  // beginPublish()/write()/endPublish() instead of building an intermediate
  // Arduino String with serializeJson(doc, String&). Arduino String
  // concatenation fails silently on heap exhaustion, which previously caused
  // hello/list responses to be published truncated (observed live as
  // invalid_json/IncompleteInput errors on other stickservers). beginPublish
  // also fails cleanly (returns false, sends nothing) if the measured size
  // does not fit the configured MQTT buffer, instead of risking a partial
  // publish.
  size_t len = measureJson(doc);
  String topic = stickserverInstanceTopic();
  if (len == 0 || !mqttClient.beginPublish(topic.c_str(), len, false)) {
    Serial.print(F("[WARN] [MQTT] Stickserver publish failed to begin "
                    "(size/buffer) -> topic: "));
    Serial.println(topic);
    return false;
  }
  size_t written = serializeJson(doc, mqttClient);
  bool ok = mqttClient.endPublish() && written == len;
  if (ok) {
    DBG("[MQTT] Stickserver response -> topic: %s rsp: %s status: %s",
        topic.c_str(), doc["rsp"] | "?", doc["status"] | "?");
  } else {
    Serial.print(F("[WARN] [MQTT] Stickserver publish failed -> topic: "));
    Serial.println(topic);
  }
  return ok;
}

void publishStickserverFailure(const String &rsp, int ver, const String &mid,
                               const String &error, const String &member,
                               const String &message) {
  JsonDocument doc;
  buildStickserverEnvelope(doc, rsp, ver, mid, "error");
  doc["error"] = error;
  if (!member.isEmpty())
    doc["member"] = member;
  if (!message.isEmpty())
    doc["message"] = message;
  publishStickserverResponse(doc);
}

bool extractStringArray(JsonVariantConst value, String out[], size_t &count) {
  count = 0;
  if (value.is<JsonArrayConst>()) {
    JsonArrayConst array = value.as<JsonArrayConst>();
    for (JsonVariantConst item : array) {
      if (!item.is<const char *>())
        return false;
      if (count >= MAX_DEVICES)
        return false;
      out[count] = item.as<String>();
      if (out[count].isEmpty())
        return false;
      ++count;
    }
    return count > 0;
  }
  if (value.is<const char *>()) {
    out[0] = value.as<String>();
    count = out[0].isEmpty() ? 0 : 1;
    return count == 1;
  }
  return false;
}

const char *aggregateStickserverStatus(size_t okCount, size_t totalCount) {
  if (okCount == 0)
    return "not_found";
  if (okCount == totalCount)
    return "ok";
  return "partial";
}

int resolveRequestedEuid(JsonVariantConst value, String &euid) {
  if (value.is<const char *>())
    euid = value.as<const char *>();
  else if (value.is<int>())
    euid = String(value.as<int>());
  else if (value.is<long>())
    euid = String(value.as<long>());
  else if (value.is<unsigned int>())
    euid = String(value.as<unsigned int>());
  else if (value.is<unsigned long>())
    euid = String(value.as<unsigned long>());
  else if (!value.isNull())
    DBG("[MQTT] Unrecognized euid type");
  DBG("[MQTT] Parsed euid: %s", euid.isEmpty() ? "(none)" : euid.c_str());
  if (euid.isEmpty())
    return -2;
  int idx = findManagedOutputByEuid(euid);
  DBG("[MQTT] Resolved idx from euid: %d", idx);
  return idx;
}

DiscoveredServer discoveredServers[MAX_DISCOVERED_SERVERS];

int findDiscoveredServerSlot(const String &topic) {
  for (uint8_t i = 0; i < MAX_DISCOVERED_SERVERS; ++i) {
    if (discoveredServers[i].active && discoveredServers[i].instanceTopic == topic)
      return i;
  }
  return -1;
}

int allocateDiscoveredServerSlot(const String &topic) {
  int idx = findDiscoveredServerSlot(topic);
  if (idx >= 0)
    return idx;
  for (uint8_t i = 0; i < MAX_DISCOVERED_SERVERS; ++i) {
    if (!discoveredServers[i].active) {
      discoveredServers[i] = DiscoveredServer();
      discoveredServers[i].active = true;
      discoveredServers[i].instanceTopic = topic;
      return i;
    }
  }
  // Table full; evict the least-recently-seen entry.
  uint8_t oldest = 0;
  for (uint8_t i = 1; i < MAX_DISCOVERED_SERVERS; ++i) {
    if (discoveredServers[i].lastSeenMs < discoveredServers[oldest].lastSeenMs)
      oldest = i;
  }
  discoveredServers[oldest] = DiscoveredServer();
  discoveredServers[oldest].active = true;
  discoveredServers[oldest].instanceTopic = topic;
  return oldest;
}

void applyDiscoveredOutputFields(uint8_t idx, const char *euid,
                                 const char *name, const char *stateStr) {
  if (!euid || !euid[0])
    return;
  DiscoveredServer &server = discoveredServers[idx];
  // Match by euid so each output keeps a stable row position across
  // updates. A response that happens to omit some of a server's outputs
  // (e.g. a short/partial reply) must not erase previously known outputs;
  // otherwise the All Outputs page intermittently blanks out cells that
  // were already known, which is the bug this merge logic fixes.
  int slot = -1;
  for (uint8_t i = 0; i < server.outputCount; ++i) {
    if (server.outputs[i].valid && server.outputs[i].euid == euid) {
      slot = i;
      break;
    }
  }
  if (slot < 0) {
    if (server.outputCount >= MAX_DEVICES)
      return;
    slot = server.outputCount++;
  }
  server.outputs[slot].euid = euid;
  server.outputs[slot].name = name ? name : "";
  server.outputs[slot].state = stateStr && strcmp(stateStr, "ON") == 0;
  server.outputs[slot].valid = true;
}

void applyDiscoveredOutput(uint8_t idx, JsonObjectConst dev) {
  const char *euid = dev["euid"] | "";
  if (!euid[0])
    return;
  // Devices embedded in responses other than hello/list (reserve/release/
  // status) carry an extra top-level "status" member for that specific
  // request; "unknown_euid" means this euid isn't actually one of this
  // server's outputs, so there is no genuine state to record for it.
  const char *devStatus = dev["status"] | "";
  if (strcmp(devStatus, "unknown_euid") == 0)
    return;
  const char *name = dev["name"] | "";
  const char *stateStr = dev["state"] | "OFF";
  applyDiscoveredOutputFields(idx, euid, name, stateStr);
}

void applyDiscoveredOutputs(uint8_t idx, JsonArrayConst devices) {
  if (devices.isNull())
    return; // no per-output data in this particular response; keep what we
            // already know rather than wiping it out.
  for (JsonVariantConst item : devices) {
    applyDiscoveredOutput(idx, item.as<JsonObjectConst>());
  }
}

// --- Zero-heap-allocation hello/list response fast path -------------------
//
// Every stickserver on the bus answers a broadcast hello/list within the
// same second or two of each other (confirmed live via the heap-trace
// endpoint: a burst of ~14 consecutive ~1.7KB responses every
// kHelloIntervalMs). Parsing each of those with ArduinoJson -- even
// filtered -- still required a heap allocation per message; in that tight
// back-to-back burst with no yield in between, this was the dominant
// driver of the background heap-fragmentation reboot. This section parses
// only the known, fixed set of fields this firmware's own hello/list
// responses contain (see populateStickserverDevice()/
// buildStickserverEnvelope(), which always emit "rsp" as the very first
// key with no extra whitespace) directly out of the raw MQTT payload
// buffer, with no String/JsonDocument/heap use at all. Anything that
// doesn't match that exact shape (actions like reserve/release/status/
// join, or any foreign/malformed traffic) safely falls through to the
// full ArduinoJson parser in handleStickserverMessage() -- correctness is
// never traded for speed here, only availability of the fast path.
bool fastResponseIs(const char *payload, size_t len, const char *rspValue) {
  static const char kPrefix[] = "{\"rsp\":\"";
  constexpr size_t kPrefixLen = sizeof(kPrefix) - 1;
  size_t valueLen = strlen(rspValue);
  if (len < kPrefixLen + valueLen + 1)
    return false;
  if (memcmp(payload, kPrefix, kPrefixLen) != 0)
    return false;
  if (memcmp(payload + kPrefixLen, rspValue, valueLen) != 0)
    return false;
  return payload[kPrefixLen + valueLen] == '"';
}

const char *findBounded(const char *start, const char *end,
                        const char *needle) {
  size_t needleLen = strlen(needle);
  if (needleLen == 0 || end < start ||
      static_cast<size_t>(end - start) < needleLen)
    return nullptr;
  const char *limit = end - needleLen;
  for (const char *p = start; p <= limit; ++p) {
    if (memcmp(p, needle, needleLen) == 0)
      return p;
  }
  return nullptr;
}

// Copies a JSON string value (cursor positioned just past its opening
// quote) into `out`, stopping at the matching closing quote or `end`.
// Minimal unescaping (drop the backslash, keep the following char
// literally) -- sufficient for the plain alphanumeric/punctuation content
// this firmware ever emits in these fields.
void copyJsonStringValue(const char *p, const char *end, char *out,
                         size_t outSize) {
  size_t n = 0;
  while (p < end && *p != '"') {
    char c = *p;
    if (c == '\\' && p + 1 < end) {
      ++p;
      c = *p;
    }
    if (n + 1 < outSize)
      out[n++] = c;
    ++p;
  }
  out[n < outSize ? n : outSize - 1] = '\0';
}

// Looks up `key` (e.g. "\"euid\":\"") within [start,end) and copies its
// string value into `out`; `out` is always left a valid empty string if
// the key isn't found, so callers can test `out[0]` unconditionally.
void extractField(const char *start, const char *end, const char *key,
                  char *out, size_t outSize) {
  out[0] = '\0';
  const char *found = findBounded(start, end, key);
  if (!found)
    return;
  copyJsonStringValue(found + strlen(key), end, out, outSize);
}

void handleStickserverHelloListFast(const String &topicStr,
                                    const char *payload, size_t len,
                                    bool isHello) {
  if (!cfg.stickserverQueryEnabled && !cfg.stickserverPassiveDiscoveryEnabled)
    return;
  const char *end = payload + len;
  char buf[40];
  extractField(payload, end, "\"status\":\"", buf, sizeof(buf));
  // "partial"/"busy"/"not_found" responses to action commands still carry
  // genuine current per-output state and are handled by the general
  // (ArduinoJson) path; only hello/list themselves require a clean "ok".
  if (strcmp(buf, "ok") != 0)
    return;

  // The topic this message arrived on is always this peer's own instance
  // topic (it published its own response there) -- the redundant "topic"
  // field inside hello responses doesn't need parsing.
  int idx = allocateDiscoveredServerSlot(topicStr);
  DiscoveredServer &server = discoveredServers[idx];
  server.lastSeenMs = millis();
  if (!isHello)
    // A list response was just observed for this server (regardless of
    // who asked for it); it carries the full up-to-date output list, so
    // there is no need for this device to also re-request "list" from the
    // same server again soon. Mirrors the fleet-wide hello suppression.
    server.lastListRequestMs = millis();

  extractField(payload, end, "\"id\":\"", buf, sizeof(buf));
  if (isHello || buf[0])
    server.hostname = buf;

  extractField(payload, end, "\"ip\":\"", buf, sizeof(buf));
  if (buf[0])
    server.ipAddress = buf;

  const char *devicesKey = "\"devices\":[";
  const char *devicesStart = findBounded(payload, end, devicesKey);
  if (!devicesStart)
    return; // no per-output data in this response; keep what we already know.
  const char *p = devicesStart + strlen(devicesKey);
  char euidBuf[40];
  char nameBuf[24];
  char stateBuf[6];
  while (p < end) {
    while (p < end &&
          (*p == ',' || *p == ' ' || *p == '\n' || *p == '\r' || *p == '\t'))
      ++p;
    if (p >= end || *p == ']')
      break;
    if (*p != '{')
      break; // unexpected shape; stop rather than mis-scan the rest.
    const char *objEnd =
        static_cast<const char *>(memchr(p, '}', end - p));
    if (!objEnd)
      break;
    extractField(p, objEnd, "\"euid\":\"", euidBuf, sizeof(euidBuf));
    extractField(p, objEnd, "\"name\":\"", nameBuf, sizeof(nameBuf));
    extractField(p, objEnd, "\"state\":\"", stateBuf, sizeof(stateBuf));
    applyDiscoveredOutputFields(idx, euidBuf, nameBuf, stateBuf);
    p = objEnd + 1;
  }
}

void handleStickserverDiscoveryResponse(const String &topicStr,
                                        JsonDocument &response) {
  // Parse hello/list responses if either this device is actively querying
  // (it needs to process the results of its own requests) or passive
  // discovery is enabled (building the All Outputs page from traffic that
  // was not necessarily requested by us).
  if (!cfg.stickserverQueryEnabled && !cfg.stickserverPassiveDiscoveryEnabled)
    return;
  String rsp = response["rsp"] | String("");
  String status = response["status"] | String("");
  if (status != "ok") {
    // "partial"/"busy"/"not_found" responses to action commands (below)
    // still carry genuine current per-output state and are handled there;
    // only hello/list themselves require a clean "ok".
    if (rsp == F("hello") || rsp == F("list"))
      return;
  }
  if (rsp == F("hello")) {
    String topic = response["topic"] | topicStr;
    int idx = allocateDiscoveredServerSlot(topic);
    discoveredServers[idx].hostname = response["id"] | String("");
    String ip = response["ip"] | String("");
    if (!ip.isEmpty())
      discoveredServers[idx].ipAddress = ip;
    discoveredServers[idx].lastSeenMs = millis();
    // Some stickserver implementations include a per-output "devices" array
    // (euid/name/state) directly in the hello response; use it immediately
    // if present instead of waiting for the next "list" request/response.
    JsonArrayConst devices = response["devices"].as<JsonArrayConst>();
    if (!devices.isNull())
      applyDiscoveredOutputs(idx, devices);
  } else if (rsp == F("list")) {
    int idx = allocateDiscoveredServerSlot(topicStr);
    discoveredServers[idx].lastSeenMs = millis();
    // A list response was just observed for this server (regardless of who
    // asked for it); it carries the full up-to-date output list, so there
    // is no need for this device to also re-request "list" from the same
    // server again soon. Mirrors the fleet-wide hello suppression below.
    discoveredServers[idx].lastListRequestMs = millis();
    String hostId = response["id"] | String("");
    if (!hostId.isEmpty())
      discoveredServers[idx].hostname = hostId;
    String ip = response["ip"] | String("");
    if (!ip.isEmpty())
      discoveredServers[idx].ipAddress = ip;
    applyDiscoveredOutputs(idx, response["devices"].as<JsonArrayConst>());
  } else if (response["devices"].is<JsonArrayConst>() ||
             response["device"].is<JsonObjectConst>()) {
    // Any other stickserver response (power_on/power_off/reserve/release/
    // status/join/reboot/...) that carries per-output state. Only apply it
    // to a server we've already discovered via hello/list -- an action
    // response alone doesn't carry enough identity (hostname/ip) to safely
    // seed a brand-new "All Outputs" column.
    int idx = findDiscoveredServerSlot(topicStr);
    if (idx < 0)
      return;
    discoveredServers[idx].lastSeenMs = millis();
    if (response["devices"].is<JsonArrayConst>())
      applyDiscoveredOutputs(idx, response["devices"].as<JsonArrayConst>());
    if (response["device"].is<JsonObjectConst>())
      applyDiscoveredOutput(idx, response["device"].as<JsonObjectConst>());
  }
}


// Last time (from any source: this device's own request or any peer's) a
// "hello" discovery command was observed on the MQTT bus. Every stickserver
// sees every hello request via the shared root-topic subscription (this
// device's own published requests are echoed back to it too), and a hello
// RESPONSE carries full self-describing device info regardless of who asked
// -- so there is no need for every one of N fleet devices to independently
// issue the same broadcast every discovery cycle. Only one hello command is
// allowed on the bus per kHelloIntervalMs; every device defers to whichever
// one (its own or a peer's) is observed first and resets its own timer from
// it, collapsing what used to be up to N redundant broadcasts per interval
// into just one.
unsigned long lastHelloCommandSeenMs = 0;

void maintainStickserverDiscovery() {
  if (!cfg.mqttEnabled || !mqttClient.connected())
    return;
  unsigned long now = millis();
  static unsigned long lastPruneMs = 0;
  const unsigned long kHelloIntervalMs = 5000;
  const unsigned long kListIntervalMs = 10000;
  // Discovery records (and the per-output state folded into them) are
  // timestamped when parsed; anything not refreshed within this window is
  // forgotten so the All Outputs page doesn't show long-gone servers.
  const unsigned long kStaleTimeoutMs = 60000;

  if (cfg.stickserverQueryEnabled) {
    if (now - lastHelloCommandSeenMs >= kHelloIntervalMs) {
      // Optimistically mark the cooldown as started immediately (rather
      // than waiting for our own publish to echo back) so a
      // near-simultaneous loop iteration on this same device can't also
      // fire before the echo arrives.
      lastHelloCommandSeenMs = now;
      JsonDocument req;
      req["cmd"] = "hello";
      req["ver"] = STICKSERVER_PROTOCOL_VERSION;
      req["mid"] = String(F("disc-")) + String(now);
      String payload;
      serializeJson(req, payload);
      mqttClient.publish(STICKSERVER_ROOT_TOPIC, payload.c_str());
    }

    String selfTopic = stickserverInstanceTopic();
    for (uint8_t i = 0; i < MAX_DISCOVERED_SERVERS; ++i) {
      if (!discoveredServers[i].active)
        continue;
      // Never poll ourselves: our own hello response (received back via the
      // root-topic echo) already carries a full "devices" array every
      // kHelloIntervalMs, so a "list" round-trip to our own instance topic
      // would be a pointless, self-inflicted MQTT message every cycle.
      if (discoveredServers[i].instanceTopic == selfTopic)
        continue;
      if (now - discoveredServers[i].lastListRequestMs >= kListIntervalMs) {
        discoveredServers[i].lastListRequestMs = now;
        JsonDocument req;
        req["cmd"] = "list";
        req["ver"] = STICKSERVER_PROTOCOL_VERSION;
        req["mid"] = String(F("disc-list-")) + String(now) + "-" + String(i);
        String payload;
        serializeJson(req, payload);
        mqttClient.publish(discoveredServers[i].instanceTopic.c_str(),
                           payload.c_str());
      }
    }
  }

  // Prune stale records whenever either discovery flag might have
  // populated them (querying ourselves, or passively observed peer
  // traffic), so entries from a mode that just got disabled are still
  // forgotten instead of lingering forever.
  if ((cfg.stickserverQueryEnabled || cfg.stickserverPassiveDiscoveryEnabled) &&
      now - lastPruneMs >= 5000) {
    lastPruneMs = now;
    for (uint8_t i = 0; i < MAX_DISCOVERED_SERVERS; ++i) {
      if (discoveredServers[i].active &&
          now - discoveredServers[i].lastSeenMs > kStaleTimeoutMs) {
        discoveredServers[i] = DiscoveredServer();
      }
    }
  }
}

void handleStickserverMessage(const String &topicStr,
                              const String &payloadStr) {
  // Only messages addressed to us (the shared root broadcast topic, or our
  // own instance topic) ever warrant a response from this device. Every
  // other message on the bus is a peer's traffic that we may passively
  // parse for discovery, but must never react to with our own publish --
  // including on parse failure. (A genuinely malformed/truncated payload
  // from a peer is that peer's problem, not something we should announce
  // on the shared bus.)
  bool addressedToUs = topicStr == STICKSERVER_ROOT_TOPIC ||
                       topicStr == stickserverInstanceTopic();

  // Opt-in (debugSerial, GUI-toggleable on the Diagnostics page) heap
  // tracking around this call: this handler runs on every message observed
  // on the shared stickserver MQTT bus and is the prime suspect for the
  // background heap-fragmentation reboot, so logging free heap/largest free
  // block before and after both the parse and the discovery-state update
  // lets that be confirmed/narrowed down from live traffic instead of guesswork.
  bool heapTrace = cfg.debugSerial;
  uint32_t heapBefore = 0, blockBefore = 0;
  if (heapTrace) {
    heapBefore = ESP.getFreeHeap();
    blockBefore = ESP.getMaxFreeBlockSize();
    Serial.printf(
        "[HEAP] [MQTT] before parse topic=%s len=%u free=%u maxBlock=%u\n",
        topicStr.c_str(), payloadStr.length(), heapBefore, blockBefore);
  }

  // Zero-heap-allocation fast path for this firmware's own hello/list
  // responses (see handleStickserverHelloListFast() above) -- the
  // dominant, bursty traffic pattern on this bus. Falls through to the
  // general ArduinoJson parser below for anything that doesn't match that
  // exact shape (commands, action responses, foreign/malformed traffic).
  bool isFastHello = fastResponseIs(payloadStr.c_str(), payloadStr.length(), "hello");
  bool isFastList = !isFastHello &&
                    fastResponseIs(payloadStr.c_str(), payloadStr.length(), "list");
  if (isFastHello || isFastList) {
    handleStickserverHelloListFast(topicStr, payloadStr.c_str(),
                                   payloadStr.length(), isFastHello);
    if (heapTrace) {
      uint32_t heapAfter = ESP.getFreeHeap();
      uint32_t blockAfter = ESP.getMaxFreeBlockSize();
      Serial.printf("[HEAP] [MQTT] fast-path (no alloc) free=%u maxBlock=%u "
                    "deltaFree=%ld\n",
                    heapAfter, blockAfter, (long)heapAfter - (long)heapBefore);
      uint8_t slot = recordHeapTraceBase(topicStr, payloadStr.length(),
                                        heapBefore, blockBefore, heapAfter,
                                        blockAfter);
      recordHeapTraceDiscovery(slot, heapAfter, blockAfter);
    }
    return;
  }

  // `request` uses the default heap allocator (freed every call, costing no
  // static RAM) but is parsed with a field Filter (see
  // stickserverRequestFilter() above) to shrink each parse's transient
  // footprint -- this is the highest-frequency, most size-variable
  // JsonDocument in the firmware (parses every message observed on the
  // stickserver MQTT bus) and the believed primary driver of the
  // background heap-fragmentation reboot.
  JsonDocument request;
  DeserializationError err = deserializeJson(
      request, payloadStr,
      DeserializationOption::Filter(stickserverRequestFilter()));

  uint8_t heapTraceSlot = 0;
  if (heapTrace) {
    uint32_t heapAfterParse = ESP.getFreeHeap();
    uint32_t blockAfterParse = ESP.getMaxFreeBlockSize();
    Serial.printf(
        "[HEAP] [MQTT] after parse free=%u maxBlock=%u deltaFree=%ld\n",
        heapAfterParse, blockAfterParse,
        (long)heapAfterParse - (long)heapBefore);
    heapTraceSlot = recordHeapTraceBase(topicStr, payloadStr.length(),
                                       heapBefore, blockBefore,
                                       heapAfterParse, blockAfterParse);
  }

  if (err) {
    Serial.print(F("[WARN] [MQTT] Stickserver JSON parse error: "));
    Serial.println(err.c_str());
    if (addressedToUs) {
      publishStickserverFailure(F("error"), STICKSERVER_PROTOCOL_VERSION,
                                String(), F("invalid_json"), String(),
                                err.c_str());
    }
    return;
  }
  if (request["rsp"].is<const char *>()) {
    // Response message (ours or another stickserver instance's); used only
    // for fleet discovery, never re-processed as a command.
    handleStickserverDiscoveryResponse(topicStr, request);
    if (heapTrace) {
      uint32_t freeAfterDiscovery = ESP.getFreeHeap();
      uint32_t blockAfterDiscovery = ESP.getMaxFreeBlockSize();
      Serial.printf("[HEAP] [MQTT] after discovery-response free=%u "
                    "maxBlock=%u deltaFree=%ld\n",
                    freeAfterDiscovery, blockAfterDiscovery,
                    (long)freeAfterDiscovery - (long)heapBefore);
      recordHeapTraceDiscovery(heapTraceSlot, freeAfterDiscovery,
                               blockAfterDiscovery);
    }
    return;
  }


  // Commands not addressed to us (root broadcast or our own instance topic)
  // belong to another stickserver instance. We don't respond, but a "list"
  // request addressed to a peer is visible to us too via the shared
  // wildcard subscription -- note it against that peer's record so this
  // device doesn't also independently re-request "list" from them again
  // too soon (the resulting response, observed above, will refresh us
  // either way). Mirrors the fleet-wide hello suppression.
  if (!addressedToUs) {
    if (cfg.stickserverQueryEnabled &&
        request["cmd"].is<const char *>() &&
        String(request["cmd"].as<const char *>()) == F("list")) {
      for (uint8_t i = 0; i < MAX_DISCOVERED_SERVERS; ++i) {
        if (discoveredServers[i].active &&
            discoveredServers[i].instanceTopic == topicStr) {
          discoveredServers[i].lastListRequestMs = millis();
          break;
        }
      }
    }
    return;
  }

  String cmd = request["cmd"] | String("");
  String mid = "";
  if (request["mid"].is<const char *>()) {
    mid = request["mid"].as<const char *>();
  } else if (request["mid"].is<int>()) {
    mid = String(request["mid"].as<int>());
  } else if (request["mid"].is<long>()) {
    mid = String(request["mid"].as<long>());
  }
  DBG("[MQTT] Stickserver cmd: %s mid: %s",
      cmd.isEmpty() ? "(none)" : cmd.c_str(),
      mid.isEmpty() ? "(none)" : mid.c_str());
  if (!request["ver"].is<int>()) {
    publishStickserverFailure(
        cmd.isEmpty() ? String(F("error")) : cmd, STICKSERVER_PROTOCOL_VERSION,
        mid, F("invalid_member"), F("ver"), F("Missing or invalid ver."));
    return;
  }
  int ver = request["ver"].as<int>();
  if (mid.isEmpty()) {
    publishStickserverFailure(cmd.isEmpty() ? String(F("error")) : cmd, ver,
                              mid, F("invalid_member"), F("mid"),
                              F("Missing or invalid mid."));
    return;
  }
  if (cmd.isEmpty()) {
    publishStickserverFailure(F("error"), ver, mid, F("invalid_member"),
                              F("cmd"), F("Missing or invalid cmd."));
    return;
  }
  if (topicStr == STICKSERVER_ROOT_TOPIC && cmd != F("hello")) {
    publishStickserverFailure(cmd, ver, mid, F("invalid_command"), F("cmd"),
                              F("Only hello is accepted on the root topic."));
    return;
  }

  if (cmd == F("hello")) {
    // A hello command was just observed on the bus (ours or a peer's);
    // reset the shared fleet-wide cooldown so no actively-querying device
    // re-broadcasts one again too soon -- see maintainStickserverDiscovery().
    if (cfg.stickserverQueryEnabled)
      lastHelloCommandSeenMs = millis();
    if (!cfg.stickserverRespondEnabled) {
      // Responses disabled: don't advertise this device to the fleet.
      return;
    }
    JsonDocument response;
    buildStickserverEnvelope(response, cmd, ver, mid, "ok");
    response["id"] = cfg.hostname;
    response["mac"] = cfg.mac;
    response["topic"] = stickserverInstanceTopic();
    response["instance"] = stickserverInstanceId();
    response["ip"] = WiFi.localIP().toString();
    response["count"] = managedOutputCount();
    response["available"] = availableManagedOutputCount();
    JsonArray devices = response["devices"].to<JsonArray>();
    for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
      if (!isManagedOutput(i))
        continue;
      populateStickserverDevice(devices.add<JsonObject>(), i);
    }
    publishStickserverResponse(response);
    return;
  }

  if (cmd == F("list")) {
    if (!cfg.stickserverRespondEnabled) {
      // Responses disabled: don't respond to list requests either.
      return;
    }
    JsonDocument response;
    buildStickserverEnvelope(response, cmd, ver, mid, "ok");
    response["id"] = cfg.hostname;
    response["mac"] = cfg.mac;
    response["ip"] = WiFi.localIP().toString();
    response["count"] = managedOutputCount();
    response["available"] = availableManagedOutputCount();
    JsonArray devices = response["devices"].to<JsonArray>();
    for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
      if (!isManagedOutput(i))
        continue;
      populateStickserverDevice(devices.add<JsonObject>(), i);
    }
    publishStickserverResponse(response);
    return;
  }

  if (cmd == F("reserve")) {
    String owner = request["owner"] | String("");
    String ntype = request["ntype"] | String("");
    if (owner.isEmpty()) {
      publishStickserverFailure(cmd, ver, mid, F("invalid_member"), F("owner"),
                                F("Missing owner."));
      return;
    }
    if (ntype.isEmpty()) {
      publishStickserverFailure(cmd, ver, mid, F("invalid_member"), F("ntype"),
                                F("Missing ntype."));
      return;
    }
    if (!request["count"].is<int>()) {
      publishStickserverFailure(cmd, ver, mid, F("invalid_member"), F("count"),
                                F("Missing or invalid count."));
      return;
    }
    int requestedCount = request["count"].as<int>();
    if (requestedCount < 1) {
      publishStickserverFailure(cmd, ver, mid, F("invalid_member"), F("count"),
                                F("count must be >= 1."));
      return;
    }
    DBG("[MQTT] reserve matching ntype='%s' against configured Model fields.",
        ntype.c_str());

    uint8_t reservedIdx[MAX_DEVICES] = {0};
    bool newReservation[MAX_DEVICES] = {false};
    size_t reservedCount = 0;
    for (uint8_t i = 0; i < cfg.numOutputs &&
                        reservedCount < static_cast<size_t>(requestedCount);
         ++i) {
      if (!isManagedOutput(i))
        continue;
      if (!deviceModelMatches(cfg.devices[i].model, ntype))
        continue;
      if (outputReservations[i].reserved &&
          outputReservations[i].owner == owner) {
        reservedIdx[reservedCount] = i;
        newReservation[reservedCount] = false;
        ++reservedCount;
      }
    }
    for (uint8_t i = 0; i < cfg.numOutputs &&
                        reservedCount < static_cast<size_t>(requestedCount);
         ++i) {
      if (!isManagedOutput(i) || outputReservations[i].reserved)
        continue;
      if (!deviceModelMatches(cfg.devices[i].model, ntype))
        continue;
      DBG("[MQTT] reserve selecting output %u with Model='%s'.",
          static_cast<unsigned>(i), cfg.devices[i].model.c_str());
      outputReservations[i].reserved = true;
      outputReservations[i].owner = owner;
      reservedIdx[reservedCount] = i;
      newReservation[reservedCount] = true;
      ++reservedCount;
    }
    maybePersistReservations();

    JsonDocument response;
    buildStickserverEnvelope(
        response, cmd, ver, mid,
        reservedCount == static_cast<size_t>(requestedCount)
            ? "ok"
            : (reservedCount > 0 ? "partial" : "unavailable"));
    response["owner"] = owner;
    response["ntype"] = ntype;
    response["requested"] = requestedCount;
    response["allocated"] = reservedCount;
    response["available"] = availableManagedOutputCount();
    JsonArray devices = response["devices"].to<JsonArray>();
    for (size_t i = 0; i < reservedCount; ++i) {
      JsonObject device = devices.add<JsonObject>();
      populateStickserverDevice(device, reservedIdx[i]);
      device["new"] = newReservation[i];
    }
    publishStickserverResponse(response);
    return;
  }

  if (cmd == F("release")) {
    String owner = request["owner"] | String("");
    String euids[MAX_DEVICES];
    size_t euidCount = 0;
    bool hasEuids = !request["euids"].isNull();
    // An explicitly-empty euids array ("euids": []) is treated the same as
    // omitting euids entirely (see releaseAll below), not as invalid input.
    bool euidsIsEmptyArray = hasEuids && request["euids"].is<JsonArrayConst>() &&
                             request["euids"].as<JsonArrayConst>().size() == 0;
    if (hasEuids && !euidsIsEmptyArray &&
        !extractStringArray(request["euids"], euids, euidCount)) {
      publishStickserverFailure(cmd, ver, mid, F("invalid_member"), F("euids"),
                                F("Invalid euids list."));
      return;
    }
    // No owner and no euids specified (or an empty euids list): release all
    // currently-reserved managed outputs.
    bool releaseAll = owner.isEmpty() && euidCount == 0;

    bool selected[MAX_DEVICES] = {false};
    JsonDocument response;
    JsonArray devices = response["devices"].to<JsonArray>();
    if (releaseAll) {
      for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
        if (isManagedOutput(i) && outputReservations[i].reserved) {
          selected[i] = true;
        }
      }
    } else {
      if (!owner.isEmpty()) {
        for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
          if (isManagedOutput(i) && outputReservations[i].reserved &&
              outputReservations[i].owner == owner) {
            selected[i] = true;
          }
        }
        response["owner"] = owner;
      }
      for (size_t i = 0; i < euidCount; ++i) {
        int idx = findManagedOutputByEuid(euids[i]);
        if (idx < 0) {
          JsonObject device = devices.add<JsonObject>();
          device["euid"] = euids[i];
          device["released"] = false;
          device["status"] = "unknown_euid";
        } else {
          selected[idx] = true;
        }
      }
    }

    size_t releasedCount = 0;
    size_t selectedCount = 0;
    for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
      if (!selected[i])
        continue;
      ++selectedCount;
      JsonObject device = devices.add<JsonObject>();
      populateStickserverDevice(device, i);
      if (outputReservations[i].reserved) {
        outputReservations[i].reserved = false;
        outputReservations[i].owner = "";
        device["released"] = true;
        device["status"] = "ok";
        device["available"] = true;
        device.remove("owner");
        ++releasedCount;
      } else {
        device["released"] = false;
        device["status"] = "not_reserved";
      }
    }
    if (releasedCount > 0)
      maybePersistReservations();

    buildStickserverEnvelope(
        response, cmd, ver, mid,
        releasedCount == selectedCount && devices.size() == selectedCount
            ? "ok"
            : (releasedCount > 0 ? "partial" : "not_found"));
    response["released"] = releasedCount;
    response["available"] = availableManagedOutputCount();
    publishStickserverResponse(response);
    return;
  }

  if (cmd == F("status")) {
    String euids[MAX_DEVICES];
    size_t euidCount = 0;
    if (!extractStringArray(request["euids"], euids, euidCount)) {
      publishStickserverFailure(cmd, ver, mid, F("invalid_member"), F("euids"),
                                F("Invalid euids list."));
      return;
    }

    JsonDocument response;
    buildStickserverEnvelope(response, cmd, ver, mid, "ok");
    JsonArray devices = response["devices"].to<JsonArray>();
    size_t okCount = 0;
    for (size_t i = 0; i < euidCount; ++i) {
      int idx = findManagedOutputByEuid(euids[i]);
      JsonObject device = devices.add<JsonObject>();
      if (idx < 0) {
        device["euid"] = euids[i];
        device["status"] = "unknown_euid";
      } else {
        populateStickserverDevice(device, static_cast<uint8_t>(idx));
        device["status"] = "ok";
        ++okCount;
      }
    }
    response["status"] = aggregateStickserverStatus(okCount, euidCount);
    publishStickserverResponse(response);
    return;
  }

  if (cmd == F("join") || cmd == F("reboot")) {
    String euid;
    int idx = resolveRequestedEuid(request["euid"], euid);
    if (idx == -2) {
      publishStickserverFailure(cmd, ver, mid, F("invalid_member"), F("euid"),
                                F("Missing euid."));
      return;
    }
    if (idx < 0) {
      publishStickserverFailure(cmd, ver, mid, F("invalid_member"), F("euid"),
                                F("Unknown euid."));
      return;
    }
    String mappedCmd;
    if (cmd == F("join")) {
      mappedCmd = F("power_on");
    } else {
      mappedCmd = F("reboot");
    }
    bool ok = handleLoadAction(static_cast<uint8_t>(idx), mappedCmd);

    JsonDocument response;
    buildStickserverEnvelope(response, cmd, ver, mid, ok ? "ok" : "busy");
    response["action"] = mappedCmd;
    JsonObject device = response["device"].to<JsonObject>();
    populateStickserverDevice(device, static_cast<uint8_t>(idx));
    device["status"] = ok ? "ok" : "busy";
    publishStickserverResponse(response);
    return;
  }

  // Commands routed by euid: power_on, power_off, factory_reset
  if (cmd == F("power_on") || cmd == F("power_off") ||
      cmd == F("factory_reset")) {
    String euid;
    int idx = resolveRequestedEuid(request["euid"], euid);
    if (idx == -2) {
      publishStickserverFailure(cmd, ver, mid, F("invalid_member"), F("euid"),
                                F("Missing euid."));
      return;
    }
    if (idx < 0) {
      publishStickserverFailure(cmd, ver, mid, F("invalid_member"), F("euid"),
                                F("Unknown euid."));
      return;
    }
    bool ok = handleLoadAction(static_cast<uint8_t>(idx), cmd);

    JsonDocument response;
    buildStickserverEnvelope(response, cmd, ver, mid, ok ? "ok" : "busy");
    JsonObject device = response["device"].to<JsonObject>();
    populateStickserverDevice(device, static_cast<uint8_t>(idx));
    device["status"] = ok ? "ok" : "busy";
    publishStickserverResponse(response);
    return;
  }

  if (cmd == F("leave")) {
    // leave supports either:
    // - euid (single; string or int), or
    // - euids (array/string list; existing behavior)
    JsonDocument response;
    JsonArray devices = response["devices"].to<JsonArray>();
    size_t okCount = 0;
    size_t knownCount = 0;
    size_t unknownCount = 0;
    size_t totalRequested = 0;

    if (!request["euid"].isNull()) {
      String euid;
      int idx = resolveRequestedEuid(request["euid"], euid);
      if (idx == -2) {
        publishStickserverFailure(cmd, ver, mid, F("invalid_member"), F("euid"),
                                  F("Missing euid."));
        return;
      }
      totalRequested = 1;
      JsonObject device = devices.add<JsonObject>();
      if (idx < 0) {
        device["euid"] = euid;
        device["status"] = "unknown_euid";
        ++unknownCount;
      } else {
        ++knownCount;
        bool ok = handleLoadAction(static_cast<uint8_t>(idx), F("leave_mesh"));
        populateStickserverDevice(device, static_cast<uint8_t>(idx));
        device["action"] = "leave_mesh";
        device["status"] = ok ? "ok" : "busy";
        if (ok)
          ++okCount;
      }
    } else {
      String euids[MAX_DEVICES];
      size_t euidCount = 0;
      if (!extractStringArray(request["euids"], euids, euidCount)) {
        publishStickserverFailure(cmd, ver, mid, F("invalid_member"),
                                  F("euids"), F("Invalid euids list."));
        return;
      }
      totalRequested = euidCount;
      for (size_t i = 0; i < euidCount; ++i) {
        int idx = findManagedOutputByEuid(euids[i]);
        DBG("[MQTT] Resolved idx from euid: %d", idx);
        JsonObject device = devices.add<JsonObject>();
        if (idx < 0) {
          device["euid"] = euids[i];
          device["status"] = "unknown_euid";
          ++unknownCount;
          continue;
        }
        ++knownCount;
        bool ok = handleLoadAction(static_cast<uint8_t>(idx), F("leave_mesh"));
        populateStickserverDevice(device, static_cast<uint8_t>(idx));
        device["action"] = "leave_mesh";
        device["status"] = ok ? "ok" : "busy";
        if (ok)
          ++okCount;
      }
    }

    const char *responseStatus = "partial";
    if (okCount == totalRequested) {
      responseStatus = "ok";
    } else if (knownCount == 0) {
      responseStatus = "not_found";
    } else if (unknownCount == 0) {
      responseStatus = "busy";
    }
    buildStickserverEnvelope(response, cmd, ver, mid, responseStatus);
    publishStickserverResponse(response);
    return;
  }

  publishStickserverFailure(cmd, ver, mid, F("invalid_command"), F("cmd"),
                            F("Unsupported stickserver command."));
}

} // namespace vibrant
