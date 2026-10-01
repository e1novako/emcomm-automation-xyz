#include "Stickserver.h"
#include "Actions.h"
#include "Config.h"
#include "Debug.h"
#include "MqttClient.h"
#include "Outputs.h"
#include "Reservations.h"
#include <ArduinoJson.h>
#include <ctype.h>

namespace vibrant {

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

String stickserverInstanceTopic() {
  return String(STICKSERVER_ROOT_TOPIC) + '/' + stickserverInstanceId();
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
  String payload;
  if (serializeJson(doc, payload) == 0) {
    logWarning(F("Stickserver: failed to serialize response JSON."));
    return false;
  }
  String topic = stickserverInstanceTopic();
  bool ok = mqttClient.publish(topic.c_str(), payload.c_str());
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

void handleStickserverDiscoveryResponse(const String &topicStr,
                                        JsonDocument &response) {
  String rsp = response["rsp"] | String("");
  String status = response["status"] | String("");
  if (status != "ok")
    return;
  if (rsp == F("hello")) {
    String topic = response["topic"] | topicStr;
    int idx = allocateDiscoveredServerSlot(topic);
    discoveredServers[idx].hostname = response["id"] | String("");
    discoveredServers[idx].instanceId = response["instance"] | String("");
    discoveredServers[idx].lastSeenMs = millis();
  } else if (rsp == F("list")) {
    int idx = allocateDiscoveredServerSlot(topicStr);
    discoveredServers[idx].lastSeenMs = millis();
    String hostId = response["id"] | String("");
    if (!hostId.isEmpty())
      discoveredServers[idx].hostname = hostId;
    JsonArrayConst devices = response["devices"].as<JsonArrayConst>();
    uint8_t count = 0;
    for (JsonVariantConst item : devices) {
      if (count >= MAX_DEVICES)
        break;
      JsonObjectConst dev = item.as<JsonObjectConst>();
      discoveredServers[idx].outputs[count].euid = dev["euid"] | String("");
      discoveredServers[idx].outputs[count].name = dev["name"] | String("");
      String stateStr = dev["state"] | String("OFF");
      discoveredServers[idx].outputs[count].state = (stateStr == "ON");
      discoveredServers[idx].outputs[count].valid = true;
      ++count;
    }
    for (uint8_t i = count; i < MAX_DEVICES; ++i) {
      discoveredServers[idx].outputs[i].valid = false;
    }
    discoveredServers[idx].outputCount = count;
  }
}

void maintainStickserverDiscovery() {
  if (!cfg.mqttEnabled || !mqttClient.connected())
    return;
  unsigned long now = millis();
  static unsigned long lastHelloMs = 0;
  static unsigned long lastPruneMs = 0;
  const unsigned long kHelloIntervalMs = 15000;
  const unsigned long kListIntervalMs = 10000;
  const unsigned long kStaleTimeoutMs = 90000;

  if (now - lastHelloMs >= kHelloIntervalMs) {
    lastHelloMs = now;
    JsonDocument req;
    req["cmd"] = "hello";
    req["ver"] = STICKSERVER_PROTOCOL_VERSION;
    req["mid"] = String(F("disc-")) + String(now);
    String payload;
    serializeJson(req, payload);
    mqttClient.publish(STICKSERVER_ROOT_TOPIC, payload.c_str());
  }

  for (uint8_t i = 0; i < MAX_DISCOVERED_SERVERS; ++i) {
    if (!discoveredServers[i].active)
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

  if (now - lastPruneMs >= 5000) {
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
  JsonDocument request;
  DeserializationError err = deserializeJson(request, payloadStr);
  if (err) {
    Serial.print(F("[WARN] [MQTT] Stickserver JSON parse error: "));
    Serial.println(err.c_str());
    publishStickserverFailure(F("error"), STICKSERVER_PROTOCOL_VERSION,
                              String(), F("invalid_json"), String(),
                              err.c_str());
    return;
  }
  if (request["rsp"].is<const char *>()) {
    // Response message (ours or another stickserver instance's); used only
    // for fleet discovery, never re-processed as a command.
    handleStickserverDiscoveryResponse(topicStr, request);
    return;
  }

  // Commands not addressed to us (root broadcast or our own instance topic)
  // belong to another stickserver instance; ignore (do not respond).
  if (topicStr != STICKSERVER_ROOT_TOPIC &&
      topicStr != stickserverInstanceTopic()) {
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
    JsonDocument response;
    buildStickserverEnvelope(response, cmd, ver, mid, "ok");
    response["id"] = cfg.hostname;
    response["mac"] = cfg.mac;
    response["topic"] = stickserverInstanceTopic();
    response["instance"] = stickserverInstanceId();
    response["count"] = managedOutputCount();
    response["available"] = availableManagedOutputCount();
    publishStickserverResponse(response);
    return;
  }

  if (cmd == F("list")) {
    JsonDocument response;
    buildStickserverEnvelope(response, cmd, ver, mid, "ok");
    response["id"] = cfg.hostname;
    response["mac"] = cfg.mac;
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
    if (hasEuids && !extractStringArray(request["euids"], euids, euidCount)) {
      publishStickserverFailure(cmd, ver, mid, F("invalid_member"), F("euids"),
                                F("Invalid euids list."));
      return;
    }
    if (owner.isEmpty() && euidCount == 0) {
      publishStickserverFailure(cmd, ver, mid, F("invalid_member"),
                                F("owner/euids"),
                                F("Release requires owner and/or euids."));
      return;
    }

    bool selected[MAX_DEVICES] = {false};
    JsonDocument response;
    JsonArray devices = response["devices"].to<JsonArray>();
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
