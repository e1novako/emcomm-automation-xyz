#pragma once
#include "Config.h"
#include <ArduinoJson.h>
namespace vibrant {
extern const char STICKSERVER_ROOT_TOPIC[], STICKSERVER_OUTPUT_TYPE[];
extern const uint8_t STICKSERVER_PROTOCOL_VERSION;

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
