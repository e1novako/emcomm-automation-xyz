#pragma once
#include <ESP8266WebServer.h>

namespace c4matrix {
extern ESP8266WebServer server;
void registerWebRoutes();
String htmlEscape(const String &);
String jsonEscape(const String &);
bool ensureAuthorized();
void handleHome();
void handleConfig();
void handleConfigSave();
void handleFactoryReset();
void handleStatus();
void handleTextPost();
void handleScrollPost();
void handleNotFound();
}
