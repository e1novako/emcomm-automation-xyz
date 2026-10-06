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
void handleConfigNetwork();
void handleConfigNetworkSave();
void handleConfigDisplay();
void handleConfigDisplaySave();
void handleConfigOta();
void handleConfigOtaSave();
void handleConfigDiagnostics();
void handleConfigDiagnosticsSave();
void handleFactoryReset();
void handleStatus();
void handleTextPost();
void handleScrollPost();
void handleLedsPost();
void handleNotFound();
}
