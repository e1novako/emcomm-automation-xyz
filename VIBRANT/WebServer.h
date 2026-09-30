#pragma once
#include "Config.h"
#include <ESP8266WebServer.h>
#include "../libraries/EmcommCommon/src/EmcommCommon/Web.h"
namespace vibrant {
using emcomm::htmlEscape;
extern ESP8266WebServer server;
void registerWebRoutes();
String pinOption(int, const PinMapping &);
bool parsePinValue(const String &, int &);
bool parseIndexValue(const String &, int &);
bool parseFloatValue(const String &, float &);
bool usingFactoryPassword();
String passwordWarningHtml();
bool ensureAuthorized();
void handleHome();
void handleToggle();
void handleAction();
void handleCancelAction();
void handleActionStatus();
void handleAllOn();
void handleAllOff();
void handleFactoryResetAll();
void handleLeaveMeshAll();
void handleSettingsGet();
void handleSettingsPost();
void handleNotFound();
} // namespace vibrant
