#pragma once
#include "Config.h"
#include <ESP8266WebServer.h>
namespace vibrant {
extern ESP8266WebServer server;
void registerWebRoutes();
String htmlEscape(const String &);
String jsonEscape(const String &);
String formatUptimeHHMMSS(unsigned long);
void sendChunkedHtml(int, const String &);
String pinOption(int, const PinMapping &);
bool parsePinValue(const String &, int &);
bool parseIndexValue(const String &, int &);
bool parseFloatValue(const String &, float &);
bool usingFactoryPassword();
String passwordWarningHtml();
bool ensureAuthorized();
void handleHome();
void handleHomePartial();
void handleToggle();
void handleAction();
void handleCancelAction();
void handleActionStatus();
void handleAllOn();
void handleAllOff();
void handleFactoryResetAll();
void handleLeaveMeshAll();
void handleSettingsGet();
void handleNetworkSettingsGet();
void handleNetworkSettingsPost();
void handleDeviceSettingsGet();
void handleDeviceSettingsPost();
void handleDiagnosticsGet();
void handleDiagnosticsData();
void handleDiagnosticsHeapLog();
void handleDiagnosticsPost();
void handleRebootDevice();
void handleStickserverFleetGet();
void handleStickserverFleetData();
void handleFleetOutputToggle();
void handleFleetBulkAction();
void handleReleaseReservation();
void handleGuiReserveOutput();
void handleNotFound();
} // namespace vibrant
