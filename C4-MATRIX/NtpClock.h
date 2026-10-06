#pragma once
#include <Arduino.h>

namespace c4matrix {
// Starts the SNTP client. Safe to call once Wi-Fi is configured; actual
// synchronization happens asynchronously once network access is available.
void ntpBegin();
// Call periodically from loop(): re-requests a time sync every
// NTP_RESYNC_INTERVAL_MS (a few times per hour) to keep the clock accurate.
void ntpMaintain();
// True once the device has obtained a plausible time from NTP.
bool ntpTimeValid();
// Returns "HH:MM" (or "HH MM" when showColon is false, for a blinking
// colon) in local time (per cfg.utcOffsetMinutes), or "--:--"/"-- --" if
// the clock has not synchronized yet.
String ntpTimeString(bool showColon = true);
}
