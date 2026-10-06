#include "NtpClock.h"
#include "Config.h"
#include "Debug.h"
#include <time.h>

namespace c4matrix {
namespace {
// Re-request a sync every 15 minutes (4 times per hour), as requested.
constexpr unsigned long NTP_RESYNC_INTERVAL_MS = 15UL * 60UL * 1000UL;
bool ntpStarted = false;
unsigned long lastSyncAttempt = 0;
} // namespace

void ntpBegin() {
  // Always fetch UTC; the configured UTC offset is applied at render time
  // (in ntpTimeString()) so changing it live needs no re-sync.
  configTime(0, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");
  ntpStarted = true;
  lastSyncAttempt = millis();
  logStatus(F("NTP time sync started."));
}

void ntpMaintain() {
  if (!ntpStarted)
    return;
  unsigned long now = millis();
  if (now - lastSyncAttempt < NTP_RESYNC_INTERVAL_MS)
    return;
  lastSyncAttempt = now;
  configTime(0, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");
  logStatus(F("NTP re-synchronization requested."));
}

bool ntpTimeValid() {
  // Before a successful sync, ESP8266 time() returns a small epoch value
  // (close to 0); this is the standard sanity check used to detect that.
  return time(nullptr) > 8 * 3600 * 2;
}

String ntpTimeString(bool showColon) {
  char separator = showColon ? ':' : ' ';
  if (!ntpTimeValid())
    return String(F("--")) + separator + String(F("--"));
  time_t now = time(nullptr) +
               static_cast<int32_t>(cfg.utcOffsetMinutes) * 60;
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  char buffer[6];
  snprintf(buffer, sizeof(buffer), "%02d%c%02d", timeinfo.tm_hour, separator,
           timeinfo.tm_min);
  return String(buffer);
}
}
