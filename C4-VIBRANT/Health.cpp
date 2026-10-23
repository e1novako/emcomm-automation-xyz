#include "Health.h"
#include "Config.h"
#include "Debug.h"
#include "Stickserver.h"
#include <LittleFS.h>
extern "C" {
#include "user_interface.h"
}

namespace vibrant {

HealthStats health = {0xFFFFFFFFUL, 0xFFFFFFFFUL, 0, 0};

namespace {
const char LAST_RESTART_PATH[] = "/lastrestart.txt";
constexpr unsigned long SAMPLE_INTERVAL_MS = 500UL;
// Below these the next page render or MQTT burst can fail an allocation.
constexpr uint32_t LOW_FREE_HEAP_BYTES = 4096;
constexpr uint32_t LOW_MAX_BLOCK_BYTES = 3584;
// Heap that stays exhausted after shedding is not going to recover; restart
// cleanly instead of crashing mid-write.
constexpr unsigned long LOW_HEAP_RESTART_AFTER_MS = 60000UL;

String bootCause;
unsigned long lowHeapSinceMs = 0;
bool shedThisEpisode = false;

void clearDiscoveredServers() {
  for (uint8_t i = 0; i < MAX_DISCOVERED_SERVERS; ++i)
    discoveredServers[i].reset();
}
} // namespace

const String &healthBootCause() { return bootCause; }

void recordRestartReason(const char *reason) {
  if (!reason)
    return;
  File f = LittleFS.open(LAST_RESTART_PATH, "w");
  if (!f)
    return;
  f.print(reason);
  f.close();
}

void healthBegin() {
  const rst_info *info = ESP.getResetInfoPtr();
  bootCause = ESP.getResetReason();
  if (info->reason == REASON_EXCEPTION_RST) {
    bootCause += F(" (exccause=");
    bootCause += String(info->exccause);
    bootCause += F(" epc1=0x");
    bootCause += String(info->epc1, HEX);
    bootCause += F(" excvaddr=0x");
    bootCause += String(info->excvaddr, HEX);
    bootCause += ')';
  }
  if (LittleFS.exists(LAST_RESTART_PATH)) {
    File f = LittleFS.open(LAST_RESTART_PATH, "r");
    if (f) {
      String reason = f.readString();
      f.close();
      // Only trust the recorded reason if the chip really did a software
      // restart; after a crash or power loss the file is stale.
      if (info->reason == REASON_SOFT_RESTART) {
        bootCause += F(": ");
        bootCause += reason;
      }
    }
    LittleFS.remove(LAST_RESTART_PATH);
  }
  Serial.print(F("[INFO] Last reboot cause: "));
  Serial.println(bootCause);
}

void maintainHealth() {
  unsigned long now = millis();
  static unsigned long lastLoopMs = 0, lastSampleMs = 0;
  if (lastLoopMs != 0) {
    uint32_t gap = now - lastLoopMs;
    if (gap > health.maxLoopGapMs)
      health.maxLoopGapMs = gap;
  }
  lastLoopMs = now;
  if (now - lastSampleMs < SAMPLE_INTERVAL_MS)
    return;
  lastSampleMs = now;

  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t maxBlock = ESP.getMaxFreeBlockSize();
  if (freeHeap < health.minFreeHeap)
    health.minFreeHeap = freeHeap;
  if (maxBlock < health.minMaxBlock)
    health.minMaxBlock = maxBlock;

  if (freeHeap >= LOW_FREE_HEAP_BYTES && maxBlock >= LOW_MAX_BLOCK_BYTES) {
    lowHeapSinceMs = 0;
    shedThisEpisode = false;
    return;
  }
  if (lowHeapSinceMs == 0)
    lowHeapSinceMs = now;
  if (!shedThisEpisode) {
    shedThisEpisode = true;
    ++health.loadShedCount;
    logWarning(String(F("[HEALTH] Low heap (free ")) + String(freeHeap) +
               F(" B, max block ") + String(maxBlock) +
               F(" B); dropping discovered fleet data."));
    clearDiscoveredServers();
    return;
  }
  if (now - lowHeapSinceMs >= LOW_HEAP_RESTART_AFTER_MS)
    restartDevice(F("Heap stayed exhausted after load shedding."));
}

} // namespace vibrant
