#pragma once
#include <Arduino.h>
namespace vibrant {

// Runtime health telemetry used to diagnose unexpected reboots and to shed
// load before the heap is exhausted.
struct HealthStats {
  uint32_t minFreeHeap;  // lowest free heap seen since boot (bytes)
  uint32_t minMaxBlock;  // lowest largest-free-block seen since boot (bytes)
  uint32_t maxLoopGapMs; // longest time between two loop() iterations
  uint16_t loadShedCount; // times discovery data was dropped to recover heap
};
extern HealthStats health;

// Call once in setup() after LittleFS is mounted: captures the hardware
// reset cause (exception/watchdog/power) and the reason recorded by the
// previous software restart, if any.
void healthBegin();
// Persists why the firmware is about to restart so the next boot can report
// it. Safe to call when the filesystem is unavailable.
void recordRestartReason(const char *reason);
// Call every loop() iteration.
void maintainHealth();
// Human-readable cause of the most recent reboot.
const String &healthBootCause();
} // namespace vibrant
