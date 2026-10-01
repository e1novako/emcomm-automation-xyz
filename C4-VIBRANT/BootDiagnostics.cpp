#include "BootDiagnostics.h"
#include "Config.h"
#include "Debug.h"
#include <Arduino.h>

namespace vibrant {

const uint8_t FLASH_BUTTON_PIN = 0;
const unsigned long FLASH_BOOT_DETECTION_WINDOW_MS = 750UL;
constexpr unsigned long FLASH_BOOT_SAMPLE_INTERVAL_MS = 10UL;
constexpr uint8_t FLASH_BOOT_REQUIRED_LOW_PERCENT = 30;
constexpr unsigned long FLASH_BOOT_MIN_SAMPLES = 4UL;

constexpr unsigned long ceilDiv(unsigned long numerator,
                                unsigned long denominator) {
  if (denominator == 0)
    return 0;
  return (numerator + denominator - 1UL) / denominator;
}

constexpr unsigned long ceilPercentOf(unsigned long value,
                                      unsigned long percent) {
  return static_cast<unsigned long>(
      (static_cast<unsigned long long>(value) * percent + 99ULL) / 100ULL);
}

constexpr unsigned long FLASH_BOOT_SAMPLE_COUNT =
    ceilDiv(FLASH_BOOT_DETECTION_WINDOW_MS, FLASH_BOOT_SAMPLE_INTERVAL_MS);
constexpr unsigned long FLASH_BOOT_REQUIRED_LOW_SAMPLES =
    ceilPercentOf(FLASH_BOOT_SAMPLE_COUNT, FLASH_BOOT_REQUIRED_LOW_PERCENT);

static_assert(FLASH_BOOT_SAMPLE_INTERVAL_MS > 0,
              "FLASH boot sample interval must be greater than zero.");
static_assert(
    FLASH_BOOT_SAMPLE_COUNT >= FLASH_BOOT_MIN_SAMPLES,
    "FLASH boot detection window must collect the minimum number of samples.");

bool detectStableFlashPressDuringBoot() {
  unsigned long lowSamples = 0;

  // This short blocking window is intentional: GPIO0 is only sampled during
  // boot, not polled continuously at runtime.
  for (unsigned long i = 0; i < FLASH_BOOT_SAMPLE_COUNT; ++i) {
    if (i > 0) {
      delay(FLASH_BOOT_SAMPLE_INTERVAL_MS);
    }
    if (digitalRead(FLASH_BUTTON_PIN) == LOW) {
      ++lowSamples;
    }
  }

  Serial.print(F("[INFO] [FLASH] Boot-time samples low="));
  Serial.print(lowSamples);
  Serial.print(F("/"));
  Serial.println(FLASH_BOOT_SAMPLE_COUNT);

  return lowSamples >= FLASH_BOOT_REQUIRED_LOW_SAMPLES;
}

void checkFlashFactoryResetOnBoot() {
  logStatus(
      F("[FLASH] Checking boot-time FLASH/GPIO0 factory reset trigger..."));
  bool stablePressed = detectStableFlashPressDuringBoot();

  Serial.print(F("[INFO] [FLASH] Stable pressed condition "));
  Serial.println(stablePressed ? F("detected.") : F("not detected."));

  if (!stablePressed) {
    logStatus(F("[FLASH] Boot-time FLASH/GPIO0 reset skipped."));
    return;
  }

  logWarning(F("[FLASH] Boot-time FLASH/GPIO0 reset trigger detected. Applying "
               "factory defaults."));
  performFactoryResetAndRestart(
      F("FLASH/GPIO0 was held low during the boot-time detection window."));
}

} // namespace vibrant
