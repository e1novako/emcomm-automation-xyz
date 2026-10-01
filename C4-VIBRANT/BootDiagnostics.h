#pragma once
#include <Arduino.h>
namespace vibrant {
extern const uint8_t FLASH_BUTTON_PIN;
extern const unsigned long FLASH_BOOT_DETECTION_WINDOW_MS;
bool detectStableFlashPressDuringBoot();
void checkFlashFactoryResetOnBoot();
} // namespace vibrant
