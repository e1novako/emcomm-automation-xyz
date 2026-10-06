#pragma once
#include <Arduino.h>

namespace c4matrix {
enum class DisplayMode : uint8_t { Text, Fill, Off, Count, Clock };
enum class ScrollDirection : uint8_t { Left, Right };
const char *displayModeName(DisplayMode mode);
uint16_t displayLedCount();
uint16_t displayMatrixWidth();
uint16_t displayMatrixHeight();
constexpr uint16_t MATRIX_MAX_WIDTH = 32;
constexpr uint16_t MATRIX_MAX_HEIGHT = 8;
constexpr uint16_t MATRIX_MAX_LEDS = MATRIX_MAX_WIDTH * MATRIX_MAX_HEIGHT;
constexpr uint16_t DEFAULT_MATRIX_WIDTH = 32;
constexpr uint16_t DEFAULT_MATRIX_HEIGHT = 8;
bool displayBegin(int8_t pin);
void displaySetMatrixSize(uint16_t width, uint16_t height);
void displaySetText(const String &text);
void displaySetLeds(DisplayMode mode, uint32_t color);
void displaySetLedCount(uint32_t color, uint16_t count);
void displaySetBrightness(uint8_t brightness);
void displaySetColor(uint32_t color);
void displaySetSerpentine(bool enabled);
void displaySetOrientation(bool flipped);
uint16_t xyToIndex(uint8_t x, uint8_t y);
void scrollSetDirection(ScrollDirection direction);
void scrollSetSpeed(uint16_t milliseconds);
void scrollStart();
void scrollStop();
void maintainDisplay();
// Immediately draws a centered Wi-Fi "pie" icon, used to indicate an active
// OTA firmware update (ArduinoOTA or the web /update upload). Bypasses the
// normal mode/dirty logic so it is visible even while the main loop is busy
// servicing the transfer.
void displayShowOtaIcon();
// Forces the next maintainDisplay() call to fully redraw, used to restore
// the normal display after an OTA attempt fails (a successful update
// reboots the device, so no restore is needed in that case).
void displayForceRedraw();
}
