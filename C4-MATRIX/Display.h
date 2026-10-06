#pragma once
#include <Arduino.h>

namespace c4matrix {
enum class DisplayMode : uint8_t { Text, Fill, Off };
enum class ScrollDirection : uint8_t { Left, Right };
const char *displayModeName(DisplayMode mode);
bool displayBegin(int8_t pin);
void displaySetText(const String &text);
void displaySetLeds(DisplayMode mode, uint32_t color);
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
}
