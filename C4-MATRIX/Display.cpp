#include "Display.h"
#include "Config.h"
#include "Debug.h"
#include "NtpClock.h"
#include "OtaService.h"
#include <Adafruit_NeoPixel.h>

namespace c4matrix {
static uint16_t matrixWidth = DEFAULT_MATRIX_WIDTH;
static uint16_t matrixHeight = DEFAULT_MATRIX_HEIGHT;
static uint16_t matrixLedCount = DEFAULT_MATRIX_WIDTH * DEFAULT_MATRIX_HEIGHT;
static Adafruit_NeoPixel strip(matrixLedCount, 16, NEO_GRB + NEO_KHZ800);
static bool stripReady = false;
static bool scrolling = false;
static bool displayDirty = true;
static unsigned long lastScrollStep = 0;
static uint16_t scrollOffset = 0;

const char *displayModeName(DisplayMode mode) {
  switch (mode) {
  case DisplayMode::Fill:
    return "fill";
  case DisplayMode::Off:
    return "off";
  case DisplayMode::Count:
    return "count";
  case DisplayMode::Clock:
    return "clock";
  default:
    return "text";
  }
}

uint16_t displayLedCount() { return matrixLedCount; }
uint16_t displayMatrixWidth() { return matrixWidth; }
uint16_t displayMatrixHeight() { return matrixHeight; }

static const uint8_t *glyphFor(char character) {
  static const uint8_t question[5] = {0x02, 0x01, 0x51, 0x09, 0x06};
  static const uint8_t digits[10][5] = {
      {0x3E, 0x51, 0x49, 0x45, 0x3E}, {0x00, 0x42, 0x7F, 0x40, 0x00},
      {0x42, 0x61, 0x51, 0x49, 0x46}, {0x21, 0x41, 0x45, 0x4B, 0x31},
      {0x18, 0x14, 0x12, 0x7F, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39},
      {0x3C, 0x4A, 0x49, 0x49, 0x30}, {0x01, 0x71, 0x09, 0x05, 0x03},
      {0x36, 0x49, 0x49, 0x49, 0x36}, {0x06, 0x49, 0x49, 0x29, 0x1E}};
  static const uint8_t letters[26][5] = {
      {0x7E, 0x11, 0x11, 0x11, 0x7E}, {0x7F, 0x49, 0x49, 0x49, 0x36},
      {0x3E, 0x41, 0x41, 0x41, 0x22}, {0x7F, 0x41, 0x41, 0x22, 0x1C},
      {0x7F, 0x49, 0x49, 0x49, 0x41}, {0x7F, 0x09, 0x09, 0x09, 0x01},
      {0x3E, 0x41, 0x49, 0x49, 0x7A}, {0x7F, 0x08, 0x08, 0x08, 0x7F},
      {0x00, 0x41, 0x7F, 0x41, 0x00}, {0x20, 0x40, 0x41, 0x3F, 0x01},
      {0x7F, 0x08, 0x14, 0x22, 0x41}, {0x7F, 0x40, 0x40, 0x40, 0x40},
      {0x7F, 0x02, 0x0C, 0x02, 0x7F}, {0x7F, 0x04, 0x08, 0x10, 0x7F},
      {0x3E, 0x41, 0x41, 0x41, 0x3E}, {0x7F, 0x09, 0x09, 0x09, 0x06},
      {0x3E, 0x41, 0x51, 0x21, 0x5E}, {0x7F, 0x09, 0x19, 0x29, 0x46},
      {0x46, 0x49, 0x49, 0x49, 0x31}, {0x01, 0x01, 0x7F, 0x01, 0x01},
      {0x3F, 0x40, 0x40, 0x40, 0x3F}, {0x1F, 0x20, 0x40, 0x20, 0x1F},
      {0x7F, 0x20, 0x18, 0x20, 0x7F}, {0x63, 0x14, 0x08, 0x14, 0x63},
      {0x03, 0x04, 0x78, 0x04, 0x03}, {0x61, 0x51, 0x49, 0x45, 0x43}};
  static const uint8_t lowercase[26][5] = {
      {0x20, 0x54, 0x54, 0x54, 0x78}, {0x7F, 0x48, 0x44, 0x44, 0x38},
      {0x38, 0x44, 0x44, 0x44, 0x20}, {0x38, 0x44, 0x44, 0x48, 0x7F},
      {0x38, 0x54, 0x54, 0x54, 0x18}, {0x08, 0x7E, 0x09, 0x01, 0x02},
      {0x0C, 0x52, 0x52, 0x52, 0x3E}, {0x7F, 0x08, 0x04, 0x04, 0x78},
      {0, 0x44, 0x7D, 0x40, 0}, {0x20, 0x40, 0x44, 0x3D, 0},
      {0x7F, 0x10, 0x28, 0x44, 0}, {0, 0x41, 0x7F, 0x40, 0},
      {0x7C, 0x04, 0x18, 0x04, 0x78}, {0x7C, 0x08, 0x04, 0x04, 0x78},
      {0x38, 0x44, 0x44, 0x44, 0x38}, {0x7C, 0x14, 0x14, 0x14, 0x08},
      {0x08, 0x14, 0x14, 0x18, 0x7C}, {0x7C, 0x08, 0x04, 0x04, 0x08},
      {0x48, 0x54, 0x54, 0x54, 0x20}, {0x04, 0x3F, 0x44, 0x40, 0x20},
      {0x3C, 0x40, 0x40, 0x20, 0x7C}, {0x1C, 0x20, 0x40, 0x20, 0x1C},
      {0x3C, 0x40, 0x30, 0x40, 0x3C}, {0x44, 0x28, 0x10, 0x28, 0x44},
      {0x0C, 0x50, 0x50, 0x50, 0x3C}, {0x44, 0x64, 0x54, 0x4C, 0x44}};
  static const uint8_t atSign[5] = {0x32, 0x49, 0x79, 0x41, 0x3E};
  static const uint8_t leftBracket[5] = {0, 0x7F, 0x41, 0x41, 0};
  static const uint8_t backslash[5] = {2, 4, 8, 0x10, 0x20};
  static const uint8_t rightBracket[5] = {0, 0x41, 0x41, 0x7F, 0};
  static const uint8_t caret[5] = {4, 2, 1, 2, 4};
  static const uint8_t underscore[5] = {0x40, 0x40, 0x40, 0x40, 0x40};
  static const uint8_t grave[5] = {0, 1, 2, 4, 0};
  static const uint8_t leftBrace[5] = {8, 0x36, 0x41, 0, 0};
  static const uint8_t verticalBar[5] = {0, 0, 0x7F, 0, 0};
  static const uint8_t rightBrace[5] = {0, 0x41, 0x36, 8, 0};
  static const uint8_t tilde[5] = {8, 4, 8, 0x10, 8};
  static const uint8_t punctuation[32][5] = {
      {0, 0, 0, 0, 0},       {0, 0, 0x5F, 0, 0},       {0, 7, 0, 7, 0},
      {0x14, 0x7F, 0x14, 0x7F, 0x14}, {0x24, 0x2A, 0x7F, 0x2A, 0x12},
      {0x23, 0x13, 8, 0x64, 0x62},    {0x36, 0x49, 0x55, 0x22, 0x50},
      {0, 5, 3, 0, 0},       {0, 0x1C, 0x22, 0x41, 0},
      {0, 0x41, 0x22, 0x1C, 0},       {0x14, 8, 0x3E, 8, 0x14},
      {8, 8, 0x3E, 8, 8},    {0, 0x50, 0x30, 0, 0},       {8, 8, 8, 8, 8},
      {0, 0x60, 0x60, 0, 0}, {0x20, 0x10, 8, 4, 2},
      {0x3E, 0x51, 0x49, 0x45, 0x3E}, {0x42, 0x61, 0x51, 0x49, 0x46},
      {0x21, 0x41, 0x45, 0x4B, 0x31}, {0x18, 0x14, 0x12, 0x7F, 0x10},
      {0x27, 0x45, 0x45, 0x45, 0x39}, {0x3C, 0x4A, 0x49, 0x49, 0x30},
      {0x01, 0x71, 0x09, 0x05, 0x03}, {0x36, 0x49, 0x49, 0x49, 0x36},
      {0x06, 0x49, 0x49, 0x29, 0x1E}, {0, 0x36, 0x36, 0, 0},
      {0, 0x36, 0x36, 0, 0}, {8, 0x14, 0x22, 0x41, 0},
      {0x14, 0x14, 0x14, 0x14, 0x14}, {0, 0x41, 0x22, 0x14, 8},
      {2, 1, 0x51, 9, 6}, {0x32, 0x49, 0x79, 0x41, 0x3E}};
  if (character >= '0' && character <= '9')
    return digits[character - '0'];
  if (character >= 'A' && character <= 'Z')
    return letters[character - 'A'];
  if (character >= 'a' && character <= 'z')
    return lowercase[character - 'a'];
  if (character >= ' ' && character <= '?')
    return punctuation[character - ' '];
  if (character == '@')
    return atSign;
  if (character == '[')
    return leftBracket;
  if (character == '\\')
    return backslash;
  if (character == ']')
    return rightBracket;
  if (character == '^')
    return caret;
  if (character == '_')
    return underscore;
  if (character == '`')
    return grave;
  if (character == '{')
    return leftBrace;
  if (character == '|')
    return verticalBar;
  if (character == '}')
    return rightBrace;
  if (character == '~')
    return tilde;
  return question;
}

static uint32_t pixelColor() {
  return strip.Color((cfg.textColor >> 16) & 0xFF, (cfg.textColor >> 8) & 0xFF,
                     cfg.textColor & 0xFF);
}

uint16_t xyToIndex(uint8_t x, uint8_t y) {
  if (x >= matrixWidth || y >= matrixHeight)
    return 0;
  if (cfg.flipHorizontal)
    x = matrixWidth - 1 - x;
  uint8_t row = (cfg.serpentine && (x & 1)) ? matrixHeight - 1 - y : y;
  return static_cast<uint16_t>(x) * matrixHeight + row;
}

static uint16_t textWidth(const String &text) {
  return text.isEmpty() ? 0 : static_cast<uint16_t>(text.length() * 6 - 1);
}

static void renderText(const String &text, int16_t originX) {
  if (!stripReady)
    return;
  strip.clear();
  uint32_t color = pixelColor();
  for (size_t characterIndex = 0; characterIndex < text.length();
       ++characterIndex) {
    const uint8_t *glyph = glyphFor(text[characterIndex]);
    int16_t characterX = originX + static_cast<int16_t>(characterIndex * 6);
    for (uint8_t column = 0; column < 5; ++column) {
      int16_t x = characterX + column;
      if (x < 0 || x >= matrixWidth)
        continue;
      for (uint8_t y = 0; y < matrixHeight; ++y) {
        if (glyph[column] & (1U << y))
          strip.setPixelColor(xyToIndex(static_cast<uint8_t>(x), y), color);
      }
    }
  }
  strip.show();
}

// Column-major 8x8 bitmap of a Wi-Fi "pie" signal icon (two concentric
// arcs plus a dot), bit 0 = top row, matching the font glyph convention
// used elsewhere in this file.
static const uint8_t WIFI_ICON[8] = {0x04, 0x16, 0x0A, 0xAA,
                                      0xAA, 0x0A, 0x16, 0x04};

static void renderWifiIcon() {
  if (!stripReady)
    return;
  strip.clear();
  uint32_t color = pixelColor();
  int16_t originX = matrixWidth > 8 ? (matrixWidth - 8) / 2 : 0;
  for (uint8_t column = 0; column < 8; ++column) {
    int16_t x = originX + column;
    if (x < 0 || x >= matrixWidth)
      continue;
    for (uint8_t y = 0; y < matrixHeight && y < 8; ++y) {
      if (WIFI_ICON[column] & (1U << y))
        strip.setPixelColor(xyToIndex(static_cast<uint8_t>(x), y), color);
    }
  }
  strip.show();
}

void displayShowOtaIcon() { renderWifiIcon(); }

void displayForceRedraw() {
  displayDirty = true;
  maintainDisplay();
}

bool displayBegin(int8_t pin) {
  matrixWidth = cfg.matrixWidth;
  matrixHeight = cfg.matrixHeight;
  matrixLedCount = static_cast<uint16_t>(matrixWidth) * matrixHeight;
  strip.updateLength(matrixLedCount);
  cfg.displayPin = pin;
  if (stripReady)
    strip.updateType(NEO_GRB + NEO_KHZ800);
  strip.setPin(pin);
  strip.begin();
  stripReady = true;
  strip.setBrightness(cfg.brightness);
  strip.clear();
  strip.show();
  displayDirty = true;
  logStatus(String(F("Matrix initialized on GPIO")) + String(pin) + F(" (") +
            String(matrixWidth) + F("x") + String(matrixHeight) + F(")"));
  scrollOffset = 0;
  lastScrollStep = millis();
  maintainDisplay();
  return true;
}

void displaySetMatrixSize(uint16_t width, uint16_t height) {
  matrixWidth = width;
  matrixHeight = height;
  matrixLedCount = static_cast<uint16_t>(width) * height;
  strip.updateLength(matrixLedCount);
  if (cfg.ledCount > matrixLedCount)
    cfg.ledCount = matrixLedCount;
  displayDirty = true;
  DBG("Matrix size set to %ux%u (%u LEDs)", width, height, matrixLedCount);
  maintainDisplay();
}

void displaySetText(const String &text) {
  cfg.text = text;
  cfg.mode = DisplayMode::Text;
  scrollOffset = 0;
  lastScrollStep = millis();
  displayDirty = true;
  logStatus(String(F("Display text updated (")) + String(text.length()) +
            F(" characters)."));
  DBG("Text content: %s", text.c_str());
  maintainDisplay();
}
void displaySetLeds(DisplayMode mode, uint32_t color) {
  cfg.mode = mode;
  cfg.fillColor = color & 0xFFFFFFUL;
  displayDirty = true;
  DBG("LED mode: %s, fill color: #%06lX", displayModeName(cfg.mode),
      static_cast<unsigned long>(cfg.fillColor));
  maintainDisplay();
}
void displaySetLedCount(uint32_t color, uint16_t count) {
  cfg.mode = DisplayMode::Count;
  cfg.fillColor = color & 0xFFFFFFUL;
  cfg.ledCount = count > matrixLedCount ? matrixLedCount : count;
  displayDirty = true;
  DBG("LED mode: count, LEDs lit: %u, fill color: #%06lX", cfg.ledCount,
      static_cast<unsigned long>(cfg.fillColor));
  maintainDisplay();
}
void displaySetBrightness(uint8_t brightness) {
  cfg.brightness = brightness;
  if (stripReady)
    strip.setBrightness(brightness);
  displayDirty = true;
  maintainDisplay();
}
void displaySetColor(uint32_t color) {
  cfg.textColor = color & 0xFFFFFFUL;
  displayDirty = true;
  maintainDisplay();
}
void displaySetSerpentine(bool enabled) {
  cfg.serpentine = enabled;
  displayDirty = true;
  maintainDisplay();
}
void displaySetOrientation(bool flipped) {
  cfg.flipHorizontal = flipped;
  displayDirty = true;
  maintainDisplay();
}
void scrollSetDirection(ScrollDirection direction) {
  cfg.scrollRight = direction == ScrollDirection::Right;
  scrollOffset = 0;
  displayDirty = true;
  logStatus(cfg.scrollRight ? F("Scroll direction: right.")
                            : F("Scroll direction: left."));
  maintainDisplay();
}
void scrollSetSpeed(uint16_t milliseconds) {
  cfg.scrollSpeed = constrain(milliseconds, 10, 1000);
  logStatus(String(F("Scroll speed: ")) + String(cfg.scrollSpeed) +
            F(" ms/column."));
}
void scrollStart() {
  scrolling = true;
  cfg.scrollEnabled = true;
  scrollOffset = 0;
  lastScrollStep = millis();
  displayDirty = true;
  logStatus(F("Text scrolling started."));
  maintainDisplay();
}
void scrollStop() {
  scrolling = false;
  cfg.scrollEnabled = false;
  displayDirty = true;
  logStatus(F("Text scrolling stopped."));
  maintainDisplay();
}
void maintainDisplay() {
  if (!stripReady)
    return;
  if (otaTransferInProgress) {
    renderWifiIcon();
    return;
  }
  if (cfg.mode == DisplayMode::Clock) {
    // Clock text ("HH:MM") is recomputed periodically rather than stored in
    // cfg.text, so it never overwrites the user's saved custom text. The
    // colon blinks at 1Hz (500ms visible / 500ms hidden).
    static String lastClockText;
    static unsigned long lastClockCheck = 0;
    unsigned long nowMillis = millis();
    if (nowMillis - lastClockCheck >= 500 || lastClockText.isEmpty()) {
      lastClockCheck = nowMillis;
      bool colonVisible = (nowMillis / 500) % 2 == 0;
      String clockText = ntpTimeString(colonVisible);
      if (clockText != lastClockText) {
        lastClockText = clockText;
        displayDirty = true;
      }
    }
    if (!displayDirty)
      return;
    uint16_t clockWidth = textWidth(lastClockText);
    int16_t clockX = clockWidth <= matrixWidth
                          ? (matrixWidth - static_cast<int16_t>(clockWidth)) / 2
                          : 0;
    renderText(lastClockText, clockX);
    displayDirty = false;
    return;
  }
  if (cfg.mode != DisplayMode::Text) {
    if (!displayDirty)
      return;
    if (cfg.mode == DisplayMode::Count) {
      strip.clear();
      uint16_t litCount = cfg.ledCount > matrixLedCount ? matrixLedCount : cfg.ledCount;
      for (uint16_t i = 0; i < litCount; ++i)
        strip.setPixelColor(i, cfg.fillColor);
    } else {
      strip.fill(cfg.mode == DisplayMode::Fill ? cfg.fillColor : 0);
    }
    strip.show();
    displayDirty = false;
    return;
  }
  uint16_t width = textWidth(cfg.text);
  if (scrolling && width > matrixWidth) {
    unsigned long now = millis();
    bool step = false;
    if (now - lastScrollStep >= cfg.scrollSpeed) {
      lastScrollStep = now;
      ++scrollOffset;
      if (scrollOffset > width + matrixWidth)
        scrollOffset = 0;
      step = true;
    }
    if (!displayDirty && !step)
      return;
    int16_t x = cfg.scrollRight
                    ? -static_cast<int16_t>(width) + scrollOffset
                    : matrixWidth - static_cast<int16_t>(scrollOffset);
    renderText(cfg.text, x);
    displayDirty = false;
    return;
  }
  if (!displayDirty)
    return;
  int16_t x = width <= matrixWidth
                  ? (matrixWidth - static_cast<int16_t>(width)) / 2
                  : 0;
  renderText(cfg.text, x);
  displayDirty = false;
}
}
