#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace emcomm {

inline int macHexValue(char value) {
  if (value >= '0' && value <= '9')
    return value - '0';
  if (value >= 'A' && value <= 'F')
    return value - 'A' + 10;
  if (value >= 'a' && value <= 'f')
    return value - 'a' + 10;
  return -1;
}

inline bool parseMacAddress(const char *text, uint8_t out[6]) {
  if (text == nullptr || out == nullptr || strlen(text) != 17)
    return false;

  uint8_t parsed[6];
  for (uint8_t i = 0; i < 6; ++i) {
    const size_t offset = static_cast<size_t>(i) * 3;
    if (i < 5 && text[offset + 2] != ':')
      return false;
    const int high = macHexValue(text[offset]);
    const int low = macHexValue(text[offset + 1]);
    if (high < 0 || low < 0)
      return false;
    parsed[i] = static_cast<uint8_t>((high << 4) | low);
  }
  for (uint8_t i = 0; i < 6; ++i)
    out[i] = parsed[i];
  return true;
}

} // namespace emcomm
