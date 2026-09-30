#pragma once

#include <stddef.h>

namespace emcomm {

template <typename StringType>
StringType htmlEscape(const StringType &value) {
  StringType escaped;
  escaped.reserve(value.length() + 16);
  for (size_t i = 0; i < value.length(); ++i) {
    switch (value[i]) {
    case '&':
      escaped += "&amp;";
      break;
    case '<':
      escaped += "&lt;";
      break;
    case '>':
      escaped += "&gt;";
      break;
    case '"':
      escaped += "&quot;";
      break;
    case '\'':
      escaped += "&#39;";
      break;
    default:
      escaped += value[i];
      break;
    }
  }
  return escaped;
}

} // namespace emcomm
