// Host-side stub: just enough of the Arduino String for tcs_protocol.cpp
#pragma once
#include <cstdint>
#include <cstdio>
#include <string>

class String : public std::string {
public:
    String(const char *s = "") : std::string(s) {}
    String(const std::string &s) : std::string(s) {}
    String(int v) : std::string(std::to_string(v)) {}
    String substring(unsigned from) const { return String(substr(from)); }
};

inline String operator+(const char *a, const String &b) { return String(std::string(a) + b); }
