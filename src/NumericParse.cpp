#include "NumericParse.h"
#include <cstdint>

namespace CompactParse {
static bool digitValue(char c, uint8_t base, uint8_t &digit) {
  if (c >= '0' && c <= '9') digit = static_cast<uint8_t>(c - '0');
  else if (c >= 'A' && c <= 'F') digit = static_cast<uint8_t>(c - 'A' + 10);
  else if (c >= 'a' && c <= 'f') digit = static_cast<uint8_t>(c - 'a' + 10);
  else return false;
  return digit < base;
}

static bool unsignedBase(const char *s, uint8_t base, uint32_t &out) {
  if (s == nullptr || *s == '\0') return false;
  uint32_t value = 0U;
  for (const char *p = s; *p != '\0'; ++p) {
    uint8_t digit = 0U;
    if (!digitValue(*p, base, digit)) return false;
    if (value > (0xFFFFFFFFUL - digit) / base) return false;
    value = value * base + digit;
  }
  out = value;
  return true;
}

bool u32(const char *s, uint32_t &out) { return unsignedBase(s, 10U, out); }
bool hex32(const char *s, uint32_t &out) { return unsignedBase(s, 16U, out); }

bool i32(const char *s, int32_t &out) {
  if (s == nullptr || *s == '\0') return false;
  const bool negative = *s == '-';
  if (*s == '-' || *s == '+') ++s;
  if (*s == '\0') return false;
  uint32_t magnitude = 0U;
  if (!u32(s, magnitude)) return false;
  const uint32_t limit = negative ? 0x80000000UL : 0x7FFFFFFFUL;
  if (magnitude > limit) return false;
  out = negative ? (magnitude == 0x80000000UL
                        ? static_cast<int32_t>(0x80000000UL)
                        : -static_cast<int32_t>(magnitude))
                 : static_cast<int32_t>(magnitude);
  return true;
}

bool real(const char *s, double &out) {
  if (s == nullptr || *s == '\0') return false;
  bool negative = false;
  if (*s == '-' || *s == '+') { negative = *s == '-'; ++s; }
  bool any = false;
  double value = 0.0;
  while (*s >= '0' && *s <= '9') {
    any = true; value = value * 10.0 + static_cast<double>(*s++ - '0');
  }
  if (*s == '.') {
    ++s; double scale = 0.1;
    while (*s >= '0' && *s <= '9') {
      any = true; value += static_cast<double>(*s++ - '0') * scale; scale *= 0.1;
    }
  }
  if (!any) return false;
  int exponent = 0;
  if (*s == 'e' || *s == 'E') {
    ++s; bool expNeg = false;
    if (*s == '-' || *s == '+') { expNeg = *s == '-'; ++s; }
    if (*s < '0' || *s > '9') return false;
    while (*s >= '0' && *s <= '9') {
      if (exponent > 100) return false;
      exponent = exponent * 10 + (*s++ - '0');
    }
    if (expNeg) exponent = -exponent;
  }
  if (*s != '\0' || exponent > 38 || exponent < -38) return false;
  while (exponent > 0) { value *= 10.0; --exponent; }
  while (exponent < 0) { value *= 0.1; ++exponent; }
  out = negative ? -value : value;
  return true;
}

bool realf(const char *s, float &out) {
  if (s == nullptr || *s == '\0') return false;
  bool negative = false;
  if (*s == '-' || *s == '+') { negative = *s == '-'; ++s; }
  bool any = false;
  float value = 0.0F;
  while (*s >= '0' && *s <= '9') {
    any = true;
    value = value * 10.0F + static_cast<float>(*s++ - '0');
  }
  if (*s == '.') {
    ++s;
    float scale = 0.1F;
    while (*s >= '0' && *s <= '9') {
      any = true;
      value += static_cast<float>(*s++ - '0') * scale;
      scale *= 0.1F;
    }
  }
  if (!any) return false;
  int exponent = 0;
  if (*s == 'e' || *s == 'E') {
    ++s;
    bool expNeg = false;
    if (*s == '-' || *s == '+') { expNeg = *s == '-'; ++s; }
    if (*s < '0' || *s > '9') return false;
    while (*s >= '0' && *s <= '9') {
      if (exponent > 100) return false;
      exponent = exponent * 10 + (*s++ - '0');
    }
    if (expNeg) exponent = -exponent;
  }
  if (*s != '\0' || exponent > 38 || exponent < -38) return false;
  while (exponent > 0) { value *= 10.0F; --exponent; }
  while (exponent < 0) { value *= 0.1F; ++exponent; }
  out = negative ? -value : value;
  return true;
}
} // namespace CompactParse
