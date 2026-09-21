#pragma once
#include <cstdint>

namespace CompactParse {
bool u32(const char *s, uint32_t &out);
bool hex32(const char *s, uint32_t &out);
bool i32(const char *s, int32_t &out);
bool real(const char *s, double &out);
bool realf(const char *s, float &out);
}
