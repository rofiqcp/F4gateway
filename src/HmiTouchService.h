#pragma once
#include <cstdint>

namespace HmiTouchService {
using EmitFn = void (*)(const char *line);

enum class Mode : uint8_t { NORMAL = 0U, RAW, CALIBRATION };

void begin(EmitFn emit);
bool handleCommand(const char *command);
void service();
void cancel();
bool active();
bool consumeRedrawRequest();
const char *modeName();
} // namespace HmiTouchService
