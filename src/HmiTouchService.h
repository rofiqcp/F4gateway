#pragma once
#include <cstdint>

namespace HmiTouchService {
using EmitFn = void (*)(const char *line);
using PersistFn = bool (*)(const uint16_t *parameters);

enum class Mode : uint8_t { NORMAL = 0U, RAW, CALIBRATION };

void begin(EmitFn emit, PersistFn persist = nullptr);
bool handleCommand(const char *command);
void service();
void cancel();
bool active();
bool consumeRedrawRequest();
const char *modeName();
} // namespace HmiTouchService
