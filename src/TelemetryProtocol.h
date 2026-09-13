#pragma once

#include "Telemetry.h"
#include <cstdint>

enum class ExtendedTelemetryDomain : uint8_t {
  NONE = 0U,
  ESC,
  PERCEPTION,
  NAVIGATION
};

struct ExtendedTelemetryParseResult {
  bool recognized{false};
  bool accepted{false};
  bool outOfOrder{false};
  bool v3{false};
  bool crcError{false};
  bool lengthError{false};
  bool versionError{false};
  bool sessionError{false};
  bool sessionChanged{false};
  ExtendedTelemetryDomain domain{ExtendedTelemetryDomain::NONE};
};

ExtendedTelemetryParseResult parseExtendedTelemetryLine(
    const char *line, VehicleTelemetry &telemetry, uint32_t nowMs,
    uint32_t expectedSession = 0U);
