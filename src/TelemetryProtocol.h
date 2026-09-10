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
  ExtendedTelemetryDomain domain{ExtendedTelemetryDomain::NONE};
};

ExtendedTelemetryParseResult parseExtendedTelemetryLine(
    const char *line, VehicleTelemetry &telemetry, uint32_t nowMs);
