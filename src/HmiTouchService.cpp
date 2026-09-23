#if !defined(F103_BUILD_BOOTLOADER)
#include "HmiTouchService.h"
#include "HmiConfig.h"
#include "HmiDisplay.h"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cstdlib>

extern HmiDisplay tft;

namespace {
using HmiTouchService::Mode;
HmiTouchService::EmitFn gEmit = nullptr;
HmiTouchService::PersistFn gPersist = nullptr;
Mode gMode = Mode::NORMAL;
bool gWasDown = false;
bool gRedraw = false;
uint32_t gLastRawReportMs = 0U;
uint32_t gModeStartedMs = 0U;
uint8_t gStep = 0U;
uint16_t gRawX[5]{};
uint16_t gRawY[5]{};
constexpr int16_t kTargetX[5] = {20, 299, 20, 299, 160};
constexpr int16_t kTargetY[5] = {20, 20, 219, 219, 120};
constexpr int32_t kRawEdgeGuard = 24;
constexpr uint16_t kMinSpan = 500U;
constexpr uint32_t kRawModeTimeoutMs = 60000U;
constexpr uint32_t kCalibrationTimeoutMs = 120000U;

void emit(const char *line) { if (gEmit != nullptr && line != nullptr) gEmit(line); }

struct RawAxisCalibration {
  uint16_t origin{0U};
  uint16_t span{0U};
  bool inverted{false};
};

bool projectAxisToPanelEdges(int32_t rawAtFirstTarget, int32_t rawAtLastTarget,
                             int32_t firstTargetPx, int32_t lastTargetPx,
                             int32_t screenMaxPx, RawAxisCalibration &out) {
  const int32_t pixelSpan = lastTargetPx - firstTargetPx;
  const int32_t rawDelta = rawAtLastTarget - rawAtFirstTarget;
  if (pixelSpan <= 0 || std::abs(rawDelta) < static_cast<int32_t>(kMinSpan))
    return false;

  // Corner targets sit 20 px inside the LCD. Extrapolate the measured line to
  // the physical screen edges instead of treating those inner samples as the
  // raw minima/maxima; otherwise the outer touch band is mapped out-of-bounds.
  const int32_t rawAtZero = rawAtFirstTarget -
      (rawDelta * firstTargetPx) / pixelSpan;
  const int32_t rawAtScreenMax = rawAtLastTarget +
      (rawDelta * (screenMaxPx - lastTargetPx)) / pixelSpan;
  int32_t low = std::min(rawAtZero, rawAtScreenMax) - kRawEdgeGuard;
  int32_t high = std::max(rawAtZero, rawAtScreenMax) + kRawEdgeGuard;
  low = std::clamp<int32_t>(low, 0, 4095);
  high = std::clamp<int32_t>(high, 0, 4095);
  if (high - low < static_cast<int32_t>(kMinSpan))
    return false;

  out.origin = static_cast<uint16_t>(low);
  out.span = static_cast<uint16_t>(high - low);
  out.inverted = rawAtScreenMax < rawAtZero;
  return true;
}

void drawTarget() {
  tft.beginFrame();
  tft.fillScreen(C_BG);
  tft.drawCircle(kTargetX[gStep], kTargetY[gStep], 9, C_WARNING);
  tft.drawFastHLine(kTargetX[gStep]-14, kTargetY[gStep], 29, C_WARNING);
  tft.drawFastVLine(kTargetX[gStep], kTargetY[gStep]-14, 29, C_WARNING);
  tft.setTextColor(C_TEXT, C_BG); tft.setTextDatum(TC_DATUM); tft.setTextSize(1);
  char line[48]; std::snprintf(line, sizeof(line), "TOUCH CAL %u/5", unsigned(gStep+1U));
  tft.drawString(line, 160, 92);
  tft.drawString("PRESS TARGET, THEN RELEASE", 160, 105);
  tft.endFrame();
}
void finishCalibration() {
  // Infer panel orientation from the four corner samples. The targets are
  // intentionally inset from the bezel, so the raw ranges below are projected
  // back out to screen x=0..319 and y=0..239 before being stored.
  const int32_t leftX=(int32_t(gRawX[0])+gRawX[2])/2, rightX=(int32_t(gRawX[1])+gRawX[3])/2;
  const int32_t leftY=(int32_t(gRawY[0])+gRawY[2])/2, rightY=(int32_t(gRawY[1])+gRawY[3])/2;
  const int32_t topX=(int32_t(gRawX[0])+gRawX[1])/2, bottomX=(int32_t(gRawX[2])+gRawX[3])/2;
  const int32_t topY=(int32_t(gRawY[0])+gRawY[1])/2, bottomY=(int32_t(gRawY[2])+gRawY[3])/2;
  const bool rotate = std::abs(rightY-leftY) > std::abs(rightX-leftX) &&
                      std::abs(bottomX-topX) > std::abs(bottomY-topY);

  RawAxisCalibration screenX{}, screenY{};
  const int32_t rawXAtLeft = rotate ? leftY : leftX;
  const int32_t rawXAtRight = rotate ? rightY : rightX;
  const int32_t rawYAtTop = rotate ? topX : topY;
  const int32_t rawYAtBottom = rotate ? bottomX : bottomY;
  if (!projectAxisToPanelEdges(rawXAtLeft, rawXAtRight,
                               kTargetX[0], kTargetX[1], W - 1, screenX) ||
      !projectAxisToPanelEdges(rawYAtTop, rawYAtBottom,
                               kTargetY[0], kTargetY[2], H - 1, screenY)) {
    emit("ERR:TOUCH:CAL:SPAN"); gMode=Mode::NORMAL; gRedraw=true; return;
  }

  const uint16_t flags = uint16_t((rotate?1U:0U) |
                                  (screenX.inverted?2U:0U) |
                                  (screenY.inverted?4U:0U));
  const uint16_t p[5] = {screenX.origin, screenX.span,
                         screenY.origin, screenY.span, flags};
  tft.setTouch(p);
  const bool persisted = gPersist == nullptr || gPersist(p);
  if (!persisted)
    emit("WARN:TOUCH:CAL:NOT_PERSISTED");
  char line[170];
  std::snprintf(line,sizeof(line),
                "ACK:TOUCH:CAL:XRAW=%u-%u:YRAW=%u-%u:CENTER=%u,%u:FLAGS=%u:PERSIST=%u",
                unsigned(screenX.origin),unsigned(screenX.origin+screenX.span),
                unsigned(screenY.origin),unsigned(screenY.origin+screenY.span),
                unsigned(gRawX[4]),unsigned(gRawY[4]),unsigned(flags),persisted?1U:0U);
  emit(line); gMode=Mode::NORMAL; gModeStartedMs=0U; gRedraw=true;
}
} // namespace

namespace HmiTouchService {
void begin(EmitFn emitFn, PersistFn persistFn) {
  gEmit=emitFn; gPersist=persistFn; gMode=Mode::NORMAL; gWasDown=false;
  gRedraw=false; gModeStartedMs=0U;
}
bool handleCommand(const char *command) {
  if (command == nullptr) return false;
  if (!std::strcmp(command,"RAWTOUCH") || !std::strcmp(command,"TOUCHTEST") ||
      !std::strcmp(command,"WINCH TOUCHTEST")) {
    if (gMode == Mode::CALIBRATION) { emit("ERR:TOUCH:RAW:CAL_ACTIVE"); return true; }
    gMode = gMode == Mode::RAW ? Mode::NORMAL : Mode::RAW;
    gModeStartedMs = gMode == Mode::NORMAL ? 0U : HAL_GetTick();
    gWasDown=false; gRedraw = gMode == Mode::NORMAL;
    emit(gMode == Mode::RAW ? "ACK:TOUCH:RAW:START" : "ACK:TOUCH:RAW:STOP");
    return true;
  }
  if (!std::strcmp(command,"CALIBRATE") || !std::strcmp(command,"TOUCHCAL")) {
    if (gMode == Mode::RAW) { emit("ERR:TOUCH:CAL:RAW_ACTIVE"); return true; }
    gMode=Mode::CALIBRATION; gModeStartedMs=HAL_GetTick(); gWasDown=false; gStep=0U; drawTarget();
    emit("ACK:TOUCH:CAL:START"); return true;
  }
  if (!std::strcmp(command,"CALSTOP")) {
    cancel(); emit("ACK:TOUCH:CAL:STOP"); return true;
  }
  return false;
}
void service() {
  if (gMode == Mode::NORMAL) return;
  const uint32_t now=HAL_GetTick();
  const uint32_t limit = gMode == Mode::RAW ? kRawModeTimeoutMs : kCalibrationTimeoutMs;
  if (gModeStartedMs != 0U && static_cast<uint32_t>(now-gModeStartedMs) >= limit) {
    emit(gMode == Mode::RAW ? "WARN:TOUCH:RAW:TIMEOUT" : "WARN:TOUCH:CAL:TIMEOUT");
    cancel();
    return;
  }
  uint16_t x=0U,y=0U; const bool down=tft.getTouch(&x,&y,TOUCH_THRESHOLD);
  if (gMode == Mode::RAW) {
    if (down && static_cast<uint32_t>(now-gLastRawReportMs)>=100U) {
      gLastRawReportMs=now; char line[120];
      std::snprintf(line,sizeof(line),"TOUCH:RAW:Z=%u:X=%u:Y=%u:SX=%u:SY=%u",
                    unsigned(tft.touchCurrentZ()),unsigned(tft.touchLastRawX()),
                    unsigned(tft.touchLastRawY()),unsigned(x),unsigned(y)); emit(line);
    }
    return;
  }
  if (down && !gWasDown) gWasDown=true;
  if (!down && gWasDown) {
    gWasDown=false;
    gRawX[gStep]=tft.touchLastRawX(); gRawY[gStep]=tft.touchLastRawY();
    if (++gStep >= 5U) { finishCalibration(); return; }
    drawTarget();
  }
}
void cancel() { gMode=Mode::NORMAL; gModeStartedMs=0U; gWasDown=false; gRedraw=true; }
bool active() { return gMode != Mode::NORMAL; }
bool consumeRedrawRequest() { const bool v=gRedraw; gRedraw=false; return v; }
const char *modeName() { return gMode==Mode::RAW?"RAW":(gMode==Mode::CALIBRATION?"CAL":"NORMAL"); }
} // namespace HmiTouchService

#endif
