// ============================================================================
// TouchButtons.h — input layer page-aware untuk kartu besar, editor, dan test.
// ============================================================================
#pragma once

#include "HmiDisplay.h"
#include "stm32f4xx_hal.h"
#include <algorithm>
#include <cstdint>
#include "Config.h"
#include "UiMenu.h"

extern HmiDisplay tft;

struct TouchEvent {
  enum Type : uint8_t { NONE = 0, PRESS, REPEAT, RELEASE } type{NONE};
  SoftKey key{SoftKey::NONE};
};

static bool touchWasDown = false;
static SoftKey touchActiveKey = SoftKey::NONE;
static uint32_t touchPressMs = 0;
static uint32_t touchLastRepeatMs = 0;

inline void resetTouchState() {
  touchWasDown = false;
  touchActiveKey = SoftKey::NONE;
  touchPressMs = 0;
  touchLastRepeatMs = 0;
}

inline void beginTouch() {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  uint16_t calData[5] = {300, 3600, 300, 3600, 1};
  tft.setTouch(calData);
  resetTouchState();
}

inline void correctTouchXY(uint16_t sx, uint16_t sy, uint16_t& tx, uint16_t& ty) {
  const int32_t mappedX = static_cast<int32_t>(sy) * (W - 1) / (H - 1);
  const int32_t mappedY = (H - 1) + static_cast<int32_t>(sx) * (0 - (H - 1)) / (W - 1);
  tx = static_cast<uint16_t>(std::clamp<int32_t>(mappedX, 0, W - 1));
  ty = static_cast<uint16_t>(std::clamp<int32_t>(mappedY, 0, H - 1));
}

inline bool hit(int px, int py, int x, int y, int w, int h) {
  return px >= x && px < (x + w) && py >= y && py < (y + h);
}

inline SoftKey cardKeyAt(int x, int y, int top, int height) {
  if (y < top || y >= top + height) return SoftKey::NONE;
  for (uint8_t i = 0; i < 3; ++i) {
    const int cardX = UI_CARD_X0 + i * (UI_CARD_W + UI_CARD_GAP);
    if (hit(x, y, cardX, top, UI_CARD_W, height)) {
      return i == 0 ? SoftKey::CARD_0 : (i == 1 ? SoftKey::CARD_1 : SoftKey::CARD_2);
    }
  }
  return SoftKey::NONE;
}

inline bool uiUsesEditFooter(const UiState& ui) {
  return menuEditKey(ui.menu) != UiEditKey::NONE || ui.menu == UiMenuId::NAV_MISSION_GO ||
         ui.menu == UiMenuId::NAV_MISSION_SAVE || ui.menu == UiMenuId::NAV_MISSION_STOP;
}

inline SoftKey touchKeyAt(const UiState& ui, int x, int y) {
  if (ui.menu != UiMenuId::OVERVIEW && hit(x, y, HOME_TOUCH_X, HOME_TOUCH_Y, HOME_TOUCH_W, HOME_TOUCH_H)) {
    return SoftKey::TOP_LEFT;
  }

  if (ui.menu == UiMenuId::OVERVIEW) {
    return cardKeyAt(x, y, OVERVIEW_CARD_Y, OVERVIEW_CARD_H);
  }

  if (menuHasChildren(ui.menu)) {
    const SoftKey card = cardKeyAt(x, y, SUBMENU_CARD_Y, SUBMENU_CARD_H);
    if (card != SoftKey::NONE) return card;
    if (hit(x, y, CAROUSEL_LEFT_X, CAROUSEL_NAV_Y, CAROUSEL_NAV_W, CAROUSEL_NAV_H)) return SoftKey::LEFT;
    if (hit(x, y, CAROUSEL_RIGHT_X, CAROUSEL_NAV_Y, CAROUSEL_NAV_W, CAROUSEL_NAV_H)) return SoftKey::RIGHT;
    return SoftKey::NONE;
  }

  if (ui.menu == UiMenuId::ESC_MANUAL_TEST) {
    if (hit(x, y, 108, 40, 104, 50)) return SoftKey::TEST_FORWARD;
    if (hit(x, y, 6, 96, 94, 70)) return SoftKey::TEST_LEFT;
    if (hit(x, y, 108, 96, 104, 70)) return SoftKey::TEST_STOP;
    if (hit(x, y, 220, 96, 94, 70)) return SoftKey::TEST_RIGHT;
    if (hit(x, y, 108, 172, 104, 52)) return SoftKey::TEST_REVERSE;
    return SoftKey::NONE;
  }

  if (uiUsesEditFooter(ui)) {
    if (hit(x, y, SOFTKEY_X0, SOFTKEY_Y, SOFTKEY_W, SOFTKEY_H)) return SoftKey::LEFT;
    if (hit(x, y, SOFTKEY_X0 + SOFTKEY_W + SOFTKEY_GAP, SOFTKEY_Y, SOFTKEY_W, SOFTKEY_H)) return SoftKey::OK;
    if (hit(x, y, SOFTKEY_X0 + 2 * (SOFTKEY_W + SOFTKEY_GAP), SOFTKEY_Y, SOFTKEY_W, SOFTKEY_H)) return SoftKey::RIGHT;
  }
  return SoftKey::NONE;
}

inline TouchEvent pollTouch(const UiState& ui) {
  TouchEvent ev{};
  uint16_t sx = 0, sy = 0;
  const bool down = tft.getTouch(&sx, &sy, TOUCH_THRESHOLD);
  const uint32_t now = HAL_GetTick();

  if (!down) {
    if (touchWasDown) {
      ev.type = TouchEvent::RELEASE;
      ev.key = touchActiveKey;
      resetTouchState();
    }
    return ev;
  }

  uint16_t tx = 0, ty = 0;
  correctTouchXY(sx, sy, tx, ty);
  const SoftKey key = touchKeyAt(ui, tx, ty);

  if (!touchWasDown) {
    touchWasDown = true;
    touchActiveKey = key;
    touchPressMs = now;
    touchLastRepeatMs = now;
    ev.type = TouchEvent::PRESS;
    ev.key = key;
    return ev;
  }

  if (key != touchActiveKey) return ev;
  const bool repeatable = key == SoftKey::LEFT || key == SoftKey::RIGHT;
  if (repeatable && now - touchPressMs >= 450U && now - touchLastRepeatMs >= 120U) {
    touchLastRepeatMs = now;
    ev.type = TouchEvent::REPEAT;
    ev.key = key;
  }
  return ev;
}
