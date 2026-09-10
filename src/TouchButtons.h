// ============================================================================
// TouchButtons.h — Stage-1 HOME/MAIN/domain/detail input with dead-man safety.
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
  enum Type : uint8_t { NONE = 0, PRESS, REPEAT, HOLD, RELEASE } type{NONE};
  SoftKey key{SoftKey::NONE};
};

static bool touchWasDown = false;
static SoftKey touchActiveKey = SoftKey::NONE;
static uint32_t touchPressMs = 0U;
static uint32_t touchLastRepeatMs = 0U;
static bool touchCanceled = false;
static bool touchHoldSent = false;
static bool touchDidRepeat = false;
static uint32_t touchGeneration = 1U;
static uint32_t touchPressGeneration = 0U;
static bool touchBlockUntilRelease = false;

inline void resetTouchState() {
  touchWasDown = false;
  touchActiveKey = SoftKey::NONE;
  touchPressMs = 0U;
  touchLastRepeatMs = 0U;
  touchCanceled = false;
  touchHoldSent = false;
  touchDidRepeat = false;
  touchPressGeneration = 0U;
}

inline void invalidateTouchGeneration() {
  ++touchGeneration;
  if (touchGeneration == 0U) touchGeneration = 1U;
  resetTouchState();
}

inline void suppressTouchUntilRelease() {
  touchBlockUntilRelease = true;
  resetTouchState();
}

inline void beginTouch() {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  uint16_t calData[5] = {300, 3600, 300, 3600, 1};
  tft.setTouch(calData);
  invalidateTouchGeneration();
}

inline void correctTouchXY(uint16_t sx, uint16_t sy, uint16_t &tx, uint16_t &ty) {
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
  for (uint8_t i = 0U; i < DOMAIN_PAGE_SIZE; ++i) {
    const int cardX = UI_CARD_X0 + i * (UI_CARD_W + UI_CARD_GAP);
    if (hit(x, y, cardX, top, UI_CARD_W, height))
      return i == 0U ? SoftKey::CARD_0 : (i == 1U ? SoftKey::CARD_1 : SoftKey::CARD_2);
  }
  return SoftKey::NONE;
}

inline bool uiUsesEditFooter(const UiState &ui) {
  return menuEditKey(ui.menu) != UiEditKey::NONE ||
         ui.menu == UiMenuId::SYSTEM_TFT_TEST ||
         ui.menu == UiMenuId::NAV_MISSION_GO ||
         ui.menu == UiMenuId::NAV_MISSION_SAVE ||
         ui.menu == UiMenuId::NAV_MISSION_STOP;
}

inline bool uiUsesPagerFooter(const UiState &ui) {
  if (ui.menu == UiMenuId::MAIN_MENU || ui.menu == UiMenuId::HOME ||
      ui.menu == UiMenuId::ESC_MANUAL_TEST || uiUsesEditFooter(ui))
    return false;
  if (menuHasChildren(ui.menu)) {
    uint8_t count = 0U;
    (void)menuChildren(ui.menu, count);
    return menuPageCount(count) > 1U;
  }
  return menuViewCount(ui.menu) > 1U;
}

inline SoftKey touchKeyAt(const UiState &ui, int x, int y) {
  if (ui.menu == UiMenuId::HOME || ui.menu == UiMenuId::OVERVIEW) {
    return hit(x, y, HOME_MENU_X, HOME_MENU_Y, HOME_MENU_W, HOME_MENU_H)
               ? SoftKey::MENU
               : SoftKey::NONE;
  }

  if (ui.menu != UiMenuId::SPLASH &&
      hit(x, y, HOME_TOUCH_X, HOME_TOUCH_Y, HOME_TOUCH_W, HOME_TOUCH_H))
    return SoftKey::TOP_LEFT;

  if (ui.menu == UiMenuId::MAIN_MENU)
    return cardKeyAt(x, y, SUBMENU_CARD_Y, SUBMENU_CARD_H);

  if (ui.menu == UiMenuId::ESC_MANUAL_TEST) {
    if (hit(x, y, 108, 40, 104, 50)) return SoftKey::TEST_FORWARD;
    if (hit(x, y, 6, 96, 94, 70)) return SoftKey::TEST_LEFT;
    if (hit(x, y, 108, 96, 104, 70)) return SoftKey::TEST_STOP;
    if (hit(x, y, 220, 96, 94, 70)) return SoftKey::TEST_RIGHT;
    if (hit(x, y, 108, 172, 104, 52)) return SoftKey::TEST_REVERSE;
    return SoftKey::NONE;
  }

  if (menuHasChildren(ui.menu)) {
    const SoftKey card = cardKeyAt(x, y, SUBMENU_CARD_Y, SUBMENU_CARD_H);
    if (card != SoftKey::NONE) {
      const uint8_t slot = card == SoftKey::CARD_0 ? 0U : (card == SoftKey::CARD_1 ? 1U : 2U);
      return menuCardAt(ui.menu, ui.pageIndex, slot) == UiMenuId::SPLASH
                 ? SoftKey::NONE
                 : card;
    }
  }

  if (uiUsesEditFooter(ui)) {
    if (hit(x, y, SOFTKEY_X0, SOFTKEY_Y, SOFTKEY_W, SOFTKEY_H)) return SoftKey::LEFT;
    if (hit(x, y, SOFTKEY_X0 + SOFTKEY_W + SOFTKEY_GAP, SOFTKEY_Y, SOFTKEY_W, SOFTKEY_H)) return SoftKey::OK;
    if (hit(x, y, SOFTKEY_X0 + 2 * (SOFTKEY_W + SOFTKEY_GAP), SOFTKEY_Y, SOFTKEY_W, SOFTKEY_H)) return SoftKey::RIGHT;
    return SoftKey::NONE;
  }

  if (uiUsesPagerFooter(ui)) {
    if (hit(x, y, CAROUSEL_LEFT_X, CAROUSEL_NAV_Y, CAROUSEL_NAV_W, CAROUSEL_NAV_H)) return SoftKey::LEFT;
    if ((ui.menu == UiMenuId::NAV_MISSION ||
         menuDetailActionTarget(ui.menu, ui.detailViewIndex) != UiMenuId::SPLASH) &&
        hit(x, y, CAROUSEL_PAGE_X, CAROUSEL_NAV_Y, CAROUSEL_PAGE_W, CAROUSEL_NAV_H))
      return SoftKey::OK;
    if (hit(x, y, CAROUSEL_RIGHT_X, CAROUSEL_NAV_Y, CAROUSEL_NAV_W, CAROUSEL_NAV_H)) return SoftKey::RIGHT;
  }
  return SoftKey::NONE;
}

inline bool isManualMotionKey(SoftKey key) {
  return key == SoftKey::TEST_FORWARD || key == SoftKey::TEST_REVERSE ||
         key == SoftKey::TEST_LEFT || key == SoftKey::TEST_RIGHT;
}

inline TouchEvent pollTouch(const UiState &ui) {
  TouchEvent ev{};
  uint16_t sx = 0U, sy = 0U;
  const bool down = tft.getTouch(&sx, &sy, TOUCH_THRESHOLD);
  const uint32_t now = HAL_GetTick();

  if (touchBlockUntilRelease) {
    if (!down) {
      touchBlockUntilRelease = false;
      resetTouchState();
    }
    return ev;
  }

  if (!down) {
    if (touchWasDown) {
      const uint32_t heldMs = static_cast<uint32_t>(now - touchPressMs);
      ev.type = TouchEvent::RELEASE;
      if (touchPressGeneration != touchGeneration) {
        resetTouchState();
        return TouchEvent{};
      }
      if (isManualMotionKey(touchActiveKey) && touchHoldSent) {
        ev.key = touchActiveKey;
      } else if (touchActiveKey == SoftKey::MENU && touchHoldSent) {
        // Long-hold Service entry is one-shot; never also execute normal MENU.
        ev.key = SoftKey::NONE;
      } else if (!touchCanceled && !touchDidRepeat && heldMs >= TOUCH_TAP_MIN_MS) {
        ev.key = touchActiveKey;
      }
      resetTouchState();
    }
    return ev;
  }

  if (touchWasDown && touchPressGeneration != touchGeneration) {
    resetTouchState();
    return ev;
  }

  uint16_t tx = 0U, ty = 0U;
  correctTouchXY(sx, sy, tx, ty);
  const SoftKey key = touchKeyAt(ui, tx, ty);

  if (!touchWasDown) {
    touchWasDown = true;
    touchActiveKey = key;
    touchPressGeneration = touchGeneration;
    touchPressMs = now;
    touchLastRepeatMs = now;
    touchCanceled = key == SoftKey::NONE;
    ev.type = TouchEvent::PRESS;
    ev.key = key;
    return ev;
  }

  if (key != touchActiveKey) {
    if (!touchCanceled && touchHoldSent && isManualMotionKey(touchActiveKey)) {
      ev.type = TouchEvent::RELEASE;
      ev.key = touchActiveKey;
    }
    touchCanceled = true;
    return ev;
  }
  if (touchCanceled) return ev;

  if (key == SoftKey::MENU && !touchHoldSent &&
      static_cast<uint32_t>(now - touchPressMs) >= SERVICE_HOLD_MS) {
    touchHoldSent = true;
    ev.type = TouchEvent::HOLD;
    ev.key = SoftKey::MENU;
    return ev;
  }

  if (isManualMotionKey(key) && !touchHoldSent &&
      static_cast<uint32_t>(now - touchPressMs) >= TOUCH_HOLD_ACTION_MS) {
    touchHoldSent = true;
    ev.type = TouchEvent::HOLD;
    ev.key = key;
    return ev;
  }

  const bool repeatable = key == SoftKey::LEFT || key == SoftKey::RIGHT;
  if (repeatable && static_cast<uint32_t>(now - touchPressMs) >= TOUCH_REPEAT_DELAY_MS &&
      static_cast<uint32_t>(now - touchLastRepeatMs) >= TOUCH_REPEAT_MS) {
    touchLastRepeatMs = now;
    touchDidRepeat = true;
    ev.type = TouchEvent::REPEAT;
    ev.key = key;
  }
  return ev;
}
