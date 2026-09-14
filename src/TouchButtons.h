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
  // Empirical 7-point calibration from the installed panel:
  // raw X rises left->right (~0.9k, 2.3k, 3.7k), raw Y falls top->bottom
  // (~3.2k, 2.1k, 0.7k). Keep axes unrotated and invert screen Y.
  uint16_t calData[5] = {580, 3440, 330, 3310, 4};
  tft.setTouch(calData);
  invalidateTouchGeneration();
}

inline void correctTouchXY(uint16_t sx, uint16_t sy, uint16_t &tx, uint16_t &ty) {
  // HmiDisplay::getTouch() follows TFT_eSPI semantics: setTouch() calibration
  // (including rotate/invert flags) is already applied and sx/sy are screen
  // coordinates.  A second landscape rotation here used to move valid touches
  // away from their visible buttons (notably HOME -> MENU).
  tx = static_cast<uint16_t>(std::clamp<int32_t>(sx, 0, W - 1));
  ty = static_cast<uint16_t>(std::clamp<int32_t>(sy, 0, H - 1));
}

inline bool hit(int px, int py, int x, int y, int w, int h) {
  return px >= x && px < (x + w) && py >= y && py < (y + h);
}

inline int touchColumnAt(int x, int y, int top, int height) {
  if (y < top || y >= top + height) return -1;
  if (hit(x, y, TOUCH_COL0_X, top, TOUCH_COL_W, height)) return 0;
  if (hit(x, y, TOUCH_COL1_X, top, TOUCH_COL_W, height)) return 1;
  if (hit(x, y, TOUCH_COL2_X, top, TOUCH_COL_W, height)) return 2;
  return -1;
}

inline SoftKey cardKeyAt(int x, int y) {
  const int col = touchColumnAt(x, y, TOUCH_MIDDLE_Y, TOUCH_MIDDLE_H);
  if (col == 0) return SoftKey::CARD_0;
  if (col == 1) return SoftKey::CARD_1;
  if (col == 2) return SoftKey::CARD_2;
  return SoftKey::NONE;
}

inline SoftKey footerKeyAt(int x, int y) {
  const int col = touchColumnAt(x, y, TOUCH_BOTTOM_Y, TOUCH_BOTTOM_H);
  if (col == 0) return SoftKey::LEFT;
  if (col == 1) return SoftKey::OK;
  if (col == 2) return SoftKey::RIGHT;
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
  if (ui.menu == UiMenuId::SPLASH)
    return SoftKey::NONE;

  // Zone 1: small HOME/MENU control at the physical top-left.  On HOME the
  // icon opens MAIN MENU; elsewhere the same fixed zone performs back/home.
  if (hit(x, y, HOME_TOUCH_X, HOME_TOUCH_Y, HOME_TOUCH_W, HOME_TOUCH_H))
    return (ui.menu == UiMenuId::HOME || ui.menu == UiMenuId::OVERVIEW)
               ? SoftKey::MENU
               : SoftKey::TOP_LEFT;

  // Zones 2..4: only the three middle columns are selectable menu cards.
  if (ui.menu == UiMenuId::HOME || ui.menu == UiMenuId::OVERVIEW ||
      ui.menu == UiMenuId::MAIN_MENU)
    return cardKeyAt(x, y);

  if (ui.menu == UiMenuId::ESC_MANUAL_TEST) {
    // Manual-test actions are deliberately constrained to the same six fixed
    // middle/footer zones; there are no extra hidden touch rectangles.
    const int middle = touchColumnAt(x, y, TOUCH_MIDDLE_Y, TOUCH_MIDDLE_H);
    if (middle == 0) return SoftKey::TEST_LEFT;
    if (middle == 1) return SoftKey::TEST_STOP;
    if (middle == 2) return SoftKey::TEST_RIGHT;
    const int bottom = touchColumnAt(x, y, TOUCH_BOTTOM_Y, TOUCH_BOTTOM_H);
    if (bottom == 0) return SoftKey::TEST_REVERSE;
    if (bottom == 1) return SoftKey::TEST_STOP;
    if (bottom == 2) return SoftKey::TEST_FORWARD;
    return SoftKey::NONE;
  }

  if (menuHasChildren(ui.menu)) {
    const SoftKey card = cardKeyAt(x, y);
    if (card != SoftKey::NONE) {
      const uint8_t slot = card == SoftKey::CARD_0 ? 0U : (card == SoftKey::CARD_1 ? 1U : 2U);
      return menuCardAt(ui.menu, ui.pageIndex, slot) == UiMenuId::SPLASH
                 ? SoftKey::NONE
                 : card;
    }
  }

  // Zones 5..7: LEFT / OK / RIGHT.  A page may ignore OK when it has no
  // center action, but no touch outside these fixed zones is accepted.
  if (uiUsesEditFooter(ui))
    return footerKeyAt(x, y);

  if (uiUsesPagerFooter(ui)) {
    const SoftKey footer = footerKeyAt(x, y);
    if (footer == SoftKey::LEFT || footer == SoftKey::RIGHT)
      return footer;
    if (footer == SoftKey::OK &&
        (ui.menu == UiMenuId::NAV_MISSION ||
         menuDetailActionTarget(ui.menu, ui.detailViewIndex) != UiMenuId::SPLASH))
      return SoftKey::OK;
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
