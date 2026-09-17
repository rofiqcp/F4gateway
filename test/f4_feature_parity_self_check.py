#!/usr/bin/env python3
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
main=(ROOT/'src/main.cpp').read_text(); winch=(ROOT/'src/Bts7960Winch.cpp').read_text(); board=(ROOT/'src/BoardSupport.cpp').read_text(); ui=(ROOT/'src/HmiOperatorUi.h').read_text(); touch=(ROOT/'src/HmiTouchService.cpp').read_text(); hmi=(ROOT/'src/HmiDisplay.h').read_text(); pio=(ROOT/'platformio.ini').read_text()
def need(v,m):
    if not v: raise SystemExit('FAIL: '+m)
    print('PASS:',m)
for token in ['GPIO_PIN_6 | GPIO_PIN_7','GPIO_PULLUP','GPIO_PIN_2','GPIO_PIN_8','TIM2','TIM4']:
    need(token in board,'BTS/LS hardware '+token)
for token in ['kLimitConfirmMs = 30U','kReverseDeadtimeMs = 50U','kHomeWatchdogMs = 30000U','gTopConfirmed && gBottomConfirmed','TOP_LIMIT','BOTTOM_LIMIT']:
    need(token in winch,'winch safety '+token)
for cmd in ['UP HOME','UP 1','UP 2','DOWN HOME','DOWN 1','DOWN 2','WINCH 0','WINCH 1','WINCH 2','WINCH STOP']:
    need(cmd in winch,'legacy winch command '+cmd)
for cmd in ['LIMITS','LS STATUS','CONFIG RESET','WINCH PWM ','RAWTOUCH','CALIBRATE','CALSTOP','HMI RESET','TFT RESET','HMI REDRAW']:
    need(cmd in main or cmd in touch,'f4 service command '+cmd)
need('LS TOP ACTIVE' in ui and 'LS BOTTOM ACTIVE' in ui,'HMI exposes both LS sensors')
need('touch_x0_{580U}' in hmi and 'touch_x1_{3440U}' in hmi and 'touch_y0_{330U}' in hmi and 'touch_y1_{3310U}' in hmi and 'touch_rotate_{false}' in hmi and 'touch_invert_y_{true}' in hmi,'touch mapping matches proven F4 v1 physical-panel calibration')
need('-DVECT_TAB_OFFSET=0x00004000U' in pio and 'cdc_boot_upload.py' in pio,'USB CDC bootloader contract preserved')
need(not (ROOT/'include').exists() and not (ROOT/'lib').exists(),'headers/libraries consolidated under src')
print('F4_FEATURE_PARITY_SELF_CHECK_PASS')
