#!/usr/bin/env python3
from pathlib import Path
import re
ROOT=Path(__file__).resolve().parents[1]
def rd(p): return (ROOT/p).read_text(errors='replace')
def need(ok,msg):
    if not ok: raise SystemExit('FAIL '+msg)
    print('PASS '+msg)

pio=rd('platformio.ini'); board=rd('src/BoardSupport.cpp')
usbc=rd('src/usb/usbd_conf.c'); linker=rd('linker/STM32F103C8TX_APP.ld')
tp=rd('src/TelemetryProtocolF103.cpp'); hmi=rd('src/HmiDisplay.cpp')
need('[env:bluepill_f103c8]' in pio,'STM32F103C8T6 compatibility target remains available')
need('board = bluepill_f103c8' in pio and '-DSTM32F103xB' in pio,'PlatformIO target is F103C8')
need('board_upload.maximum_size = 63488' in pio,'application is capped at 62 KiB')
need(re.search(r'FLASH\s+\(rx\)\s*:\s*ORIGIN\s*=\s*0x8000000,\s*LENGTH\s*=\s*62K', linker) is not None,
     'linker reserves final 2 KiB flash region')
need(re.search(r'RAM\s+\(xrw\)\s*:\s*ORIGIN\s*=\s*0x20000000,\s*LENGTH\s*=\s*20K', linker) is not None,
     'linker enforces official 20 KiB SRAM')
need('-DPERSIST_STORAGE_BASE=0x0800F800UL' in pio and '-DPERSIST_STORAGE_LIMIT=0x08010000UL' in pio,'persistent config owns final 2 KiB flash region')
need('RCC_PLL_MUL9' in board and 'RCC_USBCLKSOURCE_PLL_DIV1_5' in board,'72 MHz core and 48 MHz USB clock are explicit')
need('htim3.Instance = TIM3' in board and 'TIM3_IRQn' in board,'TIM3 backs the application watchdog')
need('GPIO_PIN_6 | GPIO_PIN_7' in board and 'GPIO_PULLUP' in board,'limit switches remain active-low pullups')
need('GPIO_PIN_2' in board and 'GPIO_PIN_8' in board and 'TIM2' in board and 'TIM4' in board,'BTS7960 PWM pins/timers are retained')
need('constexpr uint16_t kTftResetPin = GPIO_PIN_10;' in board and
     'constexpr uint16_t kTftRst = GPIO_PIN_10;' in hmi,
     'F103 TFT reset is mapped to PB10')
need('hpcd_USB_FS.Instance = USB' in usbc and 'USB_LP_CAN1_RX0_IRQn' in usbc,'F103 USB FS device IRQ is configured')
need('std::strcmp(parts[2],"CAM")==0' in tp and 'std::strcmp(parts[2],"IMU")==0' in tp and 'std::strcmp(parts[2],"NAV2")==0' in tp,'F103 parser matches all F4X3 groups emitted by host')
need('#if !defined(BOARD_F103C8)\n#include "fonts/Font16.inc"' in hmi,'large font table is excluded only on F103')
need('[env:blackpill_f411ce_romdfu]' in pio,'F411 regression/recovery environment remains available')
print('V3_F103C8_SELF_CHECK_PASS')
