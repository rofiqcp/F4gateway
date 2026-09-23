#!/usr/bin/env python3
from pathlib import Path

root = Path(__file__).resolve().parents[1]
h = (root/'src/usb/UsbCdcPort.h').read_text()
c = (root/'src/usb/UsbCdcPort.cpp').read_text()
cdc = (root/'src/usb/usbd_cdc_if.cpp').read_text()
handler = (root/'src/F103MinimalCommandHandler.inc').read_text()
board = (root/'src/BoardSupport.cpp').read_text()
boot = (root/'src/bootloader/main.c').read_text()
rules = (root/'99-f103c8-gateway.rules').read_text()

req = [
    ('init hook declaration', 'onUsbClassInit()', h),
    ('deinit hook declaration', 'onUsbClassDeInit()', h),
    ('main-context service', 'void service()', h),
    ('class init hook', 'gUsb.onUsbClassInit();', cdc),
    ('class deinit hook', 'gUsb.onUsbClassDeInit();', cdc),
    ('session queue reset', 'resetSessionState(true, true)', c),
    ('HAL TxState cross-check', 'hcdc->TxState == 0U', c),
    ('bounded TX repair', 'kTxStallRepairMs', c),
    ('explicit recovery command', 'USB:RECOVER', handler),
    ('observable status command', 'USB:STATUS', handler),
    ('host session handshake', 'HOST:HELLO:', handler),
    ('runtime product identity', 'BLUEPILL_F103 CDC in FS Mode', rules),
    ('boot product identity', 'BLUEPILL_F103 BOOT CDC', rules),
    ('stable runtime alias', 'SYMLINK+="f4gateway"', rules),
    ('stable boot alias', 'SYMLINK+="f4gateway-boot"', rules),
]
for name, token, text in req:
    if token not in text:
        raise SystemExit(f'FAIL {name}: missing {token}')

for name, text in [('application', board), ('bootloader', boot)]:
    for token in ('RCC_OSCILLATORTYPE_HSE', 'RCC_HSE_ON',
                  'RCC_PLLSOURCE_HSE', 'RCC_PLL_MUL9',
                  'RCC_USBCLKSOURCE_PLL_DIV1_5'):
        if token not in text:
            raise SystemExit(f'FAIL {name} USB clock: missing {token}')
    if 'RCC_PLLSOURCE_HSI' in text:
        raise SystemExit(f'FAIL {name} USB clock regressed to HSI')

if 'APP_BASE 0x08002000UL' not in boot or 'PERSIST_BASE 0x0800F800UL' not in boot:
    raise SystemExit('FAIL C8 boot/application/persistent layout')
if 'tx_started_ms_ = 0U;' not in c:
    raise SystemExit('FAIL TX timestamp is not cleared')
if 'ATTR{serial}' in rules or 'ATTRS{serial}' in rules:
    raise SystemExit('FAIL clone support regressed to hardcoded MCU serial')

print('F103C8_USB_SESSION_RECOVERY_SELF_CHECK_PASS')
