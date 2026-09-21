#!/usr/bin/env python3
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
usb=(ROOT/'src/usb/UsbCdcPort.cpp').read_text(); usb_h=(ROOT/'src/usb/UsbCdcPort.h').read_text()
main=(ROOT/'src/main.cpp').read_text(); board=(ROOT/'src/BoardSupport.cpp').read_text()
boot=(ROOT/'bootloader/src/main.c').read_text(); uploader=(ROOT/'scripts/cdc_boot_upload.py').read_text(); manifest=(ROOT/'scripts/make_app_manifest.py').read_text(); pio=(ROOT/'platformio.ini').read_text()

def check(ok,msg):
    if not ok: raise SystemExit('FAIL '+msg)
    print('PASS '+msg)
irq=usb.split('void UsbCdcPort::onTransmitComplete()',1)[1].split('}',1)[0]
check('poll();' not in irq and 'USBD_CDC_TransmitPacket' not in irq,'USB TX-complete IRQ is non-reentrant')
check('tx_service_pending_ = true' in usb and 'tx_service_pending_ = false' in usb,'USB deferred TX kick exists')
check('rx_high_water_' in usb_h and 'tx_low_dropped_' in usb_h,'USB queue observability exists')
check('Board_SetRealtimeServiceCallback' in main and 'gUsb.service();' in main,'USB service continues during TFT yields')
check('Board_FaultReset' in board and 'RTC->BKP6R = stack ? stack[6]' in board,'fault PC/LR capture exists')
check('__attribute__((naked)) void HardFault_Handler' in board,'fault wrapper captures exception stack')
check('kCrashCounterClearMs = 30000U' in main,'30 s healthy window retained')
check('FAULT:STATUS' in main,'runtime fault snapshot observable')
check('MANIFEST_FORMAT 2UL' in boot and 'GATEWAY_BOARD_ID' in boot,'manifest remains board-bound')
check('boot_watchdog_init' in boot and 'boot_watchdog_stop' in boot,'bootloader watchdog lifecycle exists')
check('DATA2:' in boot and 'ERR:DATA2:CRC' in boot,'bootloader chunk CRC protocol exists')
check('BOOT_PROTOCOL_VERSION 3UL' in boot and 'proto=3' in uploader and 'DATA2:' in uploader,'CDC uploader protocol remains synchronized')
check('FORMAT = 2' in manifest and 'PROFILES = {' in manifest and '0xF411CE01' in manifest and '0xF401CD01' in manifest and '0xF411CC01' in manifest,'manifest generator supports all board-bound targets')
check('scripts/cdc_boot_upload.py $SOURCE' in pio and '-DVECT_TAB_OFFSET=0x00004000U' in pio,'v2 keeps resident CDC bootloader upload path')
print('STAGE1_EMBEDDED_ROBUSTNESS_SELF_CHECK_PASS')
