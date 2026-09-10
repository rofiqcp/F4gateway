#!/usr/bin/env python3
from pathlib import Path
root=Path(__file__).parents[1]
s='\n'.join((root/'include/UsbCdcPort.h').read_text(),) if False else ''
s=(root/'include/UsbCdcPort.h').read_text()+(root/'src/usb/UsbCdcPort.cpp').read_text()+(root/'src/main.cpp').read_text()+ (root/'platformio.ini').read_text()
for tok in ['HMI_TEST_HOOKS','testSuppressTx','TEST:USB:TX_SILENT','8000U','blackpill_f411ce_faulttest']:
 assert tok in s,tok
print('USB_FAULT_INJECTION_SELF_CHECK_PASS')
