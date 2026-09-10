#!/usr/bin/env python3
from pathlib import Path
root=Path(__file__).resolve().parents[1]
h=(root/'include/UsbCdcPort.h').read_text()
c=(root/'src/usb/UsbCdcPort.cpp').read_text()
cdc=(root/'src/usb/usbd_cdc_if.cpp').read_text()
main=(root/'src/main.cpp').read_text()
rules=(root/'99-blackpill-stm32.rules').read_text()
req=[('init hook decl','onUsbClassInit()',h),('deinit hook decl','onUsbClassDeInit()',h),('main service','void service()',h),('class init hook','gUsb.onUsbClassInit();',cdc),('class deinit hook','gUsb.onUsbClassDeInit();',cdc),('session queue drop','resetSessionState(true, true)',c),('ST TxState cross-check','hcdc->TxState == 0U',c),('bounded repair','kTxStallRepairMs',c),('explicit recovery','USB:RECOVER',main),('observable status','USB:STATUS',main),('main-context service','gUsb.service();',main),('exact runtime serial','ATTR{serial}=="338133833134"',rules),('exact boot serial','ATTRS{serial}=="338133833134"',rules)]
for name,tok,text in req:
    if tok not in text: raise SystemExit(f'FAIL {name}: missing {tok}')
if 'tx_started_ms_ = 0U;' not in c: raise SystemExit('FAIL tx timestamp is not cleared')
if 'TxState==1' not in c or 'host backpressure' not in c: raise SystemExit('FAIL backpressure guard missing')
print('USB_SESSION_RECOVERY_SELF_CHECK_PASS')
