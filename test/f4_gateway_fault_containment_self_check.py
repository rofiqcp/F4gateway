#!/usr/bin/env python3
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def rd(path): return (ROOT/path).read_text(errors='replace')
def need(ok,msg):
    if not ok: raise SystemExit('FAIL '+msg)
    print('PASS '+msg)
main=rd('src/main.cpp'); usb=rd('src/usb/UsbCdcPort.cpp'); board=rd('src/BoardSupport.cpp'); pio=rd('platformio.ini')
need('pollSerialGui(std::size_t byteBudget = 256U)' in main,'PC RX parsing has byte budget')
need('uint8_t commandBudget = 4U' in main,'PC RX parsing has command budget')
critical=usb.split('bool UsbCdcPort::writeLineCritical',1)[1].split('#ifdef HMI_TEST_HOOKS',1)[0]
need('writeLineHighPriority' in critical and 'service();' in critical,'critical USB writes are nonblocking high-priority')
need('HAL_Delay' not in critical and 'while (' not in critical,'critical USB writes never spin or sleep')
need('gDriveStopPending' in main and 'serviceSafetyControlTx' in main,'safety STOP retry remains latched')
need('kTxStallRepairMs' in rd('src/usb/UsbCdcPort.h') and 'tx_stall_recovery_count_' in usb,'USB endpoint stall repair remains enabled')
need('kAppCrashMagic' in board and 'NVIC_SystemReset' in board,'fatal failures reset into recovery path')
for handler in ('HardFault_Handler','MemManage_Handler','BusFault_Handler','UsageFault_Handler'):
    need(handler in board,handler+' resets through common fault path')
need('APP_WATCHDOG_TIMEOUT_TICKS' in main and 'gWatchdogProgressEpoch' in main and 'Board_WatchdogStart' in main,'application watchdog remains enabled')
need('default_envs = bluepill_f103c8' in pio and '[env:bluepill_f103c8]' in pio and '[env:blackpill_f411ce_v2]' in pio,'v3 defaults to F103C8 while F411 recovery target remains available')
need('MCP2515' not in board and 'NEO3PRO' not in main,'removed CAN path cannot re-enter runtime')
print('F4_GATEWAY_FAULT_CONTAINMENT_SELF_CHECK_PASS')
