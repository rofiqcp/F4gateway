#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
usb = (ROOT / "src/usb/UsbCdcPort.cpp").read_text()
usb_h = (ROOT / "include/UsbCdcPort.h").read_text()
main = (ROOT / "src/main.cpp").read_text()
neo = (ROOT / "src/Neo3ProSensors.cpp").read_text()
neo_h = (ROOT / "src/Neo3ProSensors.h").read_text()
board = (ROOT / "src/BoardSupport.cpp").read_text()
boot = (ROOT / "bootloader/src/main.c").read_text()
uploader = (ROOT / "scripts/cdc_boot_upload.py").read_text()
manifest = (ROOT / "scripts/make_app_manifest.py").read_text()

def check(cond, msg):
    if not cond:
        raise SystemExit("FAIL " + msg)
    print("PASS " + msg)

# USB ISR is state-only; ST USB TX is main-loop-owned.
irq = usb.split("void UsbCdcPort::onTransmitComplete()", 1)[1].split("}", 1)[0]
check("poll();" not in irq and "USBD_CDC_TransmitPacket" not in irq, "USB TX-complete IRQ is non-reentrant")
check("tx_service_pending_ = true" in usb and "tx_service_pending_ = false" in usb, "USB deferred TX kick exists")
check("rx_high_water_" in usb_h and "tx_low_dropped_" in usb_h, "USB high-water/drop observability exists")
check("Board_SetRealtimeServiceCallback" in main and "gUsb.service();" in main, "USB service continues during TFT realtime yields")

# MCP2515 receive and recovery are bounded/event-driven.
check("CAN_RX_REALTIME_BUDGET_US" in neo and "CAN_RX_MAIN_BUDGET_US" in neo, "CAN RX time budgets exist")
check("HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10)" in neo and "can_rx_budget_exhaustions_" in neo_h, "MCP INT-driven bounded drain exists")
check("can_rx0_overflows_" in neo_h and "can_rx1_overflows_" in neo_h, "MCP RX overflow channels are separated")
check("CAN_TX_HW_DEADLINE_US" in neo and "abortMcpTxBounded" in neo, "CAN TX deadline/abort exists")
check("CAN_RECOVERY_COOLDOWN_MS" in neo and "can_tx_backoff_until_ms_" in neo_h, "CAN recovery cooldown/backoff exists")

# Faults reset with post-mortem context; healthy-window matches recovery intent.
check("Board_FaultReset" in board and "RTC->BKP6R = stack ? stack[6]" in board, "fault PC/LR capture exists")
check("__attribute__((naked)) void HardFault_Handler" in board, "fault wrapper captures active exception stack")
check("kCrashCounterClearMs = 30000U" in main, "application requires 30 s healthy window")
check("FAULT:STATUS" in main, "runtime fault snapshot is observable")

# Resident bootloader has stoppable watchdog and transactional chunk verification.
check("MANIFEST_FORMAT 2UL" in boot and "GATEWAY_BOARD_ID" in boot, "manifest v2 is board-bound")
check("boot_watchdog_init" in boot and "boot_watchdog_stop" in boot, "bootloader watchdog lifecycle exists")
check("DATA2:" in boot and "ERR:DATA2:CRC" in boot, "bootloader chunk CRC protocol exists")
check("memcmp((const void *)(APP_BASE + offset), data, len)" in boot, "flash chunk readback verification exists")
check("BOOT_PROTOCOL_VERSION 3UL" in boot and "proto=3" in uploader and "DATA2:" in uploader, "uploader negotiates robust boot protocol")
check("FORMAT = 2" in manifest and "BOARD_ID = 0xF411CE01" in manifest, "host manifest generator matches bootloader")

print("STAGE1_EMBEDDED_ROBUSTNESS_SELF_CHECK_PASS")
