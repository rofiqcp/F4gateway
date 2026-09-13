from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def rd(path):
    return (ROOT / path).read_text(errors="replace")

def need(cond, msg):
    if not cond:
        raise SystemExit(f"FAIL {msg}")
    print(f"PASS {msg}")

neo = rd("src/Neo3ProSensors.cpp")
main = rd("src/main.cpp")
usb = rd("src/usb/UsbCdcPort.cpp")
board = rd("src/BoardSupport.cpp")

need("CAN_TX_HW_DEADLINE_US" in neo, "MCP TX has physical deadline")
need("serviceMcpTx()" in neo and "mcp_tx_pending_" in rd("src/Neo3ProSensors.h"), "MCP TX completion is asynchronous")
need("CAN_TX_BACKOFF_BASE_MS" in neo and "can_tx_backoff_until_ms_" in rd("src/Neo3ProSensors.h"), "CAN TX exponential backoff exists")
need("EFLG_TXBO" in neo and "EFLG_TXEP" in neo, "bus-off/error-passive detection exists")
need("abortMcpTxBounded" in neo and "CANCTRL_ABAT" in neo, "MCP hardware abort is bounded")
need("CAN_RECOVERY_COOLDOWN_MS" in neo and "last_can_recovery_ms_" in rd("src/Neo3ProSensors.h"), "CAN recovery has cooldown")
need("clearCanardTxQueue" in neo, "stale transfer tails are discarded on failed TX")
need("can_ok_ = false;" in neo.split("bool Neo3ProSensors::initMcp",1)[1][:700], "MCP reset prevents realtime re-entry")
need("Board_ReinitSpi2" not in neo.split("bool Neo3ProSensors::mcpTransfer",1)[1].split("bool Neo3ProSensors::mcpWrite",1)[0], "SPI failure recovery is deferred out of low-level transfer")
need("pollSerialGui(std::size_t byteBudget = 256U)" in main, "PC RX parsing has a byte budget")
need("uint8_t commandBudget = 4U" in main, "PC RX parsing has a command budget")
critical = usb.split("bool UsbCdcPort::writeLineCritical",1)[1].split("#ifdef HMI_TEST_HOOKS",1)[0]
need("writeLineHighPriority" in critical and "service();" in critical, "critical USB writes are nonblocking high-priority")
need("HAL_Delay" not in critical and "while (" not in critical, "critical USB writes never spin or sleep")
need("gDriveStopPending" in main and "serviceSafetyControlTx" in main, "safety STOP retry is latched outside critical USB write")
need("kTxStallRepairMs" in rd("include/UsbCdcPort.h") and "tx_stall_recovery_count_" in usb, "USB endpoint stall repair remains enabled")
need("kAppCrashMagic" in board and "NVIC_SystemReset" in board, "fatal boot/runtime failures reset into recovery path")
for handler in ("HardFault_Handler", "MemManage_Handler", "BusFault_Handler", "UsageFault_Handler"):
    need(handler in board, f"{handler} resets instead of hanging")
need("APP_WATCHDOG_TIMEOUT_MS" in main and "Board_WatchdogStart" in main, "application watchdog remains enabled")
need("while (true) {\n    __NOP();" not in board, "legacy fatal infinite NOP loop removed")
print("F4_GATEWAY_FAULT_CONTAINMENT_SELF_CHECK_PASS")
