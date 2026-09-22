#!/usr/bin/env python3
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
def rd(path): return (ROOT / path).read_text(errors="replace")
def need(ok, msg):
    if not ok: raise SystemExit("FAIL: " + msg)
    print("PASS:", msg)

boot = rd("bootloader/src/main_f103.c")
boot_usb = rd("bootloader/src/usb/boot_usb.c")
app = rd("src/main.cpp")
pio = rd("platformio.ini")
uploader = rd("scripts/cdc_boot_upload.py")
manifest = rd("scripts/make_app_manifest.py")
provision = rd("scripts/provision_recovery_stlink_f103.sh")
boot_syscalls = rd("bootloader/src/syscalls.c")
app_syscalls = rd("src/PlatformSyscalls.S")

main_pos = boot.index("int main(void)")
early_pos = boot.index("early_usb_detach_hold();", main_pos)
hal_pos = boot.index("HAL_Init();", main_pos)
need(early_pos < hal_pos, "bootloader asserts D+ low before HAL init")
need("early_usb_detach_release();" in boot and
     "Reset-equivalent floating input: release D+" in boot,
     "normal boot releases PA12 before direct app handoff")
need("reset_to_app_after_usb" in boot and "NVIC_SystemReset();" in boot,
     "bootloader leaves active USB through hardware reset")
need("BOOT:STATE:active=" in boot and "BOOT_UPDATE_TIMEOUT_MS 30000UL" in boot,
     "resident protocol exposes resumable state and update timeout")
need("BOOT_USB_LOSS_RECOVERY_MS" in boot and "boot_usb_disconnect_hold();" in boot,
     "resident USB loss has local recovery")
need("BOOT_USB_RECOVERY_COOLDOWN_MS" in boot and
     "BOOT_USB_MAX_RECOVERIES" in boot and
     "usb_recovery_count < BOOT_USB_MAX_RECOVERIES" in boot,
     "resident physical USB recovery is cooldown-limited and bounded")
need(boot.count("section(\".RamFunc\")") >= 3,
     "flash erase/write/wait critical routines execute from SRAM")
need("reset_usb_peripheral_while_detached" in boot_usb and
     "__HAL_RCC_USB_FORCE_RESET" in boot_usb,
     "resident USB startup hard-resets F1 USB peripheral while detached")
need("Board_BackupWrite(0U, kF103BootReqLo)" in app and
     "Board_BackupWrite(9U, kF103BootReqHi)" in app,
     "runtime records resident-boot request before reset")
need("direct image jump" in app and "NVIC_SystemReset();" in app,
     "runtime-to-boot uses reset-clean handoff instead of direct vector jump")
need("reinterpret_cast<void (*)(void)>(bootReset)" not in app,
     "runtime no longer direct-jumps into resident Reset_Handler")
need("BOOT:STATE:" in uploader and "resume accepted at offset=" in uploader,
     "uploader resumes from resident offset after host restart")
need("max_reconnects=8" in uploader and "wait_one(BOOT_GLOB,60)" in uploader,
     "uploader reopens boot CDC after real transport loss")
need("fcntl.flock" in uploader and "UPDATE_LOCK.unlink" not in uploader,
     "uploader serializes updates without lock-inode replacement race")
need("ssize_t _write" in boot_syscalls and
     "return (ssize_t)-1;" in boot_syscalls and
     "return (ssize_t)length;" not in boot_syscalls,
     "resident bootloader libc hooks fail closed instead of discarding I/O as success")
need(".thumb_set _write, Platform_UnsupportedSyscall" in app_syscalls and
     "rsbs r0, r0, #0" in app_syscalls,
     "F103 application libc hooks fail closed with compact -1 aliases")
need("CHUNK = 64" in uploader and
     "#define BOOT_DATA_MAX 240U" in boot and
     "#define BOOT_LINE_MAX 600U" in boot,
     "F103 CDC upload uses one-packet chunks within resident parser bounds")
need('"f103": (0x0803E000, 0xF1030101, 0x20005000)' in manifest,
     "manifest generator supports F103 resident layout")
need("0x0803E800..0x08040000 is preserved" in provision and
     "flash erase_address 0x0803E000 0x800" in provision and
     "flash erase_address 0x08004000 0x3A000" in provision and
     "dump_image $PERSIST_BEFORE 0x0803E800 0x1800" in provision and
     "dump_image $PERSIST_AFTER 0x0803E800 0x1800" in provision and
     'persist_sha_before="$(sha256sum' in provision and
     'persist_sha_after="$(sha256sum' in provision,
     "ST-Link rescue preserves and hashes F103 persistent partition")
need("0x414" in provision and 'flash_kb" -eq 256' in provision,
     "ST-Link rescue refuses wrong F103 target geometry")
need("FW:INFO:V3:F103RC:256K" in app,
     "256 KiB runtime exposes unambiguous board identity")
f103_env = pio.split("[env:f103_256k_usbboot]", 1)[1]
need("-DUSER_VECT_TAB_ADDRESS" in f103_env and
     "-DVECT_TAB_OFFSET=0x00004000U" in f103_env,
     "F103RC application relocates IRQ vector table to app base")
need('#if !defined(BOARD_F103C8) || defined(BOARD_F103_256K)\n  if (!strcmp(command, "TFT:STATUS"))' in app,
     "256 KiB runtime retains TFT/touch/SPI diagnostics")
need('#if !defined(BOARD_F103C8) || defined(BOARD_F103_256K)\n  if (!std::strncmp(command, "EEPROM:GET:", 11))' in app,
     "256 KiB runtime retains persistent config diagnostics")
need("BOOT_LOCKED=" in provision and "APP_LOCKED=" in provision and
     'cp -- "$BOOT" "$BOOT_LOCKED"' in provision and
     'cp -- "$APP" "$APP_LOCKED"' in provision,
     "ST-Link rescue freezes build artifacts before flash")
need("REFUSED success: executable app is not sufficient proof" in provision and
     "application executes via SWD" in provision,
     "SWD execution alone cannot report provisioning success")
need("REFUSED: runtime CDC is in use" in provision and
     "cannot prove heartbeat/session exclusively" in provision,
     "ST-Link rescue refuses ambiguous runtime CDC ownership")
need("deadline = time.monotonic() + 15.0" in provision and
     "HOST:HELLO:" in provision and "USB:STAT:host=1" in provision,
     "ST-Link verifier requires CDC re-enumeration and proven host session")
print("F103_256K_USBBOOT_ROBUSTNESS_SELF_CHECK_PASS")
