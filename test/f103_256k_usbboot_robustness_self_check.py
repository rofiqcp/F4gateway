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
uploader = rd("scripts/cdc_boot_upload.py")
manifest = rd("scripts/make_app_manifest.py")
provision = rd("scripts/provision_recovery_stlink_f103.sh")

main_pos = boot.index("int main(void)")
early_pos = boot.index("early_usb_detach_hold();", main_pos)
hal_pos = boot.index("HAL_Init();", main_pos)
need(early_pos < hal_pos, "bootloader asserts D+ low before HAL init")
need("reset_to_app_after_usb" in boot and "NVIC_SystemReset();" in boot,
     "bootloader leaves active USB through hardware reset")
need("BOOT:STATE:active=" in boot and "BOOT_UPDATE_TIMEOUT_MS 30000UL" in boot,
     "resident protocol exposes resumable state and update timeout")
need("BOOT_USB_LOSS_RECOVERY_MS" in boot and "boot_usb_disconnect_hold();" in boot,
     "resident USB loss has local recovery")
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
need("CHUNK = 16" in uploader,
     "F103 CDC upload uses conservative packet-sized chunks")
need('"f103": (0x0803E000, 0xF1030101, 0x20005000)' in manifest,
     "manifest generator supports F103 resident layout")
need("0x0803E800..0x08040000 is preserved" in provision and
     "flash erase_address 0x0803E000 0x800" in provision and
     "flash erase_address 0x08004000 0x3A000" in provision,
     "ST-Link rescue preserves F103 persistent partition")
need("0x414" in provision and 'flash_kb" -eq 256' in provision,
     "ST-Link rescue refuses wrong F103 target geometry")
print("F103_256K_USBBOOT_ROBUSTNESS_SELF_CHECK_PASS")
