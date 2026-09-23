#!/usr/bin/env python3
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
def rd(path): return (ROOT / path).read_text(errors="replace")
def need(ok, message):
    if not ok:
        raise SystemExit("FAIL " + message)
    print("PASS " + message)

pio = rd("platformio.ini")
board = rd("src/BoardSupport.cpp")
usbc = rd("src/usb/usbd_conf.c")
app_ld = rd("linker/STM32F103C8TX_USB_APP.ld")
boot_ld = rd("linker/STM32F103C8TX_BOOT.ld")
boot = rd("src/bootloader/main.c")
mcu = rd("src/McuHal.h")
provision = rd("scripts/provision_f103_stlink.sh")
boot_upload = rd("scripts/upload_f103_bootloader.sh")

envs = re.findall(r"^\[env:([^]]+)\]", pio, re.M)
need(envs == ["f103_bootloader", "f103_stlink", "f103_usb"],
     "PlatformIO exposes exactly the three F103C8 environments")
need("board = genericSTM32F103C8" in pio and "-DSTM32F103xB" in pio and
     "-DBOARD_F103C8=1" in pio,
     "PlatformIO target is explicitly STM32F103C8T6")
need(pio.count("board_upload.maximum_size = 55280") == 2,
     "USB/ST-Link application slot is capped at 55280 bytes")
need("board_upload.maximum_size = 8192" in pio,
     "resident bootloader is capped at 8 KiB")
need("reset run; shutdown" in provision and
     "mwh 0x40006C04 0x0000" in provision and
     "mwh 0x40006C28 0x0000" in provision,
     "ST-Link provisioning clears stale boot request and leaves MCU running")
need("reset run; shutdown" in boot_upload and
     "mwh 0x40006C04 0x0000" in boot_upload,
     "bootloader-only upload also clears stale boot request and resumes MCU")

need(re.search(r"FLASH\s+\(rx\).*ORIGIN\s*=\s*0x8002000.*LENGTH\s*=\s*0xD7F0", app_ld) is not None,
     "application linker starts at 0x08002000 with C8-safe limit")
need(re.search(r"FLASH\s+\(rx\).*ORIGIN\s*=\s*0x8000000.*LENGTH\s*=\s*8K", boot_ld) is not None,
     "bootloader linker occupies the first 8 KiB")
need(re.search(r"RAM\s+\(xrw\).*ORIGIN\s*=\s*0x20000000.*LENGTH\s*=\s*20K", app_ld) is not None,
     "application linker enforces official 20 KiB SRAM")
need("-DPERSIST_STORAGE_BASE=0x0800F800UL" in pio and
     "-DPERSIST_STORAGE_LIMIT=0x08010000UL" in pio,
     "persistent config owns the final 2 KiB")

need("#define APP_BASE 0x08002000UL" in boot and
     "#define APP_LIMIT 0x0800F7F0UL" in boot and
     "#define APP_META_BASE 0x0800F7F0UL" in boot and
     "#define PERSIST_BASE 0x0800F800UL" in boot,
     "resident updater matches application/metadata/persistent layout")
need("FLASH_PAGE_C8_BYTES 0x400UL" in boot and
     "FLASH_PAGE_CLONE_414_BYTES 0x800UL" in boot and
     "DBGMCU->IDCODE & 0xFFFU" in boot,
     "bootloader selects 1 KiB C8 or 2 KiB clone-0x414 flash pages")
need("RCC_PLL_MUL9" in board and "RCC_USBCLKSOURCE_PLL_DIV1_5" in board,
     "72 MHz core and 48 MHz USB clocks remain explicit")
need("hpcd_USB_FS.Instance = USB" in usbc and "USB_LP_CAN1_RX0_IRQn" in usbc,
     "runtime USB FS is configured for STM32F1")
need("F4gateway now targets STM32F103C8T6 only" in mcu and "stm32f1xx_hal.h" in mcu,
     "MCU abstraction rejects non-F103C8 builds")

need("genericSTM32F103C8" in pio and pio.count("genericSTM32F103C8") == 1,
     "PlatformIO board selection is C8-only")
boot_usb = rd("src/bootloader/usb/boot_usb.c")
need("46,DESC_STRING" in boot_usb and
     "sizeof(product_desc) == 46U" in boot_usb,
     "minimal boot CDC product descriptor length is self-consistent")
text = rd("src/bootloader/usb/boot_usb.c")
need("stm32f1xx_hal.h" in text,
     "minimal resident boot USB uses STM32F1 HAL only")
print("V3_F103C8_SELF_CHECK_PASS")
