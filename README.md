# STM32F103C8T6 gateway firmware

Project ini hanya menargetkan STM32F103C8T6 dengan layout flash resmi 64 KiB.

## PlatformIO environments

Tersedia tepat tiga environment:

- `f103_bootloader` — resident USB CDC bootloader pada 0x08000000..0x08003FFF.
- `f103_stlink` — aplikasi C8 pada 0x08002000 dan provisioning bootloader + aplikasi melalui ST-Link.
- `f103_usb` — aplikasi C8 pada 0x08002000 dan update melalui resident USB CDC bootloader.

Default environment adalah `f103_usb`.

## Memory layout

Target resmi tetap 64 KiB Flash dan 20 KiB SRAM, termasuk bila board fisik adalah clone yang melaporkan kapasitas berbeda.

- Bootloader: 0x08000000..0x08001FFF, maksimum 8 KiB.
- Application: 0x08002000..0x0800F7EF, maksimum 55280 byte.
- Application metadata: mulai 0x0800F7F0.
- Persistent config: 0x0800F800..0x0800FFFF, 2 KiB.
- SRAM: 0x20000000..0x20004FFF, 20 KiB.

Linker yang dipakai:
- `linker/STM32F103C8TX_BOOT.ld`
- `linker/STM32F103C8TX_USB_APP.ld`

## Build

```bash
cd /home/otomasi2/forclift/F4gateway

pio run -e f103_bootloader
pio run -e f103_stlink
pio run -e f103_usb
```

## Upload

Provisioning awal melalui ST-Link:

```bash
pio run -e f103_stlink -t upload
```

Perintah tersebut membangun bootloader lalu menulis:
- bootloader ke 0x08000000;
- aplikasi ke 0x08002000;
- tidak menulis area persistent 0x0800F800..0x0800FFFF.

Update berikutnya melalui USB CDC:

```bash
pio run -e f103_usb -t upload
```

Bootloader saja melalui ST-Link:

```bash
pio run -e f103_bootloader -t upload
```

## Hardware

- HSE 8 MHz, SYSCLK 72 MHz, USB FS 48 MHz.
- USB CDC: PA11/PA12.
- ILI9341 + XPT2046: SPI1.
- TFT CS PB0, DC PB1, RESET PB10, touch CS PA4.
- Buzzer: TIM1 CH1 / PA8.
- BTS7960 winch: LPWM PA3 (TIM2 CH4 timing + DMA), RPWM PB8 (TIM4 CH3 timing + DMA), 1 kHz hardware-timed PWM.
- Limit switch aktif-LOW: PB6 TOP, PB7 BOTTOM.
- Application watchdog: TIM3.

Token protokol host seperti `F4X3` dipertahankan hanya sebagai kompatibilitas wire protocol; token tersebut bukan pilihan MCU.
