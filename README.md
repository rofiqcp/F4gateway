# F4gateway

Firmware STM32F411CEU6 untuk gateway/HMI AGV. Repository ini dipin sebagai Git submodule pada repository `rofiqcp/agv` branch `v1`.

## Hardware

- MCU: STM32F411CEU6 / WeAct BlackPill
- Framework: STM32Cube HAL melalui PlatformIO
- Clock: 96 MHz
- HMI: ILI9341 320x240 + touch
- USB: CDC 1,000,000 baud ke mini-PC/ROS
- Gateway: komunikasi F411 ke ESC/F103 dan sensor NEO-3

## Clone

```bash
git clone -b v1 https://github.com/rofiqcp/F4gateway.git
cd F4gateway
```

Jika bekerja melalui repository AGV:

```bash
git clone --recurse-submodules -b v1 https://github.com/rofiqcp/agv.git
cd agv
git submodule update --init --recursive
```

## Build

```bash
python3 -m pip install --user platformio
pio run
```
## Upload

Normal update melalui resident USB CDC bootloader:

```bash
pio run -t upload
```

Recovery/provisioning melalui ST-Link:

```bash
pio run -e blackpill_f411ce_stlink -t upload
```

ROM DFU tersedia sebagai recovery alternatif:

```bash
pio run -e blackpill_f411ce_romdfu -t upload
```

Default application berada pada `0x08004000`; Sector 0 (16 KiB) dipakai recovery bootloader dan Sector 7 dipakai persistent journal, jadi jangan menulis image application mentah ke `0x08000000`.

Layout flash STM32F411CE 512 KiB yang dipakai:

- `0x08000000..0x08003FFF` — resident recovery bootloader, maksimum 16 KiB.
- `0x08004000..0x0805FFFF` — application, maksimum 368 KiB.
- `0x08060000..0x08063FFF` — append-only manifest journal, 512 slot x 32 byte.
- `0x08064000..0x0806BFFF` — EEPROM emulation/config journal, 32 KiB (682 record slots).
- `0x0806C000..0x0807DFFF` — reserved persistent area, 72 KiB.
- `0x0807E000..0x0807FFFF` — compact DroneCAN DNA journal, 8 KiB (409 journal records).

Bootloader layout ini memakai protocol generation 3 (`AGVBL3-04000-60000`). Board yang masih memakai layout lama `0x08008000` harus diprovision satu kali melalui ST-Link; uploader CDC/ROM-DFU baru menolak layout lama sebelum melakukan erase. Provisioning ST-Link membackup Sector 7 dan journal DNA lama di Sector 6, memigrasikannya ke partition baru, lalu menghapus penuh Sector 1-6 sebelum menulis application baru.

## Branch dan integrasi AGV

Branch default repository ini adalah `v1`. Workflow sinkronisasi:

1. Commit dan push perubahan firmware di `F4gateway`.
2. Di repository AGV, update pointer submodule `F4gateway` ke commit tersebut.
3. Commit dan push pointer submodule pada branch AGV `v1`.

Build terakhir harus lolos `pio run` sebelum pointer submodule diperbarui.
