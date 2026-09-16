# F4gateway v2

Firmware STM32F411CEU6 untuk gateway USB CDC, HMI TFT/touch, dan kontrol fork BTS7960 pada AGV forklift. Branch produksi saat ini adalah `v2`; repository ini dipakai sebagai Git submodule oleh workspace `/home/otomasi2/forclift`.

## Hardware utama

- MCU: STM32F411CEU6 / WeAct BlackPill.
- Framework: STM32Cube HAL melalui PlatformIO, clock 96 MHz.
- HMI: ILI9341 320x240 + XPT2046 touch pada shared SPI bus.
- USB runtime: USB CDC FS, 1,000,000 baud ke bridge ROS 2.
- Fork BTS7960: RPWM PB8 (UP), LPWM PA2 (DOWN), PWM 1 kHz / 10-bit.
- Limit switch aktif-LOW + pull-up: PB6 = TOP, PB7 = BOTTOM.

MCP2515, libcanard/DroneCAN, dan NEO-3 tidak lagi menjadi bagian firmware produksi `v2`.

## Fitur fork dan keselamatan

- PWM default 500 dari rentang 0..1023.
- `UP/DOWN HOME`, timed 1 s, timed 2 s, dan STOP.
- Debounce/confirmation LS 30 ms dan reversal dead-time 50 ms.
- HOME watchdog 30 s.
- Kedua LS aktif bersamaan dilatch sebagai fault; satu LS aktif adalah kondisi endpoint yang valid.
- Perintah ROS memakai host/session safety gate; kontrol fork lokal HMI tetap memakai common safety gate.
- HMI menampilkan state fork, TOP/BOTTOM LS, `LS READY`, dan fault secara eksplisit.

## USB CDC dan recovery bootloader

Application berjalan dari `0x08004000`. Resident recovery bootloader berada pada Sector 0 dan update normal dilakukan melalui USB CDC tanpa ST-Link.

Layout flash:

- `0x08000000..0x08003FFF`: resident USB CDC recovery bootloader, 16 KiB.
- `0x08004000..0x0805FFFF`: application, maksimum 368 KiB.
- `0x08060000..0x08063FFF`: manifest journal.
- `0x08064000..0x0806BFFF`: persistent config journal.
- `0x0806C000..0x0807DFFF`: persistent reserved area.
- `0x0807E000..0x0807FFFF`: legacy migration reserve; tidak ada runtime DroneCAN/MCP2515 pada v2.

Boot protocol: `proto=3`, layout `AGVBL3-04000-60000`.

## Prasyarat Jetson / ROS 2 Humble

Gunakan Python system ROS secara eksplisit:

```bash
/usr/bin/python3 --version
sudo apt install -y python3-serial
/usr/bin/python3 -c "import serial; print(serial.__version__)"
```

Install udev rule sekali per Jetson:

```bash
sudo ./scripts/install_stm32_udev_rules.sh
```

Rule runtime/boot CDC di-scope ke BlackPill serial `33A433673134`; ST-Link memakai VID:PID STMicroelectronics yang sesuai.

## Build

```bash
pio run -e blackpill_f411ce_v2
```

Build produksi harus selesai `SUCCESS` sebelum flash atau update pointer submodule.

## Upload normal via USB CDC ACM

```bash
pio run -e blackpill_f411ce_v2 -t upload
```

Uploader melakukan handshake runtime `ACK:PONG`, masuk ke resident `BOOT_CDC`, memverifikasi protocol/layout, erase application region, transfer chunk+CRC, menerima `ACK:END:OK`, lalu memastikan runtime CDC kembali dan `ACK:PONG` sehat.

## Provisioning awal / recovery via ST-Link

```bash
./scripts/provision_recovery_stlink.sh
```

Script memverifikasi MCU STM32F411, membangun bootloader+application, menjaga/migrasi persistent area, menulis image secara transactional melalui SWD, dan memverifikasi hasil. Python di script dikunci ke `/usr/bin/python3` agar sama dengan environment ROS/system.

ROM DFU tersedia sebagai jalur recovery alternatif melalui environment `blackpill_f411ce_romdfu`.

## Diagnostik runtime

Command read-only yang aman antara lain:

```text
PING
FW:INFO
LIMITS
WINCH STATUS
TFT:STATUS
TOUCH:STATUS
USB:STATUS
```

Kondisi normal setelah boot mencakup `ACK:PONG`, `LS_READY=1`, TFT `OK`, dan USB CDC runtime kembali ter-enumerasi.

## Pengujian

```bash
for t in test/*_self_check.py; do
  /usr/bin/python3 "$t" || exit 1
done
```

Test penting mencakup parity fitur project F4 lama, BTS7960/LS safety, HMI display/touch, flash layout, persistent config, USB transport/session, recovery bootloader, dan verifikasi MCP2515 removal.

## Struktur repository

- `src/`: firmware application dan semua header production.
- `src/usb/`: USB CDC application stack.
- `bootloader/`: resident USB CDC recovery bootloader.
- `linker/`: linker script application relocated ke `0x08004000`.
- `scripts/`: build provenance, manifest, provisioning, CDC/DFU uploader, udev installer.
- `test/`: static/regression self-check.
- `tools/`: utilitas diagnosis/telemetry operator; bukan production firmware.

`include/` dan `lib/` legacy tidak dipakai lagi; source production dikonsolidasikan di `src/`.

## Git dan submodule

Gunakan branch `v2` untuk firmware ini. Urutan sinkronisasi:

1. Build dan jalankan self-check.
2. Commit/push `F4gateway` branch `v2`.
3. Pada repository induk `forclift`, update pointer submodule `F4gateway` dan pastikan `.gitmodules` memakai `branch = v2`.
4. Commit/push repository induk.

Jangan commit `.pio`, build output, Python cache, editor state, atau backup sementara.
