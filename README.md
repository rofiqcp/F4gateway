# F4gateway v3

Firmware gateway AGV forklift untuk **STM32F103C8T6**. Branch `v3` mempertahankan fungsi inti firmware v2—USB CDC, HMI TFT/touch, E-stop, watchdog, BTS7960, limit switch, dan telemetry host—tetapi dipadatkan agar aman pada SRAM 20 KiB dan Flash 64 KiB milik F103C8T6.

Environment F411/F401 lama tetap tersedia sebagai regression/recovery target. Default build pada branch ini adalah `bluepill_f103c8`.

## Hardware v3

- MCU: STM32F103C8T6, HSE 8 MHz, SYSCLK 72 MHz.
- USB device: 48 MHz dari PLL / 1.5, USB CDC FS pada PA11/PA12.
- HMI: ILI9341 320x240 + XPT2046 pada SPI1.
- TFT: CS PB0, DC PB1, RESET PB2; touch CS PA4.
- Buzzer: TIM1 CH1 pada PA8.
- Fork BTS7960: LPWM PA2 / TIM2 CH3, RPWM PB8 / TIM4 CH3.
- Limit switch aktif-LOW + pull-up: PB6 = TOP, PB7 = BOTTOM.
- Application watchdog: TIM3, terpisah dari SysTick.

## Layout Flash F103C8

Flash fisik dibatasi sesuai spesifikasi STM32F103C8T6, bukan kapasitas clone yang kadang terdeteksi lebih besar.

- `0x08000000..0x0800F7FF`: application, maksimum 62 KiB.
- `0x0800F800..0x0800FFFF`: persistent config, 2 KiB / dua page.
- SRAM: 20 KiB.
Linker v3 berada di `linker/STM32F103C8TX_APP.ld`. Firmware tidak boleh melewati batas application karena dua page terakhir dipakai journal konfigurasi.

## Fitur keselamatan

- E-stop HMI diproses sebelum kontrol halaman lain dan memaksa command motion ke nol.
- Safety STOP direfresh secara periodik selama E-stop ter-latch.
- Watchdog memonitor progress main loop dengan timer independen.
- SPI wait dan transfer memiliki timeout serta recovery path.
- Limit switch memiliki confirmation/debounce 30 ms.
- Reverse direction BTS7960 memakai dead-time 50 ms.
- HOME memiliki watchdog 30 s.
- TOP dan BOTTOM aktif bersamaan dilatch sebagai fault.
- Flash persistent tidak ditulis saat aktuator sedang bergerak.

## Build v3

Build default:

```bash
cd /home/otomasi2/forclift/F4gateway
pio run
```

Atau eksplisit:

```bash
pio run -e bluepill_f103c8
```

Build tervalidasi pada F103C8 menggunakan 61.664 byte (sekitar 60,2 KiB) dari region aplikasi 62 KiB dan 14.676 byte SRAM dari 20 KiB. Headroom Flash aplikasi sekitar 1.824 byte.

## Upload STM32F103C8T6

v3 menggunakan ST-Link. Tidak ada resident CDC bootloader F411 pada target F103C8.

Sebelum flash, pastikan target yang terhubung benar-benar STM32F103C8T6. Setelah identitas target terverifikasi:

```bash
pio run -e bluepill_f103c8 -t upload
```

Jangan menjalankan upload ke ST-Link yang belum dipastikan terhubung ke MCU F103C8T6.

## USB runtime

Setelah firmware berjalan, F103 memakai USB CDC FS. Command penting yang tetap tersedia antara lain:

```text
PING
FW:INFO
GET:STATE
USB:STATUS
USB:RECOVER
FAULT:STATUS
LIMITS
WINCH STATUS
HMI RESET
```

Parser F103 tetap menerima telemetry F4X3 yang diperlukan untuk kamera/perception, IMU, dan Nav2 serta jalur legacy yang dipakai HMI.
## Regression target yang tetap tersedia

Port v3 tidak menghapus target lama:

```bash
pio run -e blackpill_f411ce_romdfu
pio run -e blackpill_f411cc
pio run -e blackpill_f401cd
```

Ketiga environment tersebut harus tetap build SUCCESS sebelum perubahan v3 dianggap aman.

## Pengujian

Jalankan seluruh self-check:

```bash
for t in test/*_self_check.py; do
  /usr/bin/python3 "$t" || exit 1
done
```

Self-check v3 khusus F103 memvalidasi clock, USB FS, TIM3 watchdog, pin BTS7960/limit switch, layout 62 KiB + 2 KiB, parser telemetry, dan ketersediaan target F411.

## Git

Branch produksi port F103 adalah `v3`. Jangan commit `.pio`, build output, cache Python, editor state, atau backup sementara.

Urutan aman:

1. Jalankan seluruh self-check.
2. Build `bluepill_f103c8`.
3. Build regression F411CE/F411CC/F401CD.
4. Commit perubahan pada branch `v3`.
5. Push `v3` dan baru update pointer submodule repository induk bila diperlukan.
