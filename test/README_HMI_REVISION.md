# ADV HMI — Visual Precise 320×240 + One-Tap Actuator Control

Firmware HMI untuk **Autonomous Delivery Vehicle** pada:

- STM32F411CEU6 (Black Pill)
- ILI9341 320×240 landscape
- XPT2046 touch
- TFT_eSPI
- USB CDC / Serial 115200

Versi ini dibuat agar komposisi layar semirip mungkin dengan visual HMI yang disetujui, tetapi tetap realistis untuk TFT 320×240 dan resource STM32F411. Kontrol ACTUATOR sudah menggunakan **one-tap/tap-to-run**: tombol tidak perlu ditahan.

## 1. Yang berubah

Halaman final:

1. `HOME`
2. `CAMERA`
3. `GPS`
4. `ACTUATOR`

Bottom navigation hanya:

`CAMERA | GPS | ACTUATOR`

Navigasi lama `BACK / NEXT / SENSOR / AUTO / INFO` tidak digunakan lagi.

Battery dummy juga sudah dihapus karena project belum memiliki telemetry baterai yang valid.

## 2. Visual

Palette final:

- Deep navy background
- Dark navy navigation/control tiles
- Warm off-white main cards
- Cyan active accent
- Green ready status
- Red stop/fault status

Tidak ada badge/highlight berwarna di belakang tulisan. Menu aktif hanya memakai icon cyan + garis cyan tipis.

Font utama memakai FreeSans/FreeSansBold dari paket TFT_eSPI sehingga lebih dekat dengan karakter font HMI printer 3D daripada font pixel default.

`include/VisualAssets.h` berisi tiga ilustrasi RGB565 compact:

- delivery vehicle pada HOME
- camera/perception pada CAMERA
- GPS/location pada GPS

Total bitmap sekitar 39 kB flash, sehingga tetap ringan untuk STM32F411.

Lihat:

- `HMI_VISUAL_REFERENCE.png` — target visual
- `HMI_IMPLEMENTATION_PREVIEW.png` — preview layout implementasi 320×240

## 3. Hardware / pin — dipertahankan dari project awal

```text
TFT / touch SPI:
SCK       PA5
MISO      PA6
MOSI      PA7
TFT_CS    PB0
TFT_DC    PB1
TFT_RST   PB2
TOUCH_CS  PA4
```

Touch calibration/correction dari project awal tetap dipertahankan:

```cpp
uint16_t calData[5] = {300, 3600, 300, 3600, 1};
```

serta mapping landscape pada `correctTouchXY()`.

## 4. HOME

Menampilkan:

- speed aktual besar
- `VEHICLE READY / NOT READY / FAULT / ...`
- heading
- visual ADV
- MODE: AUTO / MANUAL
- STATE: STOPPED / RUNNING / STANDBY / FAULT
- health GPS / CAM / ESC pada top bar

## 5. CAMERA

Tidak menampilkan streaming video langsung pada STM32.

HMI menerima ringkasan hasil perception dari komputer/ROS2:

- Camera Ready / Offline
- Perception Active / Waiting
- detected object
- object distance
- drivable area
- obstacle status

## 6. GPS

Menampilkan:

- heading
- latitude
- longitude
- satellite count
- HDOP
- GNSS fix
- IMU ready
- speed

## 7. ACTUATOR

Monitoring:

- mode
- manual speed command
- actual steering
- steering target
- ESC ready
- encoder ready
- RPM

Manual control:

```text
        MAJU
          ↑

KIRI ←   0°   → KANAN

        MUNDUR       STOP
          ↓            ■
```

### Safety interlock

Manual movement hanya aktif saat:

```text
SYSTEM == READY
MODE   == MANUAL
ESC    == READY
```

Steering kiri/kanan/center juga membutuhkan:

```text
ENCODER == READY
```

Pada `AUTO`, tombol movement terlihat disabled dan tidak mengirim command motor.

### One-tap / tap-to-run manual control

Kontrol aktuator sekarang **tidak perlu ditahan**. Satu kali sentuh langsung
melatch perintah sampai operator memilih perintah lain atau menekan `STOP`.

- tap `MAJU` -> kirim `CMD:DRIVE:FWD:<speed>` sekali, kendaraan terus maju
- tap `MUNDUR` -> kirim `CMD:DRIVE:REV:<speed>` sekali, kendaraan terus mundur
- tap `STOP` -> kirim `CMD:DRIVE:STOP` dan drive berhenti
- tap `KIRI` -> target steering langsung ke `STEER_LEFT_PRESET_DEG`
- tap `KANAN` -> target steering langsung ke `STEER_RIGHT_PRESET_DEG`
- tap `0°` -> target steering kembali ke 0°
- melepas jari **tidak** mengirim STOP dan tidak mengulang command

Default steering preset saat ini adalah `-15° / 0° / +15°`, tetap di-clamp
oleh mechanical limit `STEER_MIN_DEG` dan `STEER_MAX_DEG`. Semua nilai ini
dapat dikalibrasi di `include/Config.h`.

Untuk safety, drive latched otomatis dihentikan jika:

- operator menekan `STOP`;
- operator memilih arah drive berlawanan (HMI mengirim STOP terlebih dahulu);
- halaman ACTUATOR ditinggalkan sehingga tombol STOP tidak lagi terlihat;
- mode berubah ke `AUTO`;
- system tidak lagi `READY`; atau
- ESC tidak lagi ready.

Tombol arah yang sedang aktif diberi aksen cyan tipis agar operator tahu command
yang sedang dilatch. `STOP` adalah **software stop**, bukan pengganti physical
emergency stop.

## 8. Command keluar dari HMI

Contoh:

```text
CMD:DRIVE:FWD:20
CMD:DRIVE:REV:20
CMD:DRIVE:STOP
CMD:STEER:-15.0
CMD:STEER:0.0
CMD:STEER:15.0
```

Steering target di-clamp menggunakan:

```cpp
STEER_MIN_DEG
STEER_MAX_DEG
```

Mechanical clamp default saat ini:

```text
STEER_MIN_DEG = -30°
STEER_MAX_DEG = +30°
```

One-tap preset default:

```text
KIRI   = -15°
CENTER =   0°
KANAN  = +15°
```

Sesuaikan seluruh batas/preset dengan mekanik steering kendaraan sebelum uji
bertenaga.

## 9. Telemetry masuk

Satu command per baris (`\n`):

```text
SYS:READY
MODE:MANUAL
STATE:STOPPED
SPD:0.0
HEAD:32.0

GPS:1
FIX:3
LAT:-7.050123
LON:110.440235
SAT:17
HDOP:0.82
IMU:1

CAM:1
PER:1
OBJ:PERSON
DIST:3.24
CONF:92
DRV:1
OBS:0

STEER_TARGET:15.0
STEER_ACTUAL:14.8
STEER_ERR:0.2
RPM:328
ESC:1
ENC:1
MANUAL_SPEED:20
```

Navigation command:

```text
GOTO:HOME
GOTO:CAMERA
GOTO:GPS
GOTO:ACTUATOR
GET:STATE
PING
```

## 10. Build

Dari folder project:

```bash
pio run
```

Upload ST-Link:

```bash
pio run -t upload
```

Monitor:

```bash
pio device monitor -b 115200
```

## 11. Test telemetry tanpa ROS2

Setelah board terhubung sebagai `/dev/ttyACM0`:

```bash
python3 telemetry_demo.py /dev/ttyACM0
```

Atau gunakan HMI demo compile-time dengan mengubah sementara:

```text
-DHMI_DEMO_MODE=1
```

Production default adalah `0`.

## 12. Validasi yang dilakukan pada revisi ini

- host-side C++ syntax validation: PASS
- Python `py_compile`: PASS
- ukuran dan overlap layout diperiksa terhadap canvas 320×240
- touch calibration lama dipertahankan
- bitmap visual disimpan di flash, bukan RAM framebuffer
- tidak menggunakan full-screen sprite 16-bit karena RAM STM32F411 tidak perlu dibebani
- full-screen redraw hanya pada perpindahan page; telemetry update tidak memanggil `fillScreen()`

Full PlatformIO build/upload hardware tidak dapat dijalankan di environment pembuatan file ini karena toolchain PlatformIO dan board fisik tidak tersedia. Jalankan `pio run` pada komputer development sebelum upload pertama.

## 13. File penting

```text
src/main.cpp
include/Config.h
include/Theme.h
include/VisualAssets.h
include/Icons.h
include/TopBar.h
include/BottomMenu.h
include/HomePage.h
include/CameraPage.h
include/GpsPage.h
include/ActuatorPage.h
include/Telemetry.h
include/TouchButtons.h
```

Sebelum tes motor dengan roda menyentuh lantai, verifikasi terlebih dahulu touch coordinate, direction motor, tap-to-run STOP behavior, steering preset/zero, dan mechanical steering limits dengan kondisi kendaraan aman.
