# TAP CONTROL UPDATE

Versi ini mengubah kontrol ACTUATOR dari **hold/release** menjadi **one-tap latched control**.

- `MAJU`: tap sekali -> maju terus sampai STOP/perintah pengaman.
- `MUNDUR`: tap sekali -> mundur terus sampai STOP/perintah pengaman.
- `KIRI`: tap sekali -> target steering -15° (configurable).
- `KANAN`: tap sekali -> target steering +15° (configurable).
- `0°`: tap sekali -> steering center.
- `STOP`: berhenti langsung.
- Melepas jari tidak menghentikan kendaraan.
- Keluar dari halaman ACTUATOR otomatis STOP agar tombol STOP tidak tersembunyi saat kendaraan masih bergerak.
- AUTO / system-not-ready / ESC-not-ready juga membatalkan drive latched.

Preset steering di `include/Config.h`:

```cpp
STEER_LEFT_PRESET_DEG  = -15.0f;
STEER_RIGHT_PRESET_DEG =  15.0f;
```
