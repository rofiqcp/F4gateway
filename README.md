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

Default application berada pada `0x08008000`; jangan menulis image application mentah ke `0x08000000` karena sektor awal dipakai recovery bootloader.

## Branch dan integrasi AGV

Branch default repository ini adalah `v1`. Workflow sinkronisasi:

1. Commit dan push perubahan firmware di `F4gateway`.
2. Di repository AGV, update pointer submodule `F4gateway` ke commit tersebut.
3. Commit dan push pointer submodule pada branch AGV `v1`.

Build terakhir harus lolos `pio run` sebelum pointer submodule diperbarui.
