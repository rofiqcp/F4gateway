# F4gateway v2 source layout

The production firmware is intentionally self-contained under `src/`.
There is no project-level `include/` or `lib/` directory.

- `main.cpp` — application orchestration, USB host protocol, watchdog and safety.
- `BoardSupport.*` — STM32F411 clocks, GPIO, SPI, timers and low-level board services.
- `Bts7960Winch.*` — BTS7960 fork/winch state machine and PB6/PB7 limit-switch safety.
- `HmiDisplay.*` — ILI9341/XPT2046 hardware driver and SPI recovery.
- `HmiOperatorUi.h` — single production operator UI renderer and touch actions.
- `HmiTouchService.*` — RAWTOUCH and five-point calibration/service workflow.
- `HmiConfig.h`, `HmiDiagnostics.h`, `Theme.h`, `Icons.h`, `SplashScreen.h` — HMI definitions.
- `Telemetry.*` — host telemetry model and F4X3 parser.
- `PersistentConfigStore.*` — persistent settings journal.
- `usb/` — USB CDC runtime transport.

Normal field update uses the resident CDC bootloader. ST-Link is only required to
provision/recover the resident bootloader and application layout.
