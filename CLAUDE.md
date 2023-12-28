# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Firmware for a rotary inverted-pendulum balancer. A 1440-count quadrature
encoder feeds a PID controller that commands a stepper (STEP/DIR) to stay
upright. One codebase targets four boards (Arduino Uno/Nano, Mega, ESP32, STM32
Blue Pill) via hand-written Makefiles over raw toolchains — there is **no
PlatformIO or arduino-cli**.

## Commands

```sh
make test                              # host unit tests (g++, no hardware)
make -C tests run                      # same, run directly
make BOARD=<id> build|flash|monitor|clean   # id: uno | mega | esp32 | bluepill
make BOARD=uno SRC_DIR=examples/encoder_readout build   # build an example instead of src/
scripts/install_toolchains.sh [all|avr|stm32|esp32|cores]   # one-time toolchain + core setup
```

- The default `make` goal is `help`. `make list-boards` lists targets.
- To run a **single** unit test, temporarily narrow `SRCS` in `tests/Makefile`
  (the harness auto-registers every `TEST()` in the compiled translation units).
- `flash` for `bluepill` shells out to OpenOCD:
  `openocd -f boards/bluepill/openocd.cfg -c "program build/bluepill/fw.elf verify reset exit"`.
- Override the serial/programmer port per invocation: `make BOARD=uno UPLOAD_PORT=/dev/ttyACM0 flash`.

Only `make test` runs end-to-end in a bare checkout — the embedded toolchains
(`avr-gcc`, `arm-none-eabi-gcc`, `xtensa-esp32-elf`, `avrdude`, `esptool`) and
the Arduino core sources under `vendor/` are **not** committed; they come from
`scripts/install_toolchains.sh`. OpenOCD is expected on PATH.

## Architecture

**Portability strategy.** Shared code is written against the Arduino API
(`Serial`, `pinMode`, `attachInterrupt`, `micros`), which exists on all three
cores (ArduinoCore-avr, arduino-esp32, Arduino_Core_STM32). The Makefiles
compile each board's Arduino core + libraries + firmware with that board's raw
toolchain. The same `src/main.cpp` therefore builds for every board.

**The host/firmware split is the key design rule.** Pure algorithm code carries
**no `<Arduino.h>`** so it compiles and is unit-tested with the host `g++`:
- `lib/PIDController/` — the control law (whole class is host code).
- `lib/QuadratureEncoder/` — `QuadratureDecoder` is a header-inline state
  machine (host-testable); the Arduino `QuadratureEncoder` wrapper (pins, ISR
  trampoline) is wrapped in `#ifdef ARDUINO`.
- `lib/StepperDriver/` — entirely `#ifdef ARDUINO` (wraps AccelStepper).

When adding logic, keep decision/math code Arduino-free and put hardware
touches behind `#ifdef ARDUINO` so `tests/` can cover it. `tests/Makefile`
compiles the libs *without* defining `ARDUINO`.

**Build dispatch.** `Makefile` → `boards/<id>.mk` (sets `FAMILY`, `MCU`,
`BOARD_DEF`, upload params) → `make/common.mk` (shared source list + flags) →
`make/<family>.mk`. For `avr`/`stm32` the family fragment is a real compile/
link/flash recipe over the raw toolchain. For `esp32` it is a thin wrapper that
shells out to `idf.py` (see below). Adding an AVR/STM32 board = one `boards/*.mk`
+ a `BoardConfig.h` block, reusing an existing family fragment.

**Pin maps** live only in `lib/BoardConfig/BoardConfig.h`, selected by the
`-DBOARD_*` macro from the board fragment. Never hardcode pins in `src/` or
`lib/` — add them here. On AVR the encoder must stay on D2/D3 (only external-
interrupt pins).

## Things that will bite you

- **Telemetry format is a contract.** `src/main.cpp` prints
  `Angle: a Error: e Integral: i Derivative: d Output: o Kp: .. Ki: .. Kd: ..`
  and `tools/visualization.py` parses it positionally (`parts[1,3,5,7,9,11,13,15]`).
  Change one side and the plotter silently stops updating — change both.
- **PID5 is the tuned baseline.** Default gains `Kp=1.2, Ki=0.11, Kd=0.006`
  (integral clamp ±1000) and the `16×output` speed scale are calibrated against
  the 1440-counts/rev encoder resolution. The decoder counts every valid A/B
  transition to preserve that resolution; don't switch it to a per-detent
  counter without re-tuning. `tests/test_pid.cpp` guards the defaults.
- **Control-loop dt is in milliseconds** (`(micros()-last)/1000.0`), matching
  the original sketches; `PIDController::update(measurement, dt)` expects the
  same unit. The historical `>= 0.01` gate is ~10 µs, i.e. effectively
  every-loop — preserved deliberately, not a 10 ms period.
- **ESP32 builds differently from the other boards.** AVR and STM32 compile the
  Arduino core from source with a flat Makefile. ESP32 cannot — arduino-esp32 is
  an ESP-IDF project (FreeRTOS + components + generated sdkconfig). So
  `make/esp32.mk` just wraps `idf.py` against the IDF project in `boards/esp32/`,
  where arduino-esp32 is an `EXTRA_COMPONENT_DIRS` component and `boards/esp32/
  main/CMakeLists.txt` compiles the same `src/`+`lib/` sources. `setup()`/`loop()`
  run because `sdkconfig.defaults` sets `CONFIG_AUTOSTART_ARDUINO=y`. The main
  component must define `ARDUINO`/`BOARD_ESP32` itself (IDF doesn't), so the
  `#ifdef ARDUINO` guards compile. Requires ESP-IDF installed + `.
  $IDF_PATH/export.sh` sourced so `idf.py` and xtensa are on PATH. The ESP-IDF
  version must match the pinned arduino-esp32 release (see
  `scripts/install_toolchains.sh`: IDF v5.1.x ↔ arduino-esp32 3.0.x).
- `archive/` holds the original `.ino` experiments (PID..PID5, encoder/stepper
  tests) for reference only — not built. `examples/` are the maintained,
  library-based equivalents.
