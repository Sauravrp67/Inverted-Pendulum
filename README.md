# Inverted Pendulum

Firmware and tooling for a rotary inverted-pendulum (cart-on-rail) balancer. A
1440-count optical quadrature encoder measures the pendulum angle; a PID
controller drives a stepper motor (via STEP/DIR) to keep it upright.

One shared codebase builds for **Arduino Uno/Nano, Arduino Mega, ESP32, and the
STM32 Blue Pill**, using hand-written Makefiles over the raw toolchains
(`avr-gcc`, `xtensa-esp32-elf`, `arm-none-eabi-gcc`) and flashing via
`avrdude` / `esptool` / **OpenOCD**.

## Layout

```
lib/        Reusable libraries
  PIDController/      pure C++ control law (host-testable)
  QuadratureEncoder/  pure decoder core + Arduino ISR glue
  StepperDriver/      AccelStepper wrapper
  BoardConfig/        per-board pin map (BOARD_* macros)
src/main.cpp          canonical balancer (tuned "PID5" baseline)
examples/             single-purpose bring-up sketches
tests/                host unit tests (no hardware)
make/  boards/        build system (toolchain + per-board fragments)
tools/visualization.py  live serial PID telemetry plot
docs/                 wiring + tuning plots
archive/              original experiment sketches (history)
```

## Quick start

```sh
# 1. One-time: install toolchains + Arduino core sources (Debian/Ubuntu)
scripts/install_toolchains.sh            # or: avr | stm32 | esp32 | cores

# 2. Host unit tests (no hardware required)
make test

# 3. Build + flash for a board
make BOARD=uno      build flash
make BOARD=mega     build flash
make BOARD=bluepill build flash          # STM32 via OpenOCD + ST-Link
. $IDF_PATH/export.sh                     # ESP32 only: load ESP-IDF env first
make BOARD=esp32    build flash           # ESP32 via idf.py (ESP-IDF)

# 4. Watch the controller live (set your serial port)
python tools/visualization.py --port /dev/ttyUSB0
```

Boards: `uno`, `mega`, `esp32`, `bluepill`. Pin maps and wiring are in
[`docs/wiring.md`](docs/wiring.md).

## Control baseline

`src/main.cpp` reproduces the tuned **PID5** result: `Kp=1.2, Ki=0.11,
Kd=0.006`, integral clamp ±1000, output mapped to stepper speed as
`16 × output`. Tuning plots are in [`docs/tuning/`](docs/tuning/). Send `reset`
over serial to zero the encoder count and controller state.

## Tooling notes

- `make test` runs immediately (needs only `g++`/`make`).
- On-device builds require `scripts/install_toolchains.sh` first.
- **AVR/STM32** compile the Arduino core from source with the flat Makefiles.
- **ESP32** is different: it builds through **ESP-IDF** (`make BOARD=esp32 …`
  wraps `idf.py`), with `arduino-esp32` as a component in `boards/esp32/`. You
  must install ESP-IDF and `source $IDF_PATH/export.sh` before building. Keep the
  ESP-IDF version aligned with the pinned `arduino-esp32` release.
