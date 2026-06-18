# Wiring

Pin assignments are defined per board in `lib/BoardConfig/BoardConfig.h`,
selected at build time by the `-DBOARD_*` macro from `boards/<id>.mk`.

## Signals

- **Encoder A / B** — quadrature channels from the optical rotary encoder
  (1440 counts/rev). Must be on interrupt-capable pins.
- **Encoder Z** — optional index/zero pulse; resets the count when wired
  (`255` in BoardConfig means "not connected").
- **STEP (pulse) / DIR** — to the stepper driver (e.g. A4988/DRV8825/TB6600)
  in `AccelStepper::DRIVER` mode.

## Per-board pin map

| Signal | Uno/Nano | Mega | ESP32 | Blue Pill (F103) |
|---|---|---|---|---|
| ENC_A | D2 | D2 | GPIO34 | PB6 |
| ENC_B | D3 | D3 | GPIO35 | PB7 |
| ENC_Z | — | D18 | GPIO32 | PB8 |
| STEP  | D9 | D9 | GPIO26 | PA9 |
| DIR   | D8 | D8 | GPIO27 | PA8 |

### Notes

- **AVR (Uno/Nano):** only D2 and D3 support external interrupts, so the
  encoder A/B are fixed there. Z is left unwired (only 2 interrupt lines).
- **ESP32:** GPIO34/35 are input-only, which suits the encoder inputs. Any GPIO
  can host an interrupt; STEP/DIR use output-capable GPIO26/27.
- **STM32 Blue Pill:** every GPIO is interrupt-capable. Flash and debug via
  ST-Link on the SWD header (PA13/PA14), independent of these signal pins.

Adjust any of these in `BoardConfig.h` to match your harness.
