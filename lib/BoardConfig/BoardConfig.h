// BoardConfig -- per-board pin map and serial baud, selected at compile time.
//
// The build system passes exactly one -DBOARD_* macro (see boards/*.mk):
//   BOARD_UNO      Arduino Uno / Nano   (ATmega328P)
//   BOARD_MEGA     Arduino Mega         (ATmega2560)
//   BOARD_ESP32    ESP32 dev module
//   BOARD_BLUEPILL STM32 Blue Pill      (STM32F103C8)
//
// Pin choices and wiring rationale are documented in docs/wiring.md. On AVR the
// encoder must use the two external-interrupt pins (2, 3); ESP32 and STM32
// allow interrupts on any GPIO, so any free pins work.
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#if defined(BOARD_UNO)

#define ENC_A_PIN 2
#define ENC_B_PIN 3
#define ENC_Z_PIN 255  // 255 = not wired
#define STEPPER_PULSE_PIN 9
#define STEPPER_DIR_PIN 8
#define SERIAL_BAUD 115200

#elif defined(BOARD_MEGA)

#define ENC_A_PIN 2
#define ENC_B_PIN 3
#define ENC_Z_PIN 18
#define STEPPER_PULSE_PIN 9
#define STEPPER_DIR_PIN 8
#define SERIAL_BAUD 115200

#elif defined(BOARD_ESP32)

#define ENC_A_PIN 34
#define ENC_B_PIN 35
#define ENC_Z_PIN 32
#define STEPPER_PULSE_PIN 26
#define STEPPER_DIR_PIN 27
#define SERIAL_BAUD 115200

#elif defined(BOARD_BLUEPILL)

#define ENC_A_PIN PB6
#define ENC_B_PIN PB7
#define ENC_Z_PIN PB8
#define STEPPER_PULSE_PIN PA9
#define STEPPER_DIR_PIN PA8
#define SERIAL_BAUD 115200

#else
#error "No BOARD_* macro defined. Build via the Makefile with BOARD=<id>."
#endif

#endif  // BOARD_CONFIG_H
