#pragma once
#include "pins.h"

#ifdef ARDUINO_ARCH_STM32
#include <HardwareTimer.h>
#endif

namespace pulses {
constexpr int frequency = 16000;
volatile int16_t rate = 0;
volatile int32_t count = 0;
volatile uint16_t lease = 0;
int motor_sign = 1;

inline void tick() {
  static int phase = 0, direction = 0;
  static bool high = false;
  if (high) {
    digitalWrite(pins::step_pin, LOW);
    high = false;
  }
  if (!lease || !rate) {
    phase = 0;
    return;
  }
  --lease;

  const int sign = rate > 0 ? 1 : -1;
  if (sign != direction) {
    direction = sign;
    digitalWrite(pins::dir_pin, sign * motor_sign > 0 ? HIGH : LOW);
    phase = 0;
    return; // give DIR a full timer tick before STEP
  }
  phase += sign * rate;
  if (phase >= frequency) {
    phase -= frequency;
    digitalWrite(pins::step_pin, HIGH);
    high = true;
    count += sign;
  }
}

inline void begin(int sign) {
  motor_sign = sign;
  pinMode(pins::step_pin, OUTPUT);
  pinMode(pins::dir_pin, OUTPUT);
  digitalWrite(pins::step_pin, LOW);
#ifdef __AVR__
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS10);
  TCNT1 = 0;
  OCR1A = F_CPU / frequency - 1;
  TIMSK1 = _BV(OCIE1A);
  interrupts();
#else
  static HardwareTimer timer(TIM2);
  timer.setOverflow(frequency, HERTZ_FORMAT);
  timer.attachInterrupt(tick);
  timer.resume();
#endif
}

inline void set(int16_t steps_s) {
  noInterrupts();
  rate = steps_s;
  lease = frequency / 50; // stop pulses if the main loop stalls for 20 ms
  interrupts();
}
} // namespace pulses

#ifdef __AVR__
ISR(TIMER1_COMPA_vect) { pulses::tick(); }
#endif
