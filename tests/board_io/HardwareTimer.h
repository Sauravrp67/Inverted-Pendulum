#pragma once
constexpr int TIM2 = 2, HERTZ_FORMAT = 1;
extern void (*timer_tick)();

struct HardwareTimer {
  HardwareTimer(int) {}

  void setOverflow(int, int) {}

  void attachInterrupt(void (*f)()) { timer_tick = f; }

  void resume() {}
};
