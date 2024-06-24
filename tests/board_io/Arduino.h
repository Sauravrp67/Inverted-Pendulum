#pragma once
#include <stdint.h>
#include <string>

constexpr int LOW = 0, HIGH = 1, INPUT_PULLUP = 2, OUTPUT = 3, CHANGE = 4;
constexpr int PB6 = 6, PB7 = 7, PA0 = 0, PA1 = 1, PA4 = 4, PA5 = 5;
constexpr double PI = 3.14159265358979323846;
extern unsigned long clock_us;
extern int pin_levels[8], edges;
extern void (*handlers[8])();

inline unsigned long micros() { return clock_us; }

inline void pinMode(int, int) {}

inline void noInterrupts() {}

inline void interrupts() {}

inline int digitalRead(int pin) { return pin_levels[pin]; }

inline void digitalWrite(int pin, int level) {
  if (pin == PA4 && level && !pin_levels[pin]) {
    ++edges;
  }
  pin_levels[pin] = level;
}

inline int digitalPinToInterrupt(int pin) { return pin; }

inline void attachInterrupt(int pin, void (*f)(), int) { handlers[pin] = f; }

struct Port {
  std::string input, message;

  void begin(int) {}

  int available() { return !input.empty(); }

  char read() {
    char ch = input.front();
    input.erase(0, 1);
    return ch;
  }

  void print(const char *) {}

  void print(char) {}

  void print(double, int) {}

  void println(double, int) {}

  void println(const char *s) { message = s; }
};

extern Port Serial;
