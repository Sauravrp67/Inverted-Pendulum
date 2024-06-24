#include "Arduino.h"
#include <cassert>
Port Serial;
unsigned long clock_us;
int pin_levels[8], edges;
void (*handlers[8])();
void (*timer_tick)();
void setup();
void loop();

void sample() {
  clock_us += 10000;
  loop();
  for (int i = 0; i < 160; ++i) {
    timer_tick();
  }
}

void command(std::string line) {
  Serial.input = line + "\n";
  while (Serial.available()) {
    loop();
  }
  sample();
}

int main(int argc, char **) {
  setup();
  command("swing");
  assert(!edges);
  command("zero");
  command("balance");
  assert(!edges && Serial.message == "Outside capture range.");
  command("swing-up");
  assert(!edges);
  command("swing");
  if (argc == 2) {
    assert(edges == 0 &&
           Serial.message == "Swing-up disabled in firmware settings.");
  } else {
    assert(edges > 0 && Serial.message == "Swing");
    // If the foreground freezes, the timer must stop issuing pulses on its own.
    for (int i = 0; i < 500; ++i) {
      timer_tick();
    }
    int prior = edges;
    for (int i = 0; i < 500; ++i) {
      timer_tick();
    }
    assert(edges == prior);
    clock_us += 30000;
    loop();
    assert(Serial.message == "Missed control sample.");
  }
  command("zero");
  // Half a turn = 180 quadrature cycles on the 1440-count pole encoder.
  for (int i = 0; i < 180; ++i) {
    for (int pin : {PB6, PB7, PB6, PB7}) {
      pin_levels[pin] ^= 1;
      handlers[pin]();
    }
  }
  for (int i = 0; i < 50; ++i) {
    sample();
  }
  command("balance");
  assert(Serial.message == "Balance");
  command("stop");
  int stopped = edges;
  sample();
  assert(edges == stopped);
}
