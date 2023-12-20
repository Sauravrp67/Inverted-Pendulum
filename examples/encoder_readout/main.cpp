// Example: print the quadrature encoder angle whenever it changes.
// Refactor of archive/Rotary_Encoder using the QuadratureEncoder library.
#include <Arduino.h>

#include "BoardConfig.h"
#include "QuadratureEncoder.h"

QuadratureEncoder encoder(ENC_A_PIN, ENC_B_PIN, ENC_Z_PIN, 1440.0f);

void setup() {
  Serial.begin(SERIAL_BAUD);
  encoder.begin();
  Serial.println("Encoder initialized.");
}

void loop() {
  static long last_count = 0;
  long now = encoder.count();
  if (now != last_count) {
    last_count = now;
    Serial.print("Counter: ");
    Serial.print(now);
    Serial.print(" Angle: ");
    Serial.println(encoder.angleDegrees());
  }
  delay(10);
}
