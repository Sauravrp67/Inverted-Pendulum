#include <Arduino.h>

// Define rotary encoder pins
#define ENC_A 2 // Encoder output A
#define ENC_B 3 // Encoder output B
#define ENC_Z 4 // Encoder zero position

volatile long pulse_count = 0;
const float transitions_per_revolution = 1440.0; // Encoder transitions per revolution
volatile float angle = 0.0; // Angular position

void setup() {
  // Set encoder pins and attach interrupts
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_Z, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_Z), resetPosition, FALLING);

  // Start the serial monitor to show output
  Serial.begin(115200);

  // Print initial values
  Serial.println("Encoder initialized.");
}

void loop() {
  static long lastCounter = 0;

  // If count has changed, print the new value to serial
  if (pulse_count != lastCounter) {
    lastCounter = pulse_count;
    Serial.print("Counter: ");
    Serial.println(pulse_count);
    Serial.print("Angle: ");
    Serial.println(angle);
  }

  // Small delay for stability
  delay(10);
}

void readEncoder() {
  // Encoder interrupt routine for both pins. Updates counter
  // if they are valid and have rotated a full indent
  static uint8_t old_AB = 0;  // Lookup table index
  static int8_t encval = 0;   // Encoder value
  static const int8_t enc_states[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0}; // Lookup table

  old_AB <<= 2;  // Remember previous state

  if (digitalRead(ENC_A)) old_AB |= 0x02; // Add current state of pin A
  if (digitalRead(ENC_B)) old_AB |= 0x01; // Add current state of pin B
  
  encval += enc_states[(old_AB & 0x0f)];

  // Update counter if encoder has rotated a full indent, that is at least 4 steps
  if (encval > 3) {        // Four steps forward
    pulse_count++;
    encval = 0;
  }
  else if (encval < -3) {        // Four steps backward
    pulse_count--;
    encval = 0;
  }

  // Update the angle
  angle = (float(pulse_count) / transitions_per_revolution) * 360.0;
}

void resetPosition() {
  pulse_count = 0; // Reset pulse count on zero position
  angle = 0.0; // Reset the angle to zero
  Serial.println("Z pulse detected, counter and angle reset.");
}
