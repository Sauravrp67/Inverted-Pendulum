#define ENC_A 2 // Encoder output A
#define ENC_B 3 // Encoder output B
#define ENC_Z 18 // Encoder zero position

volatile long pulse_count = 0;
const float pulses_per_revolution = 1440.0; // Encoder resolution
volatile float angle = 0.0; // Angular position
volatile float last_angle = 0.0; // Previous angular position

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Set encoder pins as inputs
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_Z, INPUT_PULLUP);

  // Attach interrupts to encoder pins
  attachInterrupt(digitalPinToInterrupt(ENC_A), readEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), readEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_Z), resetPosition, RISING); // Z output pulse on rising edge

  // Print initial values
  Serial.println("Encoder initialized.");
}

void loop() {
  // Only print the angle if it has changed
  if (angle != last_angle) {
    last_angle = angle;
    Serial.print("Angle: ");
    Serial.println(angle);
  }

  // Small delay for stability
  delay(10);
}

void readEncoderA() {
  // Read the state of both channels
  int stateA = digitalRead(ENC_A);
  int stateB = digitalRead(ENC_B);

  // Determine the direction of rotation
  if (stateA == HIGH) {
    if (stateB == LOW) {
      pulse_count++;
    } else {
      pulse_count--;
    }
  } else {
    if (stateB == HIGH) {
      pulse_count++;
    } else {
      pulse_count--;
    }
  }

  // Update the angle
  angle = (pulse_count / pulses_per_revolution) * 360.0;
}

void readEncoderB() {
  // Read the state of both channels
  int stateA = digitalRead(ENC_A);
  int stateB = digitalRead(ENC_B);

  // Determine the direction of rotation
  if (stateB == HIGH) {
    if (stateA == HIGH) {
      pulse_count++;
    } else {
      pulse_count--;
    }
  } else {
    if (stateA == LOW) {
      pulse_count++;
    } else {
      pulse_count--;
    }
  }

  // Update the angle
  angle = (pulse_count / pulses_per_revolution) * 360.0;
}

void resetPosition() {
  pulse_count = 0; // Reset pulse count on zero position
  angle = 0.0; // Reset the angle to zero
}
