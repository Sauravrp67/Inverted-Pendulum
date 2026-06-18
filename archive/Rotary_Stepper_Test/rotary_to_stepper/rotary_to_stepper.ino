#include <AccelStepper.h>

// Define stepper motor connections
#define STEPPER_PULSE_PIN 9
#define STEPPER_DIR_PIN 8
#define SW_Pin 18
// Define rotary encoder pins
#define ENC_A 2
#define ENC_B 3

// Create an instance of the AccelStepper library
AccelStepper stepper(AccelStepper::DRIVER, STEPPER_PULSE_PIN, STEPPER_DIR_PIN);

volatile int counter = 0;
volatile int angle = 0;

void setup() {
  // Set the maximum speed and acceleration for the stepper motor
  stepper.setMaxSpeed(1000); // Steps per second
  stepper.setAcceleration(100); // Steps per second^2

  // Initialize the serial communication
  Serial.begin(115200);

  // Set encoder pins and attach interrupts
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(SW_Pin,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SW_Pin),reset_counter,LOW);
}

void loop() {
  // Check if the encoder position has changed
  static long lastCounter = 0;
  if (counter != lastCounter) {
  
    lastCounter = counter;
    angle = counter * 12;
    Serial.print("Counter: ");
    Serial.println(counter);
    Serial.print("Angle: ");
    Serial.println(angle);

    stepper.moveTo(counter * 100);
    
  }
  stepper.run();
  // Run the stepper motor
  
}

void readEncoder() {
  static uint8_t old_AB = 3;
  static int8_t encval = 0;
  static const int8_t enc_states[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

  old_AB <<= 2;
  if (digitalRead(ENC_A)) old_AB |= 0x02;
  if (digitalRead(ENC_B)) old_AB |= 0x01;

  encval += enc_states[(old_AB & 0x0f)];

  if (encval > 1) {
    int changevalue = 1;
    counter = counter + changevalue;
    angle = counter * 12;
    encval = 0;
  } else if (encval < -1) {
    int changevalue = -1;
    counter = counter + changevalue;
    angle = counter * 12;
    encval = 0;
  }
}

void reset_counter() {
  counter = 0;

  stepper.setCurrentPosition(0);
}