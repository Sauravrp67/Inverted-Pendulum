#include <AccelStepper.h>

// Define stepper motor connections
#define STEPPER_PULSE_PIN 9
#define STEPPER_DIR_PIN 8

// Define rotary encoder pins
#define ENC_A 2 // Encoder output A
#define ENC_B 3 // Encoder output B

// PID parameters
float Kp = 4.3;  // Proportional gain
float Ki = 0.1;  // Integral gain
float Kd = 0;   // Derivative gain
float max_integral = 1000; // Limit for integral term to prevent windup

// Deadband values
#define DEADBAND 5
#define SAFETY_LIMIT 500

// Encoder and PID control variables
volatile long pulse_count = 0;
const float pulses_per_revolution = 1440.0; // Encoder resolution
volatile float angle = 0.0; // Angular position
float integral = 0.0;
float derivative = 0.0;
float error = 0.0;
float previous_error = 0.0;
float output = 0.0;
unsigned long last_time = 0;
unsigned long last_print_time = 0;
volatile bool start_integral = false; // Flag to indicate when to start PID calculations

// Setpoint for the PID controller
const float setpoint = 0.0; // The desired position of the pendulum (upright position)

// Create an instance of the AccelStepper library
AccelStepper stepper(AccelStepper::DRIVER, STEPPER_PULSE_PIN, STEPPER_DIR_PIN);

void setup() {
  // Set maximum speed and acceleration for the stepper motor
  stepper.setMaxSpeed(4000); // Increase the speed for better responsiveness
  stepper.setAcceleration(8000); // Increase the acceleration

  // Initialize the serial communication
  Serial.begin(115200);

  // Set encoder pins as inputs
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  // Attach interrupts to encoder pins
  attachInterrupt(digitalPinToInterrupt(ENC_A), readEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), readEncoderB, CHANGE);

  // Print initial values
  Serial.println("Encoder and PID controller initialized.");

  last_time = micros();
  last_print_time = micros();
}

void loop() {
  // PID control loop
  unsigned long now = micros();
  float time_change = (now - last_time) / 1000.0;

  if(time_change >= 0.01) {
    last_time = now;
    error = setpoint - angle; // Calculate error

    integral += error * time_change;
    // Integral windup prevention
    if (integral > max_integral) integral = max_integral;
    else if (integral < -max_integral) integral = -max_integral;

    derivative = (error - previous_error) / time_change;
    output = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;

    // Convert PID output to stepper motor steps
    stepper.setSpeed(output);
    stepper.runSpeed();
  }

  // Print values once per second
  if (now - last_print_time >= 1000000) { // 1 second in microseconds
    last_print_time = now;
    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.print(" Error: ");
    Serial.print(error);
    Serial.print(" Integral: ");
    Serial.print(integral);
    Serial.print(" Derivative: ");
    Serial.print(derivative);
    Serial.print(" Output: ");
    Serial.print(output);
    Serial.print(" Kp: ");
    Serial.print(Kp);
    Serial.print(" Ki: ");
    Serial.print(Ki);
    Serial.print(" Kd: ");
    Serial.println(Kd);
  }
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
  angle = ((pulse_count / pulses_per_revolution) * 360.0);
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
  angle = ((pulse_count / pulses_per_revolution) * 360.0);
}

void beginIntegralCalculation() {
  start_integral = true;
}
