#include <AccelStepper.h>

// Define stepper motor connections
#define STEPPER_PULSE_PIN 9
#define STEPPER_DIR_PIN 8
#define SW_Pin 18

// Define rotary encoder pins
#define ENC_A 2 // Encoder output A
#define ENC_B 3 // Encoder output B
#define ENC_Z 4 // Encoder zero position (optional)

// PID parameters
float Kp = 0.8;  // Proportional gain
float Ki = 0.5;  // Integral gain
float Kd = 0.01; // Derivative gain
float max_integral = 100; // Limit for integral term to prevent windup

// Deadband values
#define DEADBAND 1
#define SAFETY_LIMIT 320

// Encoder and PID control variables
volatile long pulse_count = 0;
const float pulses_per_revolution = 360.0; // Encoder resolution
volatile float angle = 0.0; // Angular position
volatile float last_angle = 0.0; // Previous angular position
float integral = 0.0;
float previous_error = 0.0;
unsigned long last_time = 0;
unsigned long last_print_time = 0; 
volatile long target_position = 0;

// Setpoint for the PID controller
const float setpoint = 0; // The desired position of the pendulum (upright position)

// Create an instance of the AccelStepper library
AccelStepper stepper(AccelStepper::DRIVER, STEPPER_PULSE_PIN, STEPPER_DIR_PIN);

void setup() {
  // Set maximum speed and acceleration for the stepper motor
  stepper.setMaxSpeed(4000); // Increase the speed for better responsiveness
  stepper.setAcceleration(4000); // Increase the acceleration

  // Initialize the serial communication
  Serial.begin(115200);

  // Set encoder pins as inputs
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_Z, INPUT_PULLUP); // Optional zero position

  // Attach interrupts to encoder pins
  attachInterrupt(digitalPinToInterrupt(ENC_A), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), readEncoder, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(ENC_Z), resetPosition, FALLING); // Optional zero position

  // Print initial values
  Serial.println("Encoder and PID controller initialized.");

  last_time = millis();
  last_print_time = millis(); 
}

void loop() {
  // PID control loop
  unsigned long now = millis();
  float time_change = (now - last_time) / 1000.0;
  if (time_change >= 0.01) {
    // Update PID every 10ms
    last_time = now;
    float error = setpoint - angle; // Correct error calculation
    integral += error * time_change;

    // Integral windup prevention
    if (integral > max_integral)
      integral = max_integral;
    else if (integral < -max_integral)
      integral = -max_integral;

    float derivative = (error - previous_error) / time_change;
    float output = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;

    // Safety check: If the angle is within the safety limit, do not run the motor
    
      // Convert PID output to stepper motor steps
        // if (abs(error) > 5) {
        target_position = stepper.currentPosition() + (long)output;
        stepper.moveTo(-target_position);
        // }
    

    // Print values once per second
    if (now - last_print_time >= 1000) {
      last_print_time = now;
      Serial.print("Angle: ");
      Serial.println(angle);
      Serial.print("Error: ");
      Serial.println(error);
      Serial.print("Integral: ");
      Serial.println(integral);
      Serial.print("Derivative: ");
      Serial.println(derivative);
      Serial.print("Output: ");
      Serial.println(output);
      Serial.print("Target Position: ");
      Serial.println(target_position);
      Serial.print("Current Position: ");
      Serial.println(stepper.currentPosition());
      Serial.print("Motor Status: ");
      Serial.println(abs(angle) < SAFETY_LIMIT ? "Running" : "Stopped");
    }
  }

  // Run the stepper motor
  stepper.run();
}

void readEncoder() {
  static uint8_t old_AB = 0;
  static int8_t encoder_states[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
  old_AB <<= 2; // Shift previous state
  if (digitalRead(ENC_A)) old_AB |= 0x02; // Add current state of A
  if (digitalRead(ENC_B)) old_AB |= 0x01; // Add current state of B
  pulse_count += encoder_states[old_AB & 0x0f];

  // Update the angle
  angle = (pulse_count / pulses_per_revolution) * 360.0;
}

void resetPosition() {
  pulse_count = 0; // Reset pulse count on zero position
  angle = 0.0; // Reset the angle to zero
}
