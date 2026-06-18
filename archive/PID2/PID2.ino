#include <AccelStepper.h>

// Define stepper motor connections
#define STEPPER_PULSE_PIN 9
#define STEPPER_DIR_PIN 8

// Define rotary encoder pins
#define ENC_A 2 // Encoder output A
#define ENC_B 3 // Encoder output B
// #define ENC_Z 18
// #define INTEGRAL_INTERRUPT 19
 // Encoder zero position (optional)

// PID parameters
float Kp = 100;  // Proportional gain
float Ki = 0.8;   // Integral gain
float Kd = 25;   // Derivative gain
float max_integral = 1000; // Limit for integral term to prevent windup

// Deadband values
#define DEADBAND 5
#define SAFETY_LIMIT 500

// Encoder and PID control variables
volatile long pulse_count = 0;
const float pulses_per_revolution = 1440.0; // Encoder resolution
volatile float angle = 0.0; // Angular position
volatile float last_angle = 0.0; // Previous angular position
float integral = 0.0;
float derivative = 0.0;
float error = 0.0;
float previous_error = 0.0;
float output = 0.0;
unsigned long last_time = 0;
unsigned long last_print_time = 0;
volatile long target_position = 0;
volatile bool start_integral = false; // Flag to indicate when to start PID calculations

// Setpoint for the PID controller
const float setpoint = 186.0; // The desired position of the pendulum (upright position)

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
  // pinMode(ENC_Z,INPUT);
  
  

  // Attach interrupts to encoder pins
  attachInterrupt(digitalPinToInterrupt(ENC_A), readEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), readEncoderB, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(ENC_Z), resetPosition, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(INTEGRAL_INTERRUPT), beginIntegralCalculation,CHANGE);
 // Optional zero position

  // Print initial values
  Serial.println("Encoder and PID controller initialized.");

  last_time = micros();
  last_print_time = micros();
}

void loop() {
  
  // PID control loop
  unsigned long now = micros();

  float time_change = (now - last_time) / 1000.0;

  if(time_change >= 0.01 ) {
  last_time = now;
  error = setpoint - angle; // Correct error calculation
  // if(start_integral = true){
  integral += error * time_change;
  // }
  // Integral windup prevention
  if (integral > max_integral)
    integral = max_integral;
  else if (integral < -max_integral)
    integral = -max_integral;

  derivative = (error - previous_error) / time_change;
  output = Kp * error + Ki * abs(integral) + Kd * derivative;
  previous_error = error;

  // Safety check: If the angle is within the safety limit, do not run the motor
  // if (abs(angle) < 60.0 && abs(angle) > -60.0 ) {
    // Convert PID output to stepper motor steps
    // if (abs(output) > DEADBAND) {
      // target_position = stepper.currentPosition() + (long)output; // Reverse the output to achieve opposite direction
        // if (angle > setpoint) {
        //           target_position  = 1 *target_position;
        //         }
        //         else if (angle < setpoint){
        //           target_position = -1 * target_position;
        //         }   
    stepper.setSpeed(-output);
    }

    stepper.runSpeed();
  // } 

  // Print values once per second
  if (now - last_print_time >= 10000) { // 1 second in microseconds
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
  
  // Run the stepper motor
  
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
  angle = ((pulse_count / pulses_per_revolution) * 360.0) + 186;
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
  angle = ((pulse_count / pulses_per_revolution) * 360.0) + 186;
}

// void resetPosition() {
//   pulse_count = 0; // Reset pulse count on zero position
//   angle = 0.0; // Reset the angle to zero
// }

void beginIntegralCalculation() {
  start_integral = true;
}


