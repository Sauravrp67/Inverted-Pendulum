#include <AccelStepper.h>

// Define stepper motor connections:
#define STEPPER_PULSE_PIN 9
#define STEPPER_DIR_PIN 8
#define PIN1 18
#define PIN2 19
// Create an instance of the AccelStepper library
AccelStepper stepper(AccelStepper::DRIVER, STEPPER_PULSE_PIN, STEPPER_DIR_PIN);
static long position = 0;

void setup() {
  // Set the maximum speed and acceleration for the stepper motor
  stepper.setMaxSpeed(2000); // Steps per second
  stepper.setAcceleration(2000); // Steps per second^2
  // pinMode(STEPPER_PULSE_PIN,OUTPUT);
  // pinMode(STEPPER_DIR_PIN,OUTPUT);
  // pinMode(PIN1,INPUT_PULLUP);
  // pinMode(PIN2,INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(PIN1),change_position1,RISING);
  // attachInterrupt(digitalPinToInterrupt(PIN2),change_position2,RISING);

  // stepper.move(200);
  // stepper.moveTo(1000);
  // stepper.setSpeed(100);
  // Initialize the serial communication
  // stepper.moveTo(400);
  // stepper.setSpeed(1000);
  Serial.begin(115200);
}

void loop() {
  // Set the target position to move to
  stepper.moveTo(400);
  stepper.setSpeed(100);
  
  // stepper.setSpeed(200);
  stepper.runToPosition();

  stepper.moveTo(-400);
  stepper.setSpeed(100);
  stepper.runToPosition();
  // stepper.moveTo(0);
  // stepper.runSpeedToPosition();
  // stepper.moveTo(-200);
  // stepper.runSpeedToPosition();
  // stepper.runSpeedToPosition();
  // Serial.println(position);
  // stepper.run();
  
  // Wait for a second
  

  // Serial.println("Reached home position");
}


// void change_position1() {
//     Serial.println('500 position Initiated');
//     // stepper.moveTo(100);
//     stepper.move(200);
//     position = 200;
      

// }

// void change_position2() {
//     Serial.println('-500 position Initiated');
//     // stepper.moveTo(-100);
//     stepper.move(-200);
//     position = -200;
    

// }