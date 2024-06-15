#include <AccelStepper.h>

// Define stepper motor connections
#define STEPPER_PULSE_PIN 9
#define STEPPER_DIR_PIN 8
#define SW_Pin 18

// Define rotary encoder pins
#define ENC_A 2
#define ENC_B 3

// PID parameters
float Kp = 10.0;
float Ki = 0.1;
float Kd = 0.01;
float max_integral = 100; // Limit for integral term to prevent windup

// Deadband value (adjust as needed)
#define DEADBAND 1

// Create an instance of the AccelStepper library
AccelStepper stepper(AccelStepper::DRIVER, STEPPER_PULSE_PIN, STEPPER_DIR_PIN);

// Global variables
volatile int counter = 0;
float integral = 0.0;
float previous_error = 0.0;
unsigned long last_time = 0;
unsigned long last_print_time = 0; 
volatile long target_position = 0;
void setup() {
    // maximum speed and acceleration for the stepper motor
    stepper.setMaxSpeed(2000); 
    stepper.setAcceleration(87500); 

    
    Serial.begin(115200);

    
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    pinMode(SW_Pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_A), readEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B), readEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(SW_Pin), reset_counter, LOW);

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
        float error = -counter; // reference angle is 0
        integral += error * time_change;

        // integral windup prevention
        if (integral > max_integral)
            integral = max_integral;
        else if (integral < -max_integral)
            integral = -max_integral;

        float derivative = (error - previous_error) / time_change;
        float output = Kp * error + Ki * integral + Kd * derivative;
        previous_error = error;

        // Convert PID output to stepper motor steps
        if (output != 0.0) {
            long target_position = stepper.currentPosition() + (long)output;
            if (abs(target_position - stepper.currentPosition()) > DEADBAND) {
                if (angle > set_point) {
                  target_position  = target_position;
                }
                else if (angle < set_point){
                  target_position = -1 * target_position;
                }   
                  stepper.moveTo(target_position);
                         
                }
        }
        //    if (abs(output) > DEADBAND) {
        //     target_position = stepper.currentPosition() + (long)output;
        //     stepper.moveTo(target_position);
        // }


        // Print values once per second
        if (now - last_print_time >= 1000) {
            last_print_time = now;
            Serial.print("Counter: ");
            Serial.println(counter);
            Serial.print("Error: ");
            Serial.println(error);
            Serial.print("Integral: ");
            Serial.println(integral);
            Serial.print("Derivative: ");
            Serial.println(derivative);
            Serial.print("Output: ");
            Serial.println(output);
            Serial.print("Target Position:");
            Serial.println(target_position);
            Serial.print("Current Position: ");
            Serial.println(stepper.currentPosition());
            
        }
    }

    // Run the stepper motor
    stepper.run();
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
        counter += 1;
        encval = 0;
    } else if (encval < -1) {
        counter -= 1;
        encval = 0;
    }
}

void reset_counter() {
    counter = 0;
    integral = 0;
    previous_error = 0;
    stepper.setCurrentPosition(0);
    stepper.runToPosition();
}