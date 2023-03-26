/*
This code initializes the motor and encoder pins, reads the encoder values, calculates the mecanum motor commands based on the desired movement, and converts those 
commands to individual motor speeds using the `setMotorSpeed()` function. The encoder values are printed to the serial monitor for debugging purposes.

Note that the mecanum control code is not included in this example - you'll need to write that yourself based on your specific requirements. The `vx`, `vy`, 
and `vw` variables represent the desired forward/backward velocity, left/right velocity, and rotational velocity, respectively. You'll need to use those values
to calculate the individual motor speeds using a mecanum kinematics model.
*/


#include <Encoder.h>
#define MOTOR1_PIN1 2 // Motor 1 pin 1
#define MOTOR1_PIN2 3 // Motor 1 pin 2
#define MOTOR2_PIN1 4 // Motor 2 pin 1
#define MOTOR2_PIN2 5 // Motor 2 pin 2
#define MOTOR3_PIN1 6 // Motor 3 pin 1
#define MOTOR3_PIN2 7 // Motor 3 pin 2
#define MOTOR4_PIN1 8 // Motor 4 pin 1
#define MOTOR4_PIN2 9 // Motor 4 pin 2

// Define the motor encoder pins
#define ENCODER1A_PIN 10 // Encoder 1A pin
#define ENCODER1B_PIN 11 // Encoder 1B pin
#define ENCODER2A_PIN 12 // Encoder 2A pin
#define ENCODER2B_PIN 13 // Encoder 2B pin
#define ENCODER3A_PIN 14 // Encoder 3A pin
#define ENCODER3B_PIN 15 // Encoder 3B pin
#define ENCODER4A_PIN 16 // Encoder 4A pin
#define ENCODER4B_PIN 17 // Encoder 4B pin

// Define the encoder objects
Encoder encoder1(ENCODER1A_PIN, ENCODER1B_PIN);
Encoder encoder2(ENCODER2A_PIN, ENCODER2B_PIN);
Encoder encoder3(ENCODER3A_PIN, ENCODER3B_PIN);
Encoder encoder4(ENCODER4A_PIN, ENCODER4B_PIN);

void setup() {
  // Set the motor pins as outputs
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);
  pinMode(MOTOR3_PIN1, OUTPUT);
  pinMode(MOTOR3_PIN2, OUTPUT);
  pinMode(MOTOR4_PIN1, OUTPUT);
  pinMode(MOTOR4_PIN2, OUTPUT);
  
  // Set the encoder pins as inputs
  pinMode(ENCODER1A_PIN, INPUT);
  pinMode(ENCODER1B_PIN, INPUT);
  pinMode(ENCODER2A_PIN, INPUT);
  pinMode(ENCODER2B_PIN, INPUT);
  pinMode(ENCODER3A_PIN, INPUT);
  pinMode(ENCODER3B_PIN, INPUT);
  pinMode(ENCODER4A_PIN, INPUT);
  pinMode(ENCODER4B_PIN, INPUT);
  
  // Set up the serial communication
  Serial.begin(9600);
}

void loop() {
  // Read the encoder values
  long encoder1Value = encoder1.read();
  long encoder2Value = encoder2.read();
  long encoder3Value = encoder3.read();
  long encoder4Value = encoder4.read();
  
  // Calculate the motor commands for mecanum movement
  int vx = 0; // Forward/backward velocity
  int vy = 0; // Left/right velocity
  int vw = 0; // Rotational velocity
  
  // TODO: Add your mecanum control code here
  
  // Convert the mecanum commands to individual motor commands
  int motor1Speed = vx + vy + vw;
  int motor2Speed = vx - vy - vw;
  int motor3Speed = vx - vy + vw;
  int motor4Speed = vx + vy - vw;
  
  // Set the motor speeds

  
  setMotorSpeed(MOTOR1_PIN1, MOTOR1_PIN2, motor1Speed);
setMotorSpeed(MOTOR2_PIN1, MOTOR2_PIN2, motor2Speed);
setMotorSpeed(MOTOR3_PIN1, MOTOR3_PIN2, motor3Speed);
setMotorSpeed(MOTOR4_PIN1, MOTOR4_PIN2, motor4Speed);

// Print the encoder values to the serial monitor for debugging
Serial.print("Encoder 1: ");
Serial.print(encoder1Value);
Serial.print(", Encoder 2: ");
Serial.print(encoder2Value);
Serial.print(", Encoder 3: ");
Serial.print(encoder3Value);
Serial.print(", Encoder 4: ");
Serial.println(encoder4Value);

delay(10); // Add a small delay to avoid flooding the serial monitor
}

// Function to set the speed of a motor based on the given pin values
void setMotorSpeed(int pin1, int pin2, int speed) {
if (speed > 0) {
analogWrite(pin1, speed);
analogWrite(pin2, 0);
}
else if (speed < 0) {
analogWrite(pin1, 0);
analogWrite(pin2, -speed);
}
else {
analogWrite(pin1, 0);
analogWrite(pin2, 0);
}
}
