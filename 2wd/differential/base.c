/*

This code defines the pins for controlling two motors of a 2WD differential drive mobile robot, and includes functions to move the robot forward, backward, and turn left or right. 

In the `moveForward()` and `moveBackward()` functions, both motors are set to move in the same direction to make the robot move forward or backward. In the `turnLeft()` and `turnRight()` functions, the motors are set to move in opposite directions to make the robot turn. The `turn_time` variable determines the amount of time the robot will turn for, after which the `stopMotors()` function is called to stop both motors.

Note that the specific pin numbers used in this code may need to be adjusted based on the hardware configuration of your robot. Additionally, you may need to adjust the speed values passed to the motor control functions to match the capabilities and requirements of your specific robot.

*/
// Define motor control pins
const int motor1_en_pin = 5;
const int motor1_in1_pin = 2;
const int motor1_in2_pin = 3;
const int motor2_en_pin = 6;
const int motor2_in1_pin = 4;
const int motor2_in2_pin = 5;
const int turn_time = 500;

void setup() {
  // Set motor control pins as outputs
  pinMode(motor1_en_pin, OUTPUT);
  pinMode(motor1_in1_pin, OUTPUT);
  pinMode(motor1_in2_pin, OUTPUT);
  pinMode(motor2_en_pin, OUTPUT);
  pinMode(motor2_in1_pin, OUTPUT);
  pinMode(motor2_in2_pin, OUTPUT);
}

void loop() {
  // Move the robot forward at full speed
  moveForward(255);
  delay(1000);
  
  // Turn the robot left
  turnLeft(255);
  delay(turn_time);
  stopMotors();
  
  // Move the robot backward at half speed
  moveBackward(127);
  delay(1000);
  
  // Turn the robot right
  turnRight(255);
  delay(turn_time);
  stopMotors();
}

void moveForward(int speed) {
  // Set both motors to move forward at the specified speed
  digitalWrite(motor1_in1_pin, HIGH);
  digitalWrite(motor1_in2_pin, LOW);
  digitalWrite(motor2_in1_pin, HIGH);
  digitalWrite(motor2_in2_pin, LOW);
  
  // Set the speed of both motors to the specified speed
  analogWrite(motor1_en_pin, speed);
  analogWrite(motor2_en_pin, speed);
}

void moveBackward(int speed) {
  // Set both motors to move backward at the specified speed
  digitalWrite(motor1_in1_pin, LOW);
  digitalWrite(motor1_in2_pin, HIGH);
  digitalWrite(motor2_in1_pin, LOW);
  digitalWrite(motor2_in2_pin, HIGH);
  
  // Set the speed of both motors to the specified speed
  analogWrite(motor1_en_pin, speed);
  analogWrite(motor2_en_pin, speed);
}

void turnLeft(int speed) {
  // Set motor 1 to move backward, and motor 2 to move forward
  digitalWrite(motor1_in1_pin, LOW);
  digitalWrite(motor1_in2_pin, HIGH);
  digitalWrite(motor2_in1_pin, HIGH);
  digitalWrite(motor2_in2_pin, LOW);
  
  // Set the speed of both motors to the specified speed
  analogWrite(motor1_en_pin, speed);
  analogWrite(motor2_en_pin, speed);
}

void turnRight(int speed) {
  // Set motor 1 to move forward, and motor 2 to move backward
  digitalWrite(motor1_in1_pin, HIGH);
  digitalWrite(motor1_in2_pin, LOW);
  digitalWrite(motor2_in1_pin, LOW);
  digitalWrite(motor2_in2_pin, HIGH);
  
  // Set the speed of both motors to the specified speed
  analogWrite(motor1_en_pin, speed);
  analogWrite(motor2_en_pin, speed);
}

void stopMotors() {
  // Stop both motors and set their speed to zero
  digitalWrite(motor1_in1_pin, LOW);
  digitalWrite(motor1_in2_pin, LOW);
  digitalWrite(motor2_in1_pin, LOW);
  digitalWrite(motor2_in2_pin, LOW);
  
  analogWrite(motor1_en_pin, 0);
analogWrite(motor2_en_pin, 0);
}

