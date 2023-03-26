/*

In this updated code, I've added functions for moving the robot backward, rotating left, and moving diagonally in various directions. 

To move the robot backward, you can use the `moveBackward()` function, which sets all the motor directions to move backward and the speed to the specified value.

To rotate the robot to the left, you can use the `rotateLeft()` function, which sets the directions of the motors to turn left and the speed to half of the specified value. 

To move the robot diagonally up and left, you can use the `moveUpLeft()` function, which sets motors 1 and 3 to move forward and motor 2 to stop. Similarly, to move diagonally down and left, you can use the `moveDownLeft()` function, which sets motor 3 to move backward and motors 1 and 2 to stop.

To move diagonally up and right, you can use the `moveUpRight()` function, which sets motor 3 to move forward and motors 1 and 2 to stop. And to move diagonally down and right, you can use the `moveDownRight()` function, which sets motor 2 to move backward and motors 1 and 3 to stop.

Finally, the `stopMotors()` function stops all motors and sets the speed of all motors to zero.

I hope this helps!


*/

// Define motor control pins
const int motor1_in1_pin = 2;
const int motor1_in2_pin = 3;
const int motor2_in1_pin = 4;
const int motor2_in2_pin = 5;
const int motor3_in1_pin = 6;
const int motor3_in2_pin = 7;

// Define motor speed pins
const int motor1_speed_pin = 9;
const int motor2_speed_pin = 10;
const int motor3_speed_pin = 11;

void setup() {
  // Set motor control pins as outputs
  pinMode(motor1_in1_pin, OUTPUT);
  pinMode(motor1_in2_pin, OUTPUT);
  pinMode(motor2_in1_pin, OUTPUT);
  pinMode(motor2_in2_pin, OUTPUT);
  pinMode(motor3_in1_pin, OUTPUT);
  pinMode(motor3_in2_pin, OUTPUT);
}

void loop() {
  // Move the robot forward at full speed
  moveForward(255);
  delay(1000);
  
  // Rotate the robot 120 degrees to the right
  rotateRight(120);
  delay(1000);
  
  // Move the robot backward at half speed
  moveBackward(127);
  delay(1000);
  
  // Rotate the robot 120 degrees to the left
  rotateLeft(120);
  delay(1000);
  
  // Move the robot diagonally up and left at full speed
  moveUpLeft(255);
  delay(1000);
  
  // Move the robot diagonally down and left at full speed
  moveDownLeft(255);
  delay(1000);
  
  // Move the robot diagonally up and right at full speed
  moveUpRight(255);
  delay(1000);
  
  // Move the robot diagonally down and right at full speed
  moveDownRight(255);
  delay(1000);
  
  // Stop all motors
  stopMotors();
  delay(1000);
}

void moveForward(int speed) {
  // Set all motors to move forward at the specified speed
  digitalWrite(motor1_in1_pin, HIGH);
  digitalWrite(motor1_in2_pin, LOW);
  digitalWrite(motor2_in1_pin, HIGH);
  digitalWrite(motor2_in2_pin, LOW);
  digitalWrite(motor3_in1_pin, HIGH);
  digitalWrite(motor3_in2_pin, LOW);
  
  // Set the speed of all motors
  analogWrite(motor1_speed_pin, speed);
  analogWrite(motor2_speed_pin, speed);
  analogWrite(motor3_speed_pin, speed);
}

void moveBackward(int speed) {
  // Set all motors to move backward at the specified speed
  digitalWrite(motor1_in1_pin, LOW);
  digitalWrite(motor1_in2_pin, HIGH);
  digitalWrite(motor2_in1_pin, LOW);
  digitalWrite(motor2_in2_pin, HIGH);
  digitalWrite(motor3_in1_pin, LOW);
  digitalWrite(motor3_in2_pin, HIGH);
  
  // Set the speed of all motors
  analogWrite(motor1_speed_pin, speed);
  analogWrite(motor2_speed_pin, speed);
  analogWrite(motor3_speed_pin, speed);
}

void rotateLeft(int angle) {
  // Set motors 1 and 2 to move forward at full speed, and motor 3 to move backwards at full speed
  digitalWrite(motor1_in1_pin, HIGH);
  digitalWrite(motor1_in2_pin, LOW);
  digitalWrite(motor2_in1
_pin, LOW);

// Set the speed of all motors to half speed
analogWrite(motor1_speed_pin, 127);
analogWrite(motor2_speed_pin, 127);
analogWrite(motor3_speed_pin, 127);

// Delay for the calculated time to rotate
delay(delay_time);

// Stop all motors
stopMotors();
}

void moveUpLeft(int speed) {
// Set motors 1 and 3 to move forward at the specified speed, and motor 2 to stop
digitalWrite(motor1_in1_pin, HIGH);
digitalWrite(motor1_in2_pin, LOW);
digitalWrite(motor2_in1_pin, LOW);
digitalWrite(motor2_in2_pin, LOW);
digitalWrite(motor3_in1_pin, HIGH);
digitalWrite(motor3_in2_pin, LOW);

// Set the speed of all motors
analogWrite(motor1_speed_pin, speed);
analogWrite(motor2_speed_pin, 0);
analogWrite(motor3_speed_pin, speed);
}

void moveDownLeft(int speed) {
// Set motors 1 and 2 to stop, and motor 3 to move backward at the specified speed
digitalWrite(motor1_in1_pin, LOW);
digitalWrite(motor1_in2_pin, LOW);
digitalWrite(motor2_in1_pin, LOW);
digitalWrite(motor2_in2_pin, LOW);
digitalWrite(motor3_in1_pin, LOW);
digitalWrite(motor3_in2_pin, HIGH);

// Set the speed of all motors
analogWrite(motor1_speed_pin, 0);
analogWrite(motor2_speed_pin, 0);
analogWrite(motor3_speed_pin, speed);
}

void moveUpRight(int speed) {
// Set motors 1 and 2 to stop, and motor 3 to move forward at the specified speed
digitalWrite(motor1_in1_pin, LOW);
digitalWrite(motor1_in2_pin, LOW);
digitalWrite(motor2_in1_pin, LOW);
digitalWrite(motor2_in2_pin, LOW);
digitalWrite(motor3_in1_pin, HIGH);
digitalWrite(motor3_in2_pin, LOW);

// Set the speed of all motors
analogWrite(motor1_speed_pin, 0);
analogWrite(motor2_speed_pin, 0);
analogWrite(motor3_speed_pin, speed);
}

void moveDownRight(int speed) {
// Set motors 1 and 3 to stop, and motor 2 to move backward at the specified speed
digitalWrite(motor1_in1_pin, LOW);
digitalWrite(motor1_in2_pin, LOW);
digitalWrite(motor2_in1_pin, LOW);
digitalWrite(motor2_in2_pin, HIGH);
digitalWrite(motor3_in1_pin, LOW);
digitalWrite(motor3_in2_pin, HIGH);

// Set the speed of all motors
analogWrite(motor1_speed_pin, 0);
analogWrite(motor2_speed_pin, speed);
analogWrite(motor3_speed_pin, 0);
}

void stopMotors() {
// Stop all motors
digitalWrite(motor1_in1_pin, LOW);
digitalWrite(motor1_in2_pin, LOW);
digitalWrite(motor2_in1_pin, LOW);
digitalWrite(motor2_in2_pin, LOW);
digitalWrite(motor3_in1_pin, LOW);
digitalWrite(motor3_in2_pin, LOW);

// Set the speed of all motors to zero
analogWrite(motor1_speed_pin, 0);
analogWrite(motor2_speed_pin, 0);
analogWrite(motor3_speed_pin, 0);
}
