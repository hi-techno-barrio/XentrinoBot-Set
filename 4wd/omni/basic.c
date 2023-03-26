// Define motor control pins
const int motor1_in1_pin = 2;
const int motor1_in2_pin = 3;
const int motor2_in1_pin = 4;
const int motor2_in2_pin = 5;
const int motor3_in1_pin = 6;
const int motor3_in2_pin = 7;
const int motor4_in1_pin = 8;
const int motor4_in2_pin = 9;

// Define motor speed pins
const int motor1_speed_pin = 10;
const int motor2_speed_pin = 11;
const int motor3_speed_pin = 12;
const int motor4_speed_pin = 13;

void setup() {
  // Set motor control pins as outputs
  pinMode(motor1_in1_pin, OUTPUT);
  pinMode(motor1_in2_pin, OUTPUT);
  pinMode(motor2_in1_pin, OUTPUT);
  pinMode(motor2_in2_pin, OUTPUT);
  pinMode(motor3_in1_pin, OUTPUT);
  pinMode(motor3_in2_pin, OUTPUT);
  pinMode(motor4_in1_pin, OUTPUT);
  pinMode(motor4_in2_pin, OUTPUT);
}

void loop() {
  // Move the robot forward at full speed
  moveForward(255);
  delay(1000);
  
  // Rotate the robot 90 degrees to the right
  rotateRight(90);
  delay(1000);
  
  // Move the robot backward at half speed
  moveBackward(127);
  delay(1000);
  
  // Rotate the robot 90 degrees to the left
  rotateLeft(90);
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
  digitalWrite(motor4_in1_pin, HIGH);
  digitalWrite(motor4_in2_pin, LOW);
  
  // Set the speed of all motors
  analogWrite(motor1_speed_pin, speed);
  analogWrite(motor2_speed_pin, speed);
  analogWrite(motor3_speed_pin, speed);
  analogWrite(motor4_speed_pin, speed);
}

void moveBackward(int speed) {
  // Set all motors to move backward at the specified speed
  digitalWrite(motor1_in1_pin, LOW);
  digitalWrite(motor1_in2_pin, HIGH);
  digitalWrite(motor2_in1_pin, LOW);
  digitalWrite(motor2_in2_pin, HIGH);
  digitalWrite(motor3_in1_pin, LOW);
  digitalWrite(motor3_in2_pin, HIGH);
  digitalWrite(motor4_in1_pin, LOW);
  digitalWrite(mmotor4_in2_pin, HIGH);

// Set the speed of all motors
analogWrite(motor1_speed_pin, speed);
analogWrite(motor2_speed_pin, speed);
analogWrite(motor3_speed_pin, speed);
analogWrite(motor4_speed_pin, speed);
}

void rotateRight(int angle) {
// Set motors 1 and 3 to move forward, and motors 2 and 4 to move backward
digitalWrite(motor1_in1_pin, HIGH);
digitalWrite(motor1_in2_pin, LOW);
digitalWrite(motor2_in1_pin, LOW);
digitalWrite(motor2_in2_pin, HIGH);
digitalWrite(motor3_in1_pin, HIGH);
digitalWrite(motor3_in2_pin, LOW);
digitalWrite(motor4_in1_pin, LOW);
digitalWrite(motor4_in2_pin, HIGH);

// Calculate the delay needed to rotate the specified angle
int delay_time = angle * 10;

// Set the speed of motors 1 and 3 to full speed, and motors 2 and 4 to half speed
analogWrite(motor1_speed_pin, 255);
analogWrite(motor2_speed_pin, 127);
analogWrite(motor3_speed_pin, 255);
analogWrite(motor4_speed_pin, 127);

// Delay for the calculated time to rotate
delay(delay_time);

// Stop all motors
stopMotors();
}

void rotateLeft(int angle) {
// Set motors 1 and 3 to move backward, and motors 2 and 4 to move forward
digitalWrite(motor1_in1_pin, LOW);
digitalWrite(motor1_in2_pin, HIGH);
digitalWrite(motor2_in1_pin, HIGH);
digitalWrite(motor2_in2_pin, LOW);
digitalWrite(motor3_in1_pin, LOW);
digitalWrite(motor3_in2_pin, HIGH);
digitalWrite(motor4_in1_pin, HIGH);
digitalWrite(motor4_in2_pin, LOW);

// Calculate the delay needed to rotate the specified angle
int delay_time = angle * 10;

// Set the speed of motors 1 and 3 to half speed, and motors 2 and 4 to full speed
analogWrite(motor1_speed_pin, 127);
analogWrite(motor2_speed_pin, 255);
analogWrite(motor3_speed_pin, 127);
analogWrite(motor4_speed_pin, 255);

// Delay for the calculated time to rotate
delay(delay_time);

// Stop all motors
stopMotors();
}

void moveUpLeft(int speed) {
// Set motors 1 and 4 to move forward, and motors 2 and 3 to stop
digitalWrite(motor1_in1_pin, HIGH);
digitalWrite(motor1_in2_pin, LOW);
digitalWrite(motor2_in1_pin, LOW);
digitalWrite(motor2_in2_pin, LOW);
digitalWrite(motor3_in1_pin, LOW);
digitalWrite(motor3_in2_pin, LOW);
digitalWrite(motor4_in1_pin, HIGH);
digitalWrite(motor4_in2_pin, LOW);

// Set the speed of motors 1 and 4 to the specified speed, and motors 2 and 3 to zero
analogWrite(motor1_speed_pin, speed);
analogWrite(motor2_speed_pin, 0);
analogWrite(motor3_speed_pin, 0);
analogWrite(motor4_speed_pin, speed);
}

void moveDownLeft(int speed) {
// Set motors 2 and3 to move backward, and motors 1 and 4 to stop
digitalWrite(motor1_in1_pin, LOW);
digitalWrite(motor1_in2_pin, LOW);
digitalWrite(motor2_in1_pin, LOW);
digitalWrite(motor2_in2_pin, HIGH);
digitalWrite(motor3_in1_pin, LOW);
digitalWrite(motor3_in2_pin, HIGH);
digitalWrite(motor4_in1_pin, LOW);
digitalWrite(motor4_in2_pin, LOW);

// Set the speed of motors 2 and 3 to the specified speed, and motors 1 and 4 to zero
analogWrite(motor1_speed_pin, 0);
analogWrite(motor2_speed_pin, speed);
analogWrite(motor3_speed_pin, speed);
analogWrite(motor4_speed_pin, 0);
}

void moveUpRight(int speed) {
// Set motors 2 and 3 to move forward, and motors 1 and 4 to stop
digitalWrite(motor1_in1_pin, LOW);
digitalWrite(motor1_in2_pin, HIGH);
digitalWrite(motor2_in1_pin, HIGH);
digitalWrite(motor2_in2_pin, LOW);
digitalWrite(motor3_in1_pin, HIGH);
digitalWrite(motor3_in2_pin, LOW);
digitalWrite(motor4_in1_pin, LOW);
digitalWrite(motor4_in2_pin, LOW);

// Set the speed of motors 2 and 3 to the specified speed, and motors 1 and 4 to zero
analogWrite(motor1_speed_pin, 0);
analogWrite(motor2_speed_pin, speed);
analogWrite(motor3_speed_pin, speed);
analogWrite(motor4_speed_pin, 0);
}

void moveDownRight(int speed) {
// Set motors 1 and 4 to move backward, and motors 2 and 3 to stop
digitalWrite(motor1_in1_pin, LOW);
digitalWrite(motor1_in2_pin, HIGH);
digitalWrite(motor2_in1_pin, LOW);
digitalWrite(motor2_in2_pin, LOW);
digitalWrite(motor3_in1_pin, LOW);
digitalWrite(motor3_in2_pin, LOW);
digitalWrite(motor4_in1_pin, LOW);
digitalWrite(motor4_in2_pin, HIGH);

// Set the speed of motors 1 and 4 to the specified speed, and motors 2 and 3 to zero
analogWrite(motor1_speed_pin, speed);
analogWrite(motor2_speed_pin, 0);
analogWrite(motor3_speed_pin, 0);
analogWrite(motor4_speed_pin, speed);
}

void stopMotors() {
// Stop all motors and set the speed of all motors to zero
digitalWrite(motor1_in1_pin, LOW);
digitalWrite(motor1_in2_pin, LOW);
digitalWrite(motor2_in1_pin, LOW);
digitalWrite(motor2_in2_pin, LOW);
digitalWrite(motor3_in1_pin, LOW);
digitalWrite(motor3_in2_pin, LOW);
digitalWrite(motor4_in1_pin, LOW);
digitalWrite(motor4_in2_pin, LOW);

analogWrite(motor1_speed_pin, 0);
analogWrite(motor2_speed_pin, 0);
analogWrite(motor3_speed_pin, 0);
analogWrite(motor4_speed_pin, 0);
}
