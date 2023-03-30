
/*

This code defines the pins for controlling four motors of a skid-steer 4WD mobile robot, and includes functions to move the robot forward, backward, and turn left or right. 

In the `moveForward()` and `moveBackward()` functions, two motors on one side of the robot are set to move in the same direction, while the other two motors on the other side are set to move in the opposite direction. In the `turnLeft()` and `turnRight()` functions, two motors on one side of the robot are set to move backward, while the other two motors on the other side are set to move forward. 

Note that the specific pin numbers used in this code may need to be adjusted based on the hardware configuration of your robot. Additionally, you may need to adjust the speed values passed to the motor control functions to match the capabilities and requirements of your specific robot.


*/


// Define motor control pins
const int motor1_en_pin = 5;
const int motor1_in1_pin = 2;
const int motor1_in2_pin = 3;
const int motor2_en_pin = 6;
const int motor2_in1_pin = 4;
const int motor2_in2_pin = 5;
const int motor3_en_pin = 9;
const int motor3_in1_pin = 7;
const int motor3_in2_pin = 8;
const int motor4_en_pin = 10;
const int motor4_in1_pin = 11;
const int motor4_in2_pin = 12;

void setup() {
  // Set motor control pins as outputs
  pinMode(motor1_en_pin, OUTPUT);
  pinMode(motor1_in1_pin, OUTPUT);
  pinMode(motor1_in2_pin, OUTPUT);
  pinMode(motor2_en_pin, OUTPUT);
  pinMode(motor2_in1_pin, OUTPUT);
  pinMode(motor2_in2_pin, OUTPUT);
  pinMode(motor3_en_pin, OUTPUT);
  pinMode(motor3_in1_pin, OUTPUT);
  pinMode(motor3_in2_pin, OUTPUT);
  pinMode(motor4_en_pin, OUTPUT);
  pinMode(motor4_in1_pin, OUTPUT);
  pinMode(motor4_in2_pin, OUTPUT);
}

void loop() {
  // Move the robot forward at full speed
  moveForward(255);
  delay(1000);
  
  // Turn the robot left
  turnLeft(255);
  delay(1000);
  
  // Move the robot backward at half speed
  moveBackward(127);
  delay(1000);
  
  // Turn the robot right
  turnRight(255);
  delay(1000);
}

void moveForward(int speed) {
  // Set motors 1 and 2 to move forward, and motors 3 and 4 to move forward
  digitalWrite(motor1_in1_pin, HIGH);
  digitalWrite(motor1_in2_pin, LOW);
  digitalWrite(motor2_in1_pin, HIGH);
  digitalWrite(motor2_in2_pin, LOW);
  digitalWrite(motor3_in1_pin, HIGH);
  digitalWrite(motor3_in2_pin, LOW);
  digitalWrite(motor4_in1_pin, HIGH);
  digitalWrite(motor4_in2_pin, LOW);
  
  // Set the speed of all motors to the specified speed
  analogWrite(motor1_en_pin, speed);
  analogWrite(motor2_en_pin, speed);
  analogWrite(motor3_en_pin, speed);
  analogWrite(motor4_en_pin, speed);
}

void moveBackward(int speed) {
  // Set motors 1 and 2 to move backward, and motors 3 and 4 to move backward
  digitalWrite(motor1_in1_pin, LOW);
  digitalWrite(motor1_in2_pin, HIGH);
  digitalWrite(motor2_in1_pin, LOW);
  digitalWrite(motor2_in2_pin, HIGH);
  digitalWrite(motor3_in1_pin, LOW);
  digitalWrite(motor3_in2_pin, HIGH);
  digitalWrite(motor4_in1_pin, LOW);
  digitalWrite(motor4_in2_pin, HIGH);
  
  // Set the speed of all motors to the specified speed
  analogWrite(motor1_en_pin, speed);
  analogWrite(motor2_en_pin, speed);
  analogWrite(motor3_en_pin, speed);
  analogWrite(motor4_en_pin, speed);
}

void turnLeft(int speed) {
  // Set motors 1 and 2 to
move backward, and motors 3 and 4 to move forward
digitalWrite(motor1_in1_pin, LOW);
digitalWrite(motor1_in2_pin, HIGH);
digitalWrite(motor2_in1_pin, LOW);
digitalWrite(motor2_in2_pin, HIGH);
digitalWrite(motor3_in1_pin, HIGH);
digitalWrite(motor3_in2_pin, LOW);
digitalWrite(motor4_in1_pin, HIGH);
digitalWrite(motor4_in2_pin, LOW);

// Set the speed of all motors to the specified speed
analogWrite(motor1_en_pin, speed);
analogWrite(motor2_en_pin, speed);
analogWrite(motor3_en_pin, speed);
analogWrite(motor4_en_pin, speed);
}

void turnRight(int speed) {
// Set motors 1 and 2 to move forward, and motors 3 and 4 to move backward
digitalWrite(motor1_in1_pin, HIGH);
digitalWrite(motor1_in2_pin, LOW);
digitalWrite(motor2_in1_pin, HIGH);
digitalWrite(motor2_in2_pin, LOW);
digitalWrite(motor3_in1_pin, LOW);
digitalWrite(motor3_in2_pin, HIGH);
digitalWrite(motor4_in1_pin, LOW);
digitalWrite(motor4_in2_pin, HIGH);

// Set the speed of all motors to the specified speed
analogWrite(motor1_en_pin, speed);
analogWrite(motor2_en_pin, speed);
analogWrite(motor3_en_pin, speed);
analogWrite(motor4_en_pin, speed);
}

void stopMotors() {
// Stop all motors and set their speed to zero
digitalWrite(motor1_in1_pin, LOW);
digitalWrite(motor1_in2_pin, LOW);
digitalWrite(motor2_in1_pin, LOW);
digitalWrite(motor2_in2_pin, LOW);
digitalWrite(motor3_in1_pin, LOW);
digitalWrite(motor3_in2_pin, LOW);
digitalWrite(motor4_in1_pin, LOW);
digitalWrite(motor4_in2_pin, LOW);

analogWrite(motor1_en_pin, 0);
analogWrite(motor2_en_pin, 0);
analogWrite(motor3_en_pin, 0);
analogWrite(motor4_en_pin, 0);
}

