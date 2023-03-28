/* 

This code defines the pins for controlling two motors and a caster wheel, and includes functions to move the robot forward, backward, and turn left or right. 
In the `moveForward()` and `moveBackward()` functions, the caster wheel is set to move in the direction of motion to provide stability and balance to the robot. 
In the `turnLeft()` and `turnRight()` functions, the caster wheel is set to turn in the opposite direction to help the robot turn smoothly. 

Note that the specific pin numbers used in this code may need to be adjusted based on the hardware configuration of your robot.
*/


// Define motor control pins
const int motor1_in1_pin = 2;
const int motor1_in2_pin = 3;
const int motor1_speed_pin = 5;
const int motor2_in1_pin = 4;
const int motor2_in2_pin = 5;
const int motor2_speed_pin = 6;
const int caster_pin = 7;

void setup() {
  // Set motor control pins as outputs
  pinMode(motor1_in1_pin, OUTPUT);
  pinMode(motor1_in2_pin, OUTPUT);
  pinMode(motor1_speed_pin, OUTPUT);
  pinMode(motor2_in1_pin, OUTPUT);
  pinMode(motor2_in2_pin, OUTPUT);
  pinMode(motor2_speed_pin, OUTPUT);
  pinMode(caster_pin, OUTPUT);
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
  // Set motor 1 to move forward, and motor 2 to move backward
  digitalWrite(motor1_in1_pin, HIGH);
  digitalWrite(motor1_in2_pin, LOW);
  digitalWrite(motor2_in1_pin, LOW);
  digitalWrite(motor2_in2_pin, HIGH);
  
  // Set the speed of both motors to the specified speed
  analogWrite(motor1_speed_pin, speed);
  analogWrite(motor2_speed_pin, speed);
  
  // Set the caster wheel to move in the direction of motion
  digitalWrite(caster_pin, LOW);
}

void moveBackward(int speed) {
  // Set motor 1 to move backward, and motor 2 to move forward
  digitalWrite(motor1_in1_pin, LOW);
  digitalWrite(motor1_in2_pin, HIGH);
  digitalWrite(motor2_in1_pin, HIGH);
  digitalWrite(motor2_in2_pin, LOW);
  
  // Set the speed of both motors to the specified speed
  analogWrite(motor1_speed_pin, speed);
  analogWrite(motor2_speed_pin, speed);
  
  // Set the caster wheel to move in the direction of motion
  digitalWrite(caster_pin, HIGH);
}

void turnLeft(int speed) {
  // Set motor 1 and the caster wheel to move forward, and motor 2 to stop
  digitalWrite(motor1_in1_pin, HIGH);
  digitalWrite(motor1_in2_pin, LOW);
  digitalWrite(motor2_in1_pin, LOW);
  digitalWrite(motor2_in2_pin, LOW);
  
  // Set the speed of motor 1 and the caster wheel to the specified speed, and motor 2 to zero
  analogWrite(motor1_speed_pin, speed);
  analogWrite(motor2_speed_pin, 0);
  
  // Set the caster wheel to turn left
  digitalWrite(caster_pin, HIGH);
}

void turnRight(int speed) {
  // Set motor 2 and the caster wheel to move forward, and motor 1 to stop
  digitalWrite(motor1_in1_pin, LOW);
  digitalWrite(motor1_in2_pin, LOW);
  digitalWrite(motor2_in1_pin, HIGH);
  digitalWrite(motor2_in2_pin, LOW);
  
  // Set the speed of motor 2 and the caster wheel to
the specified speed, and motor 1 to zero
analogWrite(motor1_speed_pin, 0);
analogWrite(motor2_speed_pin, speed);

// Set the caster wheel to turn right
digitalWrite(caster_pin, LOW);
}
