// Define motor control pins
const int motor1_in1_pin = 2;
const int motor1_in2_pin = 3;
const int motor2_in1_pin = 4;
const int motor2_in2_pin = 5;
const int motor3_in1_pin = 6;
const int motor3_in2_pin = 7;

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
  
  // Move the robot forward at full speed
  moveForward(255);
  delay(1000);
  
  // Rotate the robot 120 degrees to the right
  rotateRight(120);
  delay(1000);
  
  // Move the robot forward at full speed
  moveForward(255);
  delay(1000);
  
  // Rotate the robot 120 degrees to the right
  rotateRight(120);
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

void rotateRight(int angle) {
  // Set motors 1 and 3 to move forward at full speed, and motor 2 to move backwards at full speed
  digitalWrite(motor1_in1_pin, HIGH);
  digitalWrite(motor1_in2_pin, LOW);
  digitalWrite(motor2_in1_pin, LOW);
  digitalWrite(motor2_in2_pin, HIGH);
  digitalWrite(motor3_in1_pin, HIGH);
  digitalWrite(motor3_in2_pin, LOW);
  
  // Calculate the delay needed to rotate the specified angle
  int delay_time = angle * 10;
  
  // Set the speed of all motors to half speed
  analogWrite(motor1_speed_pin, 127);
  analogWrite(motor2_speed_pin, 127);
  analogWrite(motor3_speed_pin, 127);
  
  // Delay for the calculated time to rotate
  delay(delay_time);
  
  // Stop all motors
  digitalWrite(motor1_in1_pin, LOW);
  digitalWrite(motor1_in2_pin, LOW);
  digitalWrite(motor2_in1_pin, LOW);
  digitalWrite(motor2_in2_pin, LOW);
  digitalWrite(motor3_in1_pin, LOW);
  digitalWrite(motor3_in2_pin, LOW);
}
