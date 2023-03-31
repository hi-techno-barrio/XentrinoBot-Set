/*

Note that this code is just an example and may require modifications to suit your specific motor and encoder setup.


*/

// Motor control pins
const int motor1_dir_pin = 2;
const int motor1_speed_pin = 3;
const int motor2_dir_pin = 4;
const int motor2_speed_pin = 5;
const int motor3_dir_pin = 6;
const int motor3_speed_pin = 7;
const int motor4_dir_pin = 8;
const int motor4_speed_pin = 9;

// Encoder pins
const int motor1_enc_pinA = 10;
const int motor1_enc_pinB = 11;
const int motor2_enc_pinA = 12;
const int motor2_enc_pinB = 13;
const int motor3_enc_pinA = A0;
const int motor3_enc_pinB = A1;
const int motor4_enc_pinA = A2;
const int motor4_enc_pinB = A3;

// Encoder count array
volatile int encoder_counts[4] = {0, 0, 0, 0};

// PID control constants
float kp = 0.1;
float ki = 0.01;
float kd = 0.05;
int target_enc_count = 0;
int previous_error = 0;
float integral = 0;

void setup() {
  // Set motor control pins as outputs
  pinMode(motor1_dir_pin, OUTPUT);
  pinMode(motor1_speed_pin, OUTPUT);
  pinMode(motor2_dir_pin, OUTPUT);
  pinMode(motor2_speed_pin, OUTPUT);
  pinMode(motor3_dir_pin, OUTPUT);
  pinMode(motor3_speed_pin, OUTPUT);
  pinMode(motor4_dir_pin, OUTPUT);
  pinMode(motor4_speed_pin, OUTPUT);
  
  // Set encoder pins as inputs
  pinMode(motor1_enc_pinA, INPUT_PULLUP);
  pinMode(motor1_enc_pinB, INPUT_PULLUP);
  pinMode(motor2_enc_pinA, INPUT_PULLUP);
  pinMode(motor2_enc_pinB, INPUT_PULLUP);
  pinMode(motor3_enc_pinA, INPUT_PULLUP);
  pinMode(motor3_enc_pinB, INPUT_PULLUP);
  pinMode(motor4_enc_pinA, INPUT_PULLUP);
  pinMode(motor4_enc_pinB, INPUT_PULLUP);
  
  // Attach interrupt routines to encoder pins
  attachInterrupt(digitalPinToInterrupt(motor1_enc_pinA), motor1EncA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor1_enc_pinB), motor1EncB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor2_enc_pinA), motor2EncA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor2_enc_pinB), motor2EncB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor3_enc_pinA), motor3EncA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor3_enc_pinB), motor3EncB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor4_enc_pinA), motor4EncA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor4_enc_pinB), motor4EncB_ISR, CHANGE);
  
  // Set initial motor speeds to 0
  analogWrite(motor1_speed_pin, 0);
  analogWrite(motor2_speed_pin, 0);
  analogWrite(motor3_speed_pin, 0);
  analogWrite(motor4_speed_pin, 0);
}

void loop() {
  // Move forward at full speed for 1 second
  moveForward(255);
  delay(1000);
// Stop for 1 second
stopMotors();
delay(1000);

// Move backward at full speed for 1 second
moveBackward(255);
delay(1000);

// Stop for 1 second
stopMotors();
delay(1000);

// Turn left at full speed for 1 second
turnLeft(255);
delay(1000);

// Stop for 1 second
stopMotors();
delay(1000);

// Turn right at full speed for 1 second
turnRight(255);
delay(1000);

// Stop for 1 second
stopMotors();
delay(1000);
}

void moveForward(int speed) {
// Set motor directions to forward
digitalWrite(motor1_dir_pin, HIGH);
digitalWrite(motor2_dir_pin, HIGH);
digitalWrite(motor3_dir_pin, HIGH);
digitalWrite(motor4_dir_pin, HIGH);

// Set the speed of all motors
setMotorSpeeds(speed, speed, speed, speed);

// Set target encoder counts for all motors
target_enc_count = encoder_counts[0] + 100;
}

void moveBackward(int speed) {
// Set motor directions to backward
digitalWrite(motor1_dir_pin, LOW);
digitalWrite(motor2_dir_pin, LOW);
digitalWrite(motor3_dir_pin, LOW);
digitalWrite(motor4_dir_pin, LOW);

// Set the speed of all motors
setMotorSpeeds(speed, speed, speed, speed);

// Set target encoder counts for all motors
target_enc_count = encoder_counts[0] - 100;
}

void turnLeft(int speed) {
// Set left motors to move backward, and right motors to move forward
digitalWrite(motor1_dir_pin, LOW);
digitalWrite(motor2_dir_pin, LOW);
digitalWrite(motor3_dir_pin, HIGH);
digitalWrite(motor4_dir_pin, HIGH);

// Set the speed of all motors
setMotorSpeeds(speed, speed, speed, speed);

// Set target encoder counts for all motors
target_enc_count = encoder_counts[0] - 50;
}

void turnRight(int speed) {
// Set left motors to move forward, and right motors to move backward
digitalWrite(motor1_dir_pin, HIGH);
digitalWrite(motor2_dir_pin, HIGH);
digitalWrite(motor3_dir_pin, LOW);
digitalWrite(motor4_dir_pin, LOW);

// Set the speed of all motors
setMotorSpeeds(speed, speed, speed, speed);

// Set target encoder counts for all motors
target_enc_count = encoder_counts[0] + 50;
}

void stopMotors() {
// Set motor speeds to 0
analogWrite(motor1_speed_pin, 0);
analogWrite(motor2_speed_pin, 0);
analogWrite(motor3_speed_pin, 0);
analogWrite(motor4_speed_pin, 0);
}

void setMotorSpeeds(int speed1, int speed2, int speed3, int speed4) {
// Set the speed of all motors
analogWrite(motor1_speed_pin, speed1);
analogWrite(motor2_speed_pin, speed2);
analogWrite(motor3_speed_pin, speed3);
analogWrite(motor4_speed_pin, speed4);
}

// Interrupt routine for motor 1 encoder channel A
void motor1EncA_ISR() {
if (digitalRead(motor1_enc_pinA) == digitalRead(motor1_enc_pinB)) {
encoder_counts[0]++;
} else {
encoder_counts[0]--;
}
}

// Interrupt routine for motor 1 encoder channel B
void motor1EncB_ISR() {
if (digitalRead(motor1_enc_pinA) != digitalRead(motor1_enc_pinB)) {
encoder_counts[0]++;
} else {
encoder_counts[0]--;
}
}

// Interrupt routine for motor 2 encoder channel A
void motor2EncA_ISR() {
if (digitalRead(motor2_enc_pinA) == digitalRead(motor2_enc_pinB)) {
encoder_counts[1]++;
} else {
encoder_counts[1]--;
}
}

// Interrupt routine for motor 2 encoder channel B
void motor2EncB_ISR() {
if (digitalRead(motor2_enc_pinA) != digitalRead(motor2_enc_pinB)) {
encoder_counts[1]++;
} else {
encoder_counts[1]--;
}
}

// Interrupt routine for motor 3 encoder channel A
void motor3EncA_ISR() {
if (digitalRead(motor3_enc_pinA) == digitalRead(motor3_enc_pinB)) {
encoder_counts[2]++;
} else {
encoder_counts[2]--;
}
}

// Interrupt routine for motor 3 encoder channel B
void motor3EncB_ISR() {
if (digitalRead(motor3_enc_pinA) != digitalRead(motor3_enc_pinB)) {
encoder_counts[2]++;
} else {
encoder_counts[2]--;
}
}

// Interrupt routine for motor 4 encoder channel A
void motor4EncA_ISR() {
if (digitalRead(motor4_enc_pinA) == digitalRead(motor4_enc_pinB)) {
encoder_counts[3]++;
} else {
encoder_counts[3]--;
}
}

// Interrupt routine for motor 4 encoder channel B
void motor4EncB_ISR() {
if (digitalRead(motor4_enc_pinA) != digitalRead(motor4_enc_pinB)) {
encoder_counts[3]++;
} else {
encoder_counts[3]--;
}
}

// PID control loop
void PID() {
int motor_speeds[4] = {0, 0, 0, 0};
int error = target_enc_count - encoder_counts[0];
integral += error;
int derivative = error - previous_error;
motor_speeds[0] = kp * error + ki * integral + kd * derivative;
previous_error = error;

// Set the speed of all motors
setMotorSpeeds(motor_speeds[0], motor_speeds[0], motor_speeds[0], motor_speeds[0]);
}

// Main control loop
void controlLoop() {
while (true) {
// PID control loop
PID();// Wait for encoder counts to update
delay(10);
}
}

