/*
This program includes the Kalman filter code from the Kalman library. The `setup()` function initializes the Kalman filters with appropriate parameters. 
In the `loop()` function, the gyro and accelerometer readings from the MPU6050 are read, and the Kalman filters are updated with these readings. 
The angle and angular velocity estimates from the Kalman filters are then used to calculate the PID control output and apply it to the motors. 
The program also includes encoder functions to detect motor RPM, and user input functions to set the motor speed and direction based on user input 
from the serial monitor.

Note that the Kalman filter parameters and PID control constants may need to be adjusted to suit your specific hardware and requirements. Also, this 
program assumes that the left motor is connected to pins 2, 3, and 4, and the right motor is connected to pins 5, 6, and 7, as specified
in the program.
*/


#include <Wire.h>
#include <MPU6050_tockn.h>
#include <Motor.h>
#include <Kalman.h>

// Motor pin definitions
const int left_motor_pin1 = 2;
const int left_motor_pin2 = 3;
const int left_motor_pwm_pin = 4;
const int right_motor_pin1 = 5;
const int right_motor_pin2 = 6;
const int right_motor_pwm_pin = 7;

// Encoder pin definitions
const int left_encoder_pin = 18;
const int right_encoder_pin = 19;

// Encoder count variables
volatile long left_encoder_count = 0;
volatile long right_encoder_count = 0;

// PID control variables
double setpoint = 0;
double kp = 30;
double ki = 5;
double kd = 5;
double last_error = 0;
double integral = 0;

// Motor objects
Motor left_motor(left_motor_pin1, left_motor_pin2, left_motor_pwm_pin);
Motor right_motor(right_motor_pin1, right_motor_pin2, right_motor_pwm_pin);

// MPU6050 object
MPU6050 mpu6050(Wire);

// Kalman filter objects
Kalman kalman_angle;
Kalman kalman_rate;

void setup() {
  // Initialize I2C communication and MPU6050
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  
  // Initialize motor speeds and directions
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);
  left_motor.forward();
  right_motor.forward();

  // Initialize encoder interrupts
  pinMode(left_encoder_pin, INPUT_PULLUP);
  pinMode(right_encoder_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(left_encoder_pin), left_encoder_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(right_encoder_pin), right_encoder_isr, RISING);
  
  // Initialize Kalman filters
  kalman_angle.setAngle(0);
  kalman_rate.setAngle(0);
  kalman_rate.setQangle(0.01);
  kalman_rate.setQbias(0.01);
  kalman_rate.setRmeasure(0.1);
}

void loop() {
  // Read gyro and accelerometer readings from MPU6050
  mpu6050.update();
  double gx = mpu6050.getGyroX();
  double ay = mpu6050.getAccY();
  
  // Update Kalman filters with gyro and accelerometer readings
  double dt = 0.02; // Sample time in seconds
  double angle = kalman_angle.getAngle(gx, ay, dt);
  double rate = kalman_rate.getRate(gx, ay, dt);
  
  // Read angle from Kalman filter and calculate PID control output
  double error = setpoint - angle;
  integral += error;
  double derivative = error - last_error;
  double output = kp * error + ki * integral + kd * derivative;
  last_error = error;
  
  // Apply PID control output to motors
  if (output > 0) {
    left_motor.setSpeed(output);
    right_motor.setSpeed(output);
  } else {
    left_motor.setSpeed(-output);
    right_motor.setSpeed(-output);
  }
  
  // Calculate motor speeds in RPM
  unsigned long time_now = millis();
  static unsigned long last_time = time_now;
  unsigned
long time_delta = time_now - last_time;
last_time = time_now;
double left_rpm = calculate_motor_speed(left_encoder_count, time_delta);
double right_rpm = calculate_motor_speed(right_encoder_count, time_delta);

// Reset encoder counts
noInterrupts();
long left_encoder_count_now = left_encoder_count;
long right_encoder_count_now = right_encoder_count;
left_encoder_count -= left_encoder_count_now;
right_encoder_count -= right_encoder_count_now;
interrupts();

// Debug output to Serial monitor
Serial.print("Angle: ");
Serial.print(angle);
Serial.print(", Error: ");
Serial.print(error);
Serial.print(", Output: ");
Serial.print(output);
Serial.print(", Left RPM: ");
Serial.print(left_rpm);
Serial.print(", Right RPM: ");
Serial.println(right_rpm);

// Delay for PID control loop
delay(20);
}

// Interrupt service routine for left encoder
void left_encoder_isr() {
if (digitalRead(left_encoder_pin) == HIGH) {
left_encoder_count++;
} else {
left_encoder_count--;
}
}

// Interrupt service routine for right encoder
void right_encoder_isr() {
if (digitalRead(right_encoder_pin) == HIGH) {
right_encoder_count++;
} else {
right_encoder_count--;
}
}

// Function to calculate motor speed in RPM
double calculate_motor_speed(long encoder_count, unsigned long time_delta) {
const int encoder_resolution = 20; // Number of counts per revolution
double rpm = (encoder_count * 60000.0) / (encoder_resolution * time_delta);
return rpm;
}

// Function to handle user input and set motor speed and direction accordingly
void handle_user_input() {
int left_speed = 0;
int right_speed = 0;
if (Serial.available() > 0) {
char command = Serial.read();
switch (command) {
case 'w':
left_speed = 100;
right_speed = 100;
break;
case 's':
left_speed = -100;
right_speed = -100;
break;
case 'a':
left_speed = -50;
right_speed = 50;
break;
case 'd':
left_speed = 50;
right_speed = -50;
break;
default:
break;
}
}
set_motor_speed_and_direction(left_speed, right_speed);
}



