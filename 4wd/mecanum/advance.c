#include <Encoder.h>
#include <PID_v1.h>

// Motor pins
#define MOTOR1_PIN1 2
#define MOTOR1_PIN2 3
#define MOTOR2_PIN1 4
#define MOTOR2_PIN2 5
#define MOTOR3_PIN1 6
#define MOTOR3_PIN2 7
#define MOTOR4_PIN1 8
#define MOTOR4_PIN2 9

// Encoder pins
#define ENCODER1A_PIN 10
#define ENCODER1B_PIN 11
#define ENCODER2A_PIN 12
#define ENCODER2B_PIN 13
#define ENCODER3A_PIN 14
#define ENCODER3B_PIN 15
#define ENCODER4A_PIN 16
#define ENCODER4B_PIN 17

// Mecanum wheel diameter in mm
#define WHEEL_DIAMETER 100

// Define the encoder objects
Encoder encoder1(ENCODER1A_PIN, ENCODER1B_PIN);
Encoder encoder2(ENCODER2A_PIN, ENCODER2B_PIN);
Encoder encoder3(ENCODER3A_PIN, ENCODER3B_PIN);
Encoder encoder4(ENCODER4A_PIN, ENCODER4B_PIN);

// Define the PID objects
double SetpointVX, InputVX, OutputVX;
double SetpointVY, InputVY, OutputVY;
double SetpointVW, InputVW, OutputVW;
PID pidVX(&InputVX, &OutputVX, &SetpointVX, 1, 0, 0, DIRECT);
PID pidVY(&InputVY, &OutputVY, &SetpointVY, 1, 0, 0, DIRECT);
PID pidVW(&InputVW, &OutputVW, &SetpointVW, 1, 0, 0, DIRECT);

void setup() {
  // Set motor pins as outputs
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);
  pinMode(MOTOR3_PIN1, OUTPUT);
  pinMode(MOTOR3_PIN2, OUTPUT);
  pinMode(MOTOR4_PIN1, OUTPUT);
  pinMode(MOTOR4_PIN2, OUTPUT);

  // Set encoder pins as inputs
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

  // Initialize PID values
  SetpointVX = 0;
  SetpointVY = 0;
  SetpointVW = 0;
  InputVX = 0;
  InputVY = 0;
  InputVW = 0;
  pidVX.SetMode(AUTOMATIC);
  pidVY.SetMode(AUTOMATIC);
  pidVW.SetMode(AUTOMATIC);
}

void loop() {
  // Read encoder values
  long encoder1Value = encoder1.read();
  long encoder2Value = encoderwheel2Speed / 100));
int motor3Speed = (int)(255 * (wheel3Speed / 100));
int motor4Speed = (int)(255 * (wheel4Speed / 100));

// Set motor speeds
setMotorSpeed(MOTOR1_PIN1, MOTOR1_PIN2, motor1Speed);
setMotorSpeed(MOTOR2_PIN1, MOTOR2_PIN2, motor2Speed);
setMotorSpeed(MOTOR3_PIN1, MOTOR3_PIN2, motor3Speed);
setMotorSpeed(MOTOR4_PIN1, MOTOR4_PIN2, motor4Speed);

// Print encoder values to serial monitor for debugging
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

// Function to set motor speed based on pin values
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
