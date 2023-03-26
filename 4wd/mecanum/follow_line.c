#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <string>
#include <Serial.h>

// Serial communication
#define SERIAL_BAUDRATE 9600
#define SERIAL_TIMEOUT 1000
#define SERIAL_COMMAND_SIZE 16
#define SERIAL_COMMAND_FORWARD 'F'
#define SERIAL_COMMAND_BACKWARD 'B'
#define SERIAL_COMMAND_LEFT 'L'
#define SERIAL_COMMAND_RIGHT 'R'
#define SERIAL_COMMAND_DIAGONAL 'D'
#define SERIAL_COMMAND_SPEED 'S'
#define SERIAL_COMMAND_ACCELERATION 'A'
#define SERIAL_COMMAND_STOP 'X'
Serial serialPort;

// Camera settings
#define CAMERA_INDEX 0
#define CAMERA_WIDTH 640
#define CAMERA_HEIGHT 480

// Image processing settings
#define KERNEL_SIZE 5
#define THRESHOLD_VALUE 100
#define LINE_COLOR cv::Scalar(0, 255, 0) // Green
#define LINE_THICKNESS 2

// Robot movement settings
#define ROBOT_SPEED 50
#define ROBOT_ACCELERATION 10
#define TURN_SPEED 50
#define TURN_ACCELERATION 5
#define LINE_FOLLOW_SPEED 30
#define LINE_FOLLOW_ACCELERATION 5

// PID control settings
#define PID_SAMPLE_TIME 20 // in milliseconds
#define PID_INPUT_MIN -CAMERA_WIDTH / 2
#define PID_INPUT_MAX CAMERA_WIDTH / 2
#define PID_OUTPUT_MIN -100
#define PID_OUTPUT_MAX 100
#define PID_KP 1.0
#define PID_KI 0.0
#define PID_KD 0.0

// Global variables for PID control
double Input = 0;
double Setpoint = 0;
double Output = 0;
double SetpointVX = 0;
double SetpointVY = 0;
double SetpointVW = 0;
PID pidVX(&Input, &Output, &SetpointVX, PID_KP, PID_KI, PID_KD, DIRECT);
PID pidVY(&Input, &Output, &SetpointVY, PID_KP, PID_KI, PID_KD, DIRECT);
PID pidVW(&Input, &Output, &SetpointVW, PID_KP, PID_KI, PID_KD, DIRECT);

// Function prototypes
void sendMovementCommand(char direction, int speed, int acceleration);
void parseSerialCommand(char* command);
void moveForward(int speed, int acceleration);
void moveBackward(int speed, int acceleration);
void moveLeft(int speed, int acceleration);
void moveRight(int speed, int acceleration);
void moveDiagonal(int speed, int acceleration);
void setRobotSpeed(int speed);
void setRobotAcceleration(int acceleration);
void stopRobot();

int main(int argc, char** argv) {
    // Open serial communication
    serialPort.begin(SERIAL_BAUDRATE);

    // Open camera
    cv::VideoCapture camera(CAMERA_INDEX);
    if (!camera.isOpened()) {
        std::cerr << "Failed to open camera." << std::endl;
        return -1;
    }
    camera.set(cv::CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);

    // Initialize PID control
    pidVX.SetMode(AUTOMATIC);
    pidVX.SetOutputLimits(PID_OUTPUT_MIN, PID_OUTPUT_MAX);
    pidVY.SetMode(AUTOMATIC);
    pidVY.SetOutputLimits(PID_OUTPUT_MIN, PID_OUTPUT_MAX);
    pidVW.SetMode(AUTOMATIC);
    pidVW.SetOutputLimits(PID_OUTPUT_MIN, PID_OUTPUT_MAX);

   // Main loop
while (true) {
    // Check for incoming serial commands
    if (Serial.available()) {
        char command[SERIAL_COMMAND_SIZE];
        Serial.readBytesUntil('\n', command, SERIAL_COMMAND_SIZE);
        parseSerialCommand(command);
    }

    // Calculate PID outputs
    double outputVX = pidVX.Compute();
    double outputVY = pidVY.Compute();
    double outputVW = pidVW.Compute();

    // Send movement command based on PID outputs
    if (outputVX >= 0 && outputVY >= 0 && outputVW >= 0) {
        sendMovementCommand('F', outputVX, outputVY, outputVW);
    } else if (outputVX >= 0 && outputVY >= 0 && outputVW < 0) {
        sendMovementCommand('D', outputVX, outputVY, -outputVW);
    } else if (outputVX >= 0 && outputVY < 0 && outputVW >= 0) {
        sendMovementCommand('E', outputVX, -outputVY, outputVW);
    } else if (outputVX >= 0 && outputVY < 0 && outputVW < 0) {
        sendMovementCommand('A', outputVX, -outputVY, -outputVW);
    } else if (outputVX < 0 && outputVY >= 0 && outputVW >= 0) {
        sendMovementCommand('G', -outputVX, outputVY, outputVW);
    } else if (outputVX < 0 && outputVY >= 0 && outputVW < 0) {
        sendMovementCommand('B', -outputVX, outputVY, -outputVW);
    } else if (outputVX < 0 && outputVY < 0 && outputVW >= 0) {
        sendMovementCommand('H', -outputVX, -outputVY, outputVW);
    } else if (outputVX < 0 && outputVY < 0 && outputVW < 0) {
        sendMovementCommand('C', -outputVX, -outputVY, -outputVW);
    }

    // Wait for a short delay to avoid flooding serial communication
    delay(10);
}
}

// Function to send movement command over serial communication
void sendMovementCommand(char direction, int speed, int acceleration) {
char command[SERIAL_COMMAND_SIZE];
snprintf(command, SERIAL_COMMAND_SIZE, "%c%d,%d", direction, speed, acceleration);
Serial.println(command);
}

// Function to parse serial commands and update robot movement
void parseSerialCommand(char* command) {
char direction = command[0];
int speed = atoi(&command[1]);
int acceleration = atoi(&command[3]);

// Update robot movement based on serial command
switch (direction) {
case SERIAL_COMMAND_FORWARD:
SetpointVX = speed;
SetpointVY = 0;
SetpointVW = 0;
break;
case SERIAL_COMMAND_BACKWARD:
SetpointVX = -speed;
SetpointVY = 0;
SetpointVW = 0;
break;
case SERIAL_COMMAND_LEFT:
SetpointVX = 0;
SetpointVY = speed;
SetpointVW = 0;
break;
case SERIAL_COMMAND_RIGHT:
SetpointVX = 0;
SetpointVY = -speed;
SetpointVW = 0;
break;
case SERIAL_COMMAND_DIAGONAL:
SetpointVX = speed;
SetpointVY = -speed;
SetpointVW = 0;
break;
case SERIAL_COMMAND_SPEED:
pidVX.SetTunings(speed, 0, 0);
pidVY.SetTunings(speed, 0, 0);
pidVW.SetTunings(speed, 0, 0);
break;
case SERIAL_COMMAND_ACCELERATION:
pidVX.SetTunings(0, 0, acceleration);
pidVY.SetTunings(0, 0, acceleration);
pidVW.SetTunings(0, 0, acceleration);
break;
case SERIAL_COMMAND_STOP:
SetpointVX = 0;
SetpointVY = 0;
SetpointVW = 0;
break;
}
}

// Function to move the robot forward with a given speed and acceleration
void moveForward(int speed, int acceleration) {
char command[SERIAL_COMMAND_SIZE];
snprintf(command, SERIAL_COMMAND_SIZE, "%c%d,%d", SERIAL_COMMAND_FORWARD, speed, acceleration);
Serial.println(command);
}

// Function to move the robot backward with a given speed and acceleration
void moveBackward(int speed, int acceleration) {
char command[SERIAL_COMMAND_SIZE];
snprintf(command, SERIAL_COMMAND_SIZE, "%c%d,%d", SERIAL_COMMAND_BACKWARD, speed, acceleration);
Serial.println(command);
}

// Function to move the robot left with a given speed and acceleration
void moveLeft(int speed, int acceleration) {
char command[SERIAL_COMMAND_SIZE];
snprintf(command, SERIAL_COMMAND_SIZE, "%c%d,%d", SERIAL_COMMAND_LEFT, speed, acceleration);
Serial.println(command);
}

// Function to move the robot right with a given speed and acceleration
void moveRight(int speed, int acceleration) {
char command[SERIAL_COMMAND_SIZE];
snprintf(command, SERIAL_COMMAND_SIZE, "%c%d,%d", SERIAL_COMMAND_RIGHT, speed, acceleration);
Serial.println(command);
}

// Function to move the robot diagonally with a given speed and acceleration
void moveDiagonal(int speed, int acceleration) {
char command[SERIAL_COMMAND_SIZE];
snprintf(command, SERIAL_COMMAND_SIZE, "%c%d,%d", SERIAL_COMMAND_DIAGONAL, speed, acceleration);
Serial.println(command);
}

// Function to set the robot speed with a given speed value
void setRobotSpeed(int speed) {
char command[SERIAL_COMMAND_SIZE];
snprintf(command, SERIAL_COMMAND_SIZE, "%c%d", SERIAL_COMMAND_SPEED, speed);
Serial.println(command);
}

// Function to set the robot acceleration with a given acceleration value
void setRobotAcceleration(int acceleration) {
char command[SERIAL_COMMAND_SIZE];
snprintf(command, SERIAL_COMMAND_SIZE, "%c%d", SERIAL_COMMAND_ACCELERATION, acceleration);
Serial.println(command);
}

// Function to stop the robot
void stopRobot() {
char command[SERIAL_COMMAND_SIZE];
snprintf(command, SERIAL_COMMAND_SIZE, "%c", SERIAL_COMMAND_STOP);
Serial.println(command);
}

// Example usage
void setup() {
// Initialize serial communication
Serial.begin(SERIAL_BAUDRATE);
Serial.setTimeout(SERIAL_TIMEOUT);
// Send a command to move the robot forward with speed 50 and acceleration 10
moveForward(50, 10);

// Wait for 5 seconds
delay(5000);

// Send a command to stop the robot
stopRobot();
}

void loop() {
// Do nothing
}
