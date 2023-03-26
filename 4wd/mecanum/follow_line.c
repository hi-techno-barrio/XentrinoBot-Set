#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <string>
#include <Serial.h>

// Serial communication
#define SERIAL_BAUDRATE 9600
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

// Function prototypes
void sendMovementCommand(char direction, int speed, int acceleration);

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

    // Main loop
    while (true) {
        // Capture image from camera
        cv::Mat image;
        camera >> image;
        if (image.empty()) {
            std::cerr << "Failed to capture image." << std::endl;
            continue;
        }

        // Convert image to grayscale
        cv::Mat grayscale;
        cv::cvtColor(image, grayscale, cv::COLOR_BGR2GRAY);

        // Apply thresholding to detect the line
        cv::Mat thresholded;
        cv::threshold(grayscale, thresholded, THRESHOLD_VALUE, 255, cv::THRESH_BINARY);

        // Apply morphological operations to remove noise
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(KERNEL_SIZE, KERNEL_SIZE));
        cv::morphologyEx(thresholded, thresholded, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(thresholded, thresholded, cv::MORPH_CLOSE, kernel);

        // Find contours in the thresholded image
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(thresholded, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        // Find the largest contour
        int largestIndex = -1;
        double largestArea = 0;
        for (int i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > largestArea) {
                largestArea = area;
                largestIndex = i;
            }
        }

        // Draw the line
        if (largestIndex >= 0) {
            cv::drawContours(image, contours, largestIndex, LINE_COLOR, LINE_THICKNESS);

            // Calculate deviation from center
            cv::Moments moments = cv::moments(contours[largestIndex]);
            double cx = moments.m10 / moments.m00;
            double deviation = cx - (CAMERA_WIDTH / 2);

           // Send movement command based on deviation
if (deviation < -50) {
sendMovementCommand('L', TURN_SPEED, TURN_ACCELERATION);
} else if (deviation > 50) {
sendMovementCommand('R', TURN_SPEED, TURN_ACCELERATION);
} else {
sendMovementCommand('F', LINE_FOLLOW_SPEED, LINE_FOLLOW_ACCELERATION);
}

// Wait for a short delay to avoid flooding serial communication
delay(10);
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
// initialize serial communication
Serial.begin(SERIAL_BAUDRATE);
Serial.setTimeout(SERIAL_TIMEOUT);

// send a command to move the robot forward with speed 50 and acceleration 10
moveForward(50, 10);

// wait for 5 seconds
delay(5000);

// send a command to stop the robot
stopRobot();
}

void loop() {
// do nothing
}
