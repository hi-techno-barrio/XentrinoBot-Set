#include <opencv2/opencv.hpp>
#include <iostream>
#include <Serial.h>

// Serial communication settings
#define SERIAL_BAUDRATE 9600
Serial serial(SERIAL_BAUDRATE);

// Line detection settings
#define LOW_H 0
#define LOW_S 0
#define LOW_V 0
#define HIGH_H 179
#define HIGH_S 255
#define HIGH_V 255
#define KERNEL_SIZE 3
#define BLUR_SIZE 3
#define MIN_LINE_LENGTH 100
#define MAX_LINE_GAP 50
#define THRESHOLD 80

// Motor control settings
#define FORWARD_SPEED 50
#define BACKWARD_SPEED -50
#define ROTATE_SPEED 25
#define ROTATE_ANGLE 90

int main() {
    // Open camera capture device
    cv::VideoCapture cap(0);

    // Check if camera is opened successfully
    if (!cap.isOpened()) {
        std::cout << "Error: Failed to open camera" << std::endl;
        return -1;
    }

    // Set camera resolution
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    // Create named window for displaying camera output
    cv::namedWindow("Camera Output", cv::WINDOW_NORMAL);

    // Create trackbars for adjusting line detection settings
    cv::createTrackbar("Low H", "Camera Output", &LOW_H, 179);
    cv::createTrackbar("Low S", "Camera Output", &LOW_S, 255);
    cv::createTrackbar("Low V", "Camera Output", &LOW_V, 255);
    cv::createTrackbar("High H", "Camera Output", &HIGH_H, 179);
    cv::createTrackbar("High S", "Camera Output", &HIGH_S, 255);
    cv::createTrackbar("High V", "Camera Output", &HIGH_V, 255);
    cv::createTrackbar("Kernel Size", "Camera Output", &KERNEL_SIZE, 10);
    cv::createTrackbar("Blur Size", "Camera Output", &BLUR_SIZE, 10);
    cv::createTrackbar("Min Line Length", "Camera Output", &MIN_LINE_LENGTH, 500);
    cv::createTrackbar("Max Line Gap", "Camera Output", &MAX_LINE_GAP, 500);
    cv::createTrackbar("Threshold", "Camera Output", &THRESHOLD, 255);

    // Main loop
    while (true) {
        // Capture a frame from the camera
        cv::Mat frame;
        cap >> frame;

        // Apply color thresholding to detect line
        cv::Mat hsv, thresholded;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(LOW_H, LOW_S, LOW_V), cv::Scalar(HIGH_H, HIGH_S, HIGH_V), thresholded);

        // Apply morphological operations to remove noise
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(KERNEL_SIZE, KERNEL_SIZE));
        cv::morphologyEx(thresholded, thresholded, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(thresholded, thresholded, cv::MORPH_CLOSE, kernel

