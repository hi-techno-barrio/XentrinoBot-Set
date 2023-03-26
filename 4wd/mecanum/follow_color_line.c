/*

In this implementation, we use the C++ interface of the OpenCV library and call the necessary functions from within a C program. The program works as follows:

1. We define the same color range and line detection parameters as in the Python implementation.
2. We define the same movement commands as in the Python implementation, but as an array of strings.
3. We define a `process_frame()` function that takes a `Mat` object representing a frame from the video capture device and a character array representing the mode ("color" or "direction") as input, and returns a pointer to a character array representing the movement command.
4. In `main()`, we prompt the user to choose the mode and set up the video capture device using the `VideoCapture` class.
5. We capture a frame from the video capture device, process it using `process_frame()`, and output the resulting movement command to the console.
6. We display the frame with the detected lines or color pixels using the `imshow()` function.
7. We exit the loop and release the video capture device and close all windows on 'q' key press.

Note that the `process_frame()` function in this implementation works in the same way as in the Python implementation, but we use OpenCV functions and classes to perform the necessary image processing and computations. We also use C++ features such as vectors and iterators to simplify the implementation.

*/

#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace cv;

// Define the color range to detect (in HSV)
Scalar color_lower(30, 50, 50);
Scalar color_upper(70, 255, 255);

// Set up the line detection parameters
int line_rho = 1;
double line_theta = CV_PI / 180;
int line_threshold = 100;
int line_min_length = 50;
int line_max_gap = 10;

// Set up the movement commands
char *move_commands[] = {"Move left", "Move right", "Move up", "Move down", "Stop"};

// Define a function to process the image frame
char *process_frame(Mat frame, char *mode) {
    // Convert the frame to HSV color space
    Mat hsv;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    
    if (strcmp(mode, "color") == 0) {
        // Threshold the image to detect the color
        Mat mask;
        inRange(hsv, color_lower, color_upper, mask);
        
        // Compute the center of mass of the color pixels
        Moments m = moments(mask);
        if (m.m00 == 0) {
            return move_commands[4];
        }
        int cx = (int)(m.m10 / m.m00);
        int cy = (int)(m.m01 / m.m00);
        
        // Compute the movement command based on the location of the color pixels
        if (cx < frame.cols / 2) {
            return move_commands[0];
        }
        else {
            return move_commands[1];
        }
    }
    else if (strcmp(mode, "direction") == 0) {
        // Find the lines in the image
        Mat mask;
        inRange(hsv, Scalar(0, 0, 0), Scalar(0, 0, 255), mask);
        std::vector<Vec4i> lines;
        HoughLinesP(mask, lines, line_rho, line_theta, line_threshold, line_min_length, line_max_gap);
        
        // If no lines are detected, stop moving and return
        if (lines.empty()) {
            return move_commands[4];
        }
        
        // Compute the midpoint of each line and sort them by x-coordinate
        std::vector<int> midpoints;
        for (size_t i = 0; i < lines.size(); i++) {
            midpoints.push_back((lines[i][0] + lines[i][2]) / 2);
        }
        std::sort(midpoints.begin(), midpoints.end());
        
        // Compute the difference between adjacent midpoints
        std::vector<int> differences;
        std::adjacent_difference(midpoints.begin(), midpoints.end(), std::back_inserter(differences));
        
        // Compute the overall direction of the line
        if (differences[1] < 0) {
            return move_commands[0];
        }
        else {
            return move_commands[1];
        }
    }
    else {
        return move_commands[4];
    }
}

int main(int argc, char **argv) {
    // Prompt the user to choose the mode
    char mode[10];
    printf("Choose mode (color/direction): ");
    scanf("%s", mode);
    
    // Set up the video capture device
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        printf("Failed to open video capture device\n");
        return -1;
    }
    
    while (true) {
        // Capture a frame
    Mat frame;
    cap.read(frame);
    
    // Process the frame and get the movement command
    char *command = process_frame(frame, mode);
    
    // Compute the movement command based on the chosen mode
    printf("%s\n", command);
    
    // Show the frame with the detected lines or color pixels
    imshow("Frame", frame);
    
    // Exit on 'q' key press
    if (waitKey(1) == 'q') {
        break;
    }
}

// Release the video capture device and close all windows
cap.release();
destroyAllWindows();

return 0;
}
