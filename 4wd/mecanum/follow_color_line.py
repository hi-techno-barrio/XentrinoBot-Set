
# In this implementation, we prompt the user to choose the mode (either "color" or "direction") at the beginning of the program. The `process_frame()` function now takes an additional argument `mode` that specifies whether to follow the color or direction of the line.

#If `mode` is "color", we threshold the image to detect the color using the specified lower and upper bounds in HSV color space. We then compute the center of mass of the color pixels using the `cv2.moments()` function, and determine the movement command based on the location of the center of mass relative to the center of the frame.

#If `mode` is "direction", we use the HoughLinesP algorithm to detect the lines in the image. We then compute the direction of the line as before, and return it as the movement command.

#In the main loop of the program, we call `process_frame()` on each frame with the chosen mode, and compute the movement command based on the output of the function. We then output the movement command to the console and display the frame with the detected lines or color pixels.

# Note that this implementation assumes that the robot is capable of moving in the directions "left", "right", "up", and "down", and that the `move_commands` dictionary maps these directions to the appropriate movement commands for the robot. If your robot uses a different set of commands or movements, you will need to modify this dictionary accordingly.


import cv2
import numpy as np

# Define the color range to detect (in HSV)
color_lower = (30, 50, 50)
color_upper = (70, 255, 255)

# Set up the line detection parameters
line_params = dict(
    rho=1,              # distance resolution of the accumulator in pixels
    theta=np.pi/180,    # angle resolution of the accumulator in radians
    threshold=100,      # minimum number of votes required to accept a detected line
    minLineLength=50,   # minimum length of a line in pixels
    maxLineGap=10       # maximum allowed gap between line segments in pixels
)

# Set up the movement commands
move_commands = {
    "left":  "Move left",
    "right": "Move right",
    "up":    "Move up",
    "down":  "Move down",
    "stop":  "Stop"
}

# Define a function to process the image frame
def process_frame(frame, mode):
    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    if mode == "color":
        # Threshold the image to detect the color
        mask = cv2.inRange(hsv, color_lower, color_upper)
        
        # Compute the center of mass of the color pixels
        m = cv2.moments(mask)
        if m["m00"] == 0:
            return "stop"
        cx = int(m["m10"] / m["m00"])
        cy = int(m["m01"] / m["m00"])
        
        # Compute the movement command based on the location of the color pixels
        if cx < frame.shape[1]//2:
            return "left"
        else:
            return "right"
    
    elif mode == "direction":
        # Find the lines in the image
        mask = cv2.inRange(hsv, (0, 0, 0), (0, 0, 255))
        lines = cv2.HoughLinesP(mask, **line_params)
        
        # If no lines are detected, stop moving and return
        if lines is None:
            return "stop"
        
        # Compute the midpoint of each line and sort them by x-coordinate
        midpoints = [(x1+x2)//2 for x1,y1,x2,y2 in lines[0]]
        midpoints.sort()
        
        # Compute the difference between adjacent midpoints
        differences = np.diff(midpoints)
        
        # Compute the overall direction of the line
        direction = "left" if differences[0] < 0 else "right"
        
        # Return the direction of the line
        return direction
    
    else:
        return "stop"

# Set up the video capture device
cap = cv2.VideoCapture(0)

# Prompt the user to choose the mode
mode = input("Choose mode (color/direction): ")

while True:
    # Capture a frame from the camera
    ret, frame = cap.read()
    if not ret:
        break
    
    # Process the frame and get the movement command
    command = process_frame(frame, mode)
    
    # Compute the movement command based on the chosen mode
    if command == "stop":
        print("Stop")
    else:
        movement = move_commands[command]
        print(movement)
    
    # Show the frame with the detected lines or color pixels
    cv2.imshow("frame", frame)# Exit on 'q' key press
if cv2.waitKey(1) == ord('q'):
    break
   # Release the video capture device and close all windows
 cap.release()
 cv2.destroyAllWindows()
  
  
