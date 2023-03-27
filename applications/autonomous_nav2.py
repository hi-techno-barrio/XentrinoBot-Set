

#Note that this is just a rough outline, and there are many details that would need to be worked out to create a functional GPS and obstacle detection system. For example, you would need to determine how to define the waypoints of the circuit of road, how to handle different types of obstacles, and how to implement feedback control to keep the robot on the road and avoid obstacles.



import cv2
import numpy as np

# Load the Haar cascade classifiers for traffic signs
stop_cascade = cv2.CascadeClassifier('stop_classifier.xml')
yield_cascade = cv2.CascadeClassifier('yield_classifier.xml')
speed_limit_cascade = cv2.CascadeClassifier('speed_limit_classifier.xml')
no_entry_cascade = cv2.CascadeClassifier('no_entry_classifier.xml')

# Define the HSV color range for red
lower_red = np.array([0, 50, 50])
upper_red = np.array([10, 255, 255])
lower_red2 = np.array([170, 50, 50])
upper_red2 = np.array([180, 255, 255])

# Define the minimum area for traffic signs
min_area = 2000

# Define the minimum confidence for traffic sign detection
min_confidence = 0.5

# Define the Hough transform parameters for lane detection
rho = 1
theta = np.pi/180
threshold = 20
min_line_length = 20
max_line_gap = 300

# Define the obstacle detection parameters
min_obstacle_area = 2000
min_obstacle_distance = 50

# Open the video stream
cap = cv2.VideoCapture(0)

# Main loop
while True:
    # Read the frame from the video stream
    ret, frame = cap.read()

    # Convert the image to grayscale and blur it
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Threshold the image to isolate the red color
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, lower_red, upper_red)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # Find contours in the thresholded image
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Loop over the contours to detect traffic signs
    for contour in contours:
        # Compute the area and bounding box of the contour
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)

        # Skip the contour if it's too small
        if area < min_area:
            continue

        # Extract the ROI from the frame and resize it
        roi = gray[y:y+h, x:x+w]
        roi = cv2.resize(roi, (64, 64))    # Detect traffic signs using the Haar cascade classifiers
    stop_signs = stop_cascade.detectMultiScale(roi, minSize=(64, 64), minNeighbors=3)
    yield_signs = yield_cascade.detectMultiScale(roi, minSize=(64, 64), minNeighbors=3)
    speed_limit_signs = speed_limit_cascade.detectMultiScale(roi, minSize=(64, 64), minNeighbors=3)
    no_entry_signs = no_entry_cascade.detectMultiScale(roi, minSize=(64, 64), minNeighbors=3)

    # Determine the type and confidence of the detected traffic sign
    sign_type = None
    confidence = 0

    if len(stop_signs) > 0 and len(stop_signs) >= len(yield_signs) and len(stop_signs) >= len(speed_limit_signs) and len(stop_signs) >= len(no_entry_signs):
        sign_type = "stop"
        confidence = stop_signs[0][2] / min_area
    elif len(yield_signs) > 0 and len(yield_signs) >= len(stop_signs) and len(yield_signs) >= len(speed_limit_signs) and len(yield_signs) >= len(no_entry_signs):
        sign_type = "yield"
        confidence = yield_signs[0][2] / min_area
    elif len(speed_limit_signs) > 0 and len(speed_limit_signs) >= len(stop_signs) and len(speed_limit_signs) >= len(yield_signs) and len(speed_limit_signs) >= len(no_entry_signs):
        sign_type = "speed_limit"
        confidence = speed_limit_signs[0][2] / min_area
    elif len(no_entry_signs) > 0 and len(no_entry_signs) >= len(stop_signs) and len(no_entry_signs) >= len(yield_signs) and len(no_entry_signs) >= len(speed_limit_signs):
        sign_type = "no_entry"
        confidence = no_entry_signs[0][2] / min_area

    # Draw a rectangle around the detected traffic sign if the confidence is high enough
    if sign_type is not None and confidence >= min_confidence:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.putText(frame, sign_type, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

# Detect lane lines using Hough transform
edges = cv2.Canny(blur, 50, 150)
lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]), min_line_length, max_line_gap)

# Draw the detected lane lines on the original image
if lines is not None:
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

# Find the left and right lane lines
left_lane_lines = []
right_lane_lines = []

if lines is not None:
    for line in lines:
        x1, y1, x2, y2 = line[0]
        slope = (y2 - y1) / (x2 - x1)
        if slope < 0:
            left_lane_lines.append(line
else:
right_lane_lines.append(line)# Average the left and right lane lines
left_lane = None
right_lane = None

if len(left_lane_lines) > 0:
    left_lane = np.average(left_lane_lines, axis=0)
if len(right_lane_lines) > 0:
    right_lane = np.average(right_lane_lines, axis=0)

# Extrapolate the left and right lane lines to the bottom of the image
y1 = frame.shape[0]
y2 = int(y1 * 0.6)

left_x1 = None
left_x2 = None
right_x1 = None
right_x2 = None

if left_lane is not None:
    left_x1 = int((y1 - left_lane[0][1]) / left_lane[0][0])
    left_x2 = int((y2 - left_lane[0][1]) / left_lane[0][0])
    cv2.line(frame, (left_x1, y1), (left_x2, y2), (0, 0, 255), 2)

if right_lane is not None:
    right_x1 = int((y1 - right_lane[0][1]) / right_lane[0][0])
    right_x2 = int((y2 - right_lane[0][1]) / right_lane[0][0])
    cv2.line(frame, (right_x1, y1), (right_x2, y2), (0, 0, 255), 2)

# Calculate the lane width and midpoint
if left_lane is not None and right_lane is not None:
    lane_width = right_x1 - left_x1
    midpoint = (left_x1 + right_x1) // 2

    # Display the lane width and midpoint on the image
    cv2.putText(frame, f"Lane Width: {lane_width}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(frame, f"Midpoint: {midpoint}", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    # Determine the road curve
    center = frame.shape[1] // 2
    diff = midpoint - center

    if diff > 0:
        cv2.putText(frame, "Curve Right", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    elif diff < 0:
        cv2.putText(frame, "Curve Left", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    else:
        cv2.putText(frame, "Straight", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
else:
    # No lane lines detected
    cv2.putText(frame, "No Lane Lines Detected", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

# Detect obstacles using OpenCV
obstacles = []

# Convert the image to grayscale and blur it
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
blur = cv2.GaussianBlur(gray, (5, 5), 0)

# Detect edges using the Canny edge detector
edges = cv2.Canny(blur, 50, 150)# Find contours in the thresholded image
contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Loop over the contours to detect obstacles
for contour in contours:
    # Compute the area and bounding box of the contour
    area = cv2.contourArea(contour)
    x, y, w, h = cv2.boundingRect(contour)

    # Skip the contour if it's too small
    if area < min_obstacle_area:
        continue

    # Calculate the distance to the obstacle
    obstacle_center = x + w / 2
    distance = frame.shape[1] / 2 - obstacle_center

    # Skip the obstacle if it's too far away
    if abs(distance) > min_obstacle_distance:
        continue

    # Draw a rectangle around the detected obstacle
    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)

    # Add the obstacle to the list of obstacles
    obstacles.append((x, y, w, h, distance))

# Sort the obstacles by distance
obstacles.sort(key=lambda x: abs(x[4]))

# Check if there are any obstacles in the path
if len(obstacles) > 0:
    # Avoid the closest obstacle
    obstacle = obstacles[0]
    if obstacle[4] > 0:
        # Obstacle is on the right, steer left
        cv2.putText(frame, "Avoiding Obstacle: Steer Left", (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    else:
        # Obstacle is on the left, steer right
        cv2.putText(frame, "Avoiding Obstacle: Steer Right", (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

# Display the image
cv2.imshow("Road Follower", frame)

# Exit if the user presses 'q'
if cv2.waitKey(1) & 0xFF == ord('q'):
    break
Release the video capture and close the window
cap.release()
cv2.destroyAllWindows()
                                   
                                   

