import cv2
import numpy as np

# Load the pre-trained Haar cascade classifiers
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

# Define the Hough transform parameters
rho = 1
theta = np.pi/180
threshold = 20
min_line_length = 20
max_line_gap = 300

# Open the video stream
cap = cv2.VideoCapture(0)

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

    # Loop over the contours
    for contour in contours:
        # Compute the area and bounding box of the contour
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)

        # Skip the contour if it's too small
        if area < min_area:
            continue

        # Extract the ROI from the frame and resize it
        roi = gray[y:y+h, x:x+w]
        roi = cv2.resize(roi, (50, 50))

        # Match the ROI against the traffic sign templates
        stop_signs = stop_cascade.detectMultiScale(roi, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
        yield_signs = yield_cascade.detectMultiScale(roi, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
        speed_limit_signs = speed_limit_cascade.detectMultiScale(roi, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
        no_entry_signs = no_entry_cascade.detectMultiScale(roi, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        # Get the maximum confidence for the matches
        max_confidence = max(len(stop_signs), len(yield_signs), len(speed_limit_signs), len(no_entry_signs))

        # Determine the type of traffic sign based on the maximum confidence
        if max_confidence > min_confidence:
        if len(stop_signs) > len(yield_signs) and len(stop_signs) > len(speed_limit_signs) and len(stop_signs) > len(no_entry_signs):
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
        cv2.putText(frame, "STOP", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        elif len(yield_signs) > len(stop_signs) and len(yield_signs) > len(speed_limit_signs) and len(yield_signs) > len(no_entry_signs):
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 255), 2)
        cv2.putText(frame, "YIELD", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        elif len(speed_limit_signs) > len(stop_signs) and len(speed_limit_signs) > len(yield_signs) and len(speed_limit_signs) > len(no_entry_signs):
        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
        cv2.putText(frame, f"SPEED LIMIT: {speed_limit_signs[0][2]}", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        elif len(no_entry_signs) > len(stop_signs) and len(no_entry_signs) > len(yield_signs) and len(no_entry_signs) > len(speed_limit_signs):
        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 255, 0), 2)
        cv2.putText(frame, "NO ENTRY", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)# Detect lane lines using Hough transform
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
            left_lane_lines.append(line)
        else:
            right_lane_lines.append(line)

    # Average the left and right lane lines
    left_lane = np.mean(left_lane_lines, axis=0)
    right_lane = np.mean(right_lane_lines, axis=0)

    # Extrapolate the left and right lane lines to the bottom of the image
    y1 = frame.shape[0]
    y2 = int(y1 / 2)

    if left_lane isnot None:
left_x1 = int((y1 - left_lane[0][1]) / left_lane[0][0])
left_x2 = int((y2 - left_lane[0][1]) / left_lane[0][0])
cv2.line(frame, (left_x1, y1), (left_x2, y2), (0, 0, 255), 2)    if right_lane is not None:
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

# Display the final image
cv2.imshow("Road Follower", frame)

# Wait for a key press and check if it's the "q" key
if cv2.waitKey(1) == ord('q'):
    break

    Release the video stream and close all windows
    cap.release()
    cv2.destroyAllWindows()
