import cv2

# Load the YOLO object detection model
net = cv2.dnn.readNetFromDarknet("path/to/yolo/config/file.cfg", "path/to/yolo/weights/file.weights")
classes = open("path/to/yolo/classes/file.names").read().strip().split("\n")
colors = np.random.uniform(0, 255, size=(len(classes), 3))

# Initialize the tracker
tracker = cv2.TrackerCSRT_create()

# Define the color and thickness for drawing the bounding box and tracking path
thickness = 2

# Loop through the video frames
while True:
    # Read the next frame
    ret, frame = cap.read()
    if not ret:
        break
    
    # Create a blob from the frame and pass it through the YOLO model
    blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)
    net.setInput(blob)
    outputs = net.forward(net.getUnconnectedOutLayersNames())
    
    # Loop through the detected objects and display them on the frame
    detected_objects = []
    for output in outputs:
        for detection in output:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:
                box = detection[0:4] * np.array([frame.shape[1], frame.shape[0], frame.shape[1], frame.shape[0]])
                (x, y, w, h) = box.astype("int")
                label = "{}: {:.2f}%".format(classes[class_id], confidence * 100)
                cv2.rectangle(frame, (x, y), (x + w, y + h), colors[class_id], thickness)
                cv2.putText(frame, label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colors[class_id], thickness)
                detected_objects.append(classes[class_id])
    
    # Prompt the user to select which object to track
    if len(detected_objects) > 0:
        print("Detected objects: {}".format(detected_objects))
        selected_object = input("Enter the name of the object to track:")
        if selected_object in detected_objects:
            for output in outputs:
                for detection in output:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    if classes[class_id] == selected_object:
                        box = detection[0:4] * np.array([frame.shape[1], frame.shape[0], frame.shape[1], frame.shape[0]])
                        (x, y, w, h) = box.astype("int")
                        bbox = (x, y, w, h)
                        tracker.init(frame, bbox)
                        break
    
    # Update the tracker
    success, bbox = tracker.update(frame)
    
    # Draw the bounding box and tracking path
    if success:
        (x, y, w, h) = [int(i) for i in bbox]
        cv2.rectangle(frame, (x, y), (x + w, y + h), colors[class_id], thickness)
        label = "{}: {:.2f}%".format(classes[class_id], confidence * 100)
        cv2.putText(frame, label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colors[class_id], thickness)
        # Print the coordinates of the center of the object
        print("{} coordinates: ({}, {})".format(selected
        object, int(x+w/2), int(y+h/2)))
    else:
        cv2.putText(frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
    
    # Display the output frame
    cv2.imshow("Object Detection and Tracking", frame)
    
    # Exit the loop if the user presses the 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close all windows
cap.release()
cv2.destroyAllWindows()
