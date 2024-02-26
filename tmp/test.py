import time

import cv2
from ultralytics import YOLO

# Load a pretrained YOLO model
model = YOLO('pretrained_models/yolov8n.pt')

# Stream the video from the RTMP server
stream_url = "rtmp://127.0.0.1//"
stream_key = "q3"
cap = cv2.VideoCapture(stream_url + stream_key)

# Set the window size
width = 640
height = 640
cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
cv2.resizeWindow('frame', width, height)

# Loop through the video frames
while cap.isOpened():
    # Read the next frame
    success, frame = cap.read()
    if not success:
        break

    start = time.perf_counter()

    # Run YOLOv8 inference on the frame
    results = model(frame)

    end = time.perf_counter()
    total_time = end - start
    fps = 1 / total_time

    # Visualize the results on the frame
    annotated_frame = results[0].plot()

    # Add the FPS to the frame
    # cv2.putText(annotated_frame, f'FPS: {int(fps)}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    # Display the frame with the results
    cv2.imshow('frame', annotated_frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close the window
cap.release()
cv2.destroyAllWindows()
