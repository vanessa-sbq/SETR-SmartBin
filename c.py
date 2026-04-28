import cv2
import numpy as np
from ultralytics import YOLO
from picamera2 import Picamera2

# 1. Load the model
# Using the .pt file for now, but OpenVINO is recommended later for speed
model = YOLO("yolo26n.pt") 

# 2. Setup Picamera2
picam2 = Picamera2()
# We set a lower resolution (640x480) to keep the FPS high on the Pi CPU
config = picam2.create_video_configuration(main={"size": (320, 320)})
picam2.configure(config)
picam2.start()

print("Camera started! Press 'q' to exit.")

try:
    while True:
        # Capture the frame directly as a NumPy array (RGB)
        frame = picam2.capture_array()

        frame = frame[:, :, :3]

        # 3. Run YOLO inference
        # We use verbose=False to keep the terminal clean
        results = model.predict(frame, imgsz=320, conf=0.4, verbose=False)

        # 4. Process and Display
        # results[0].plot() returns an image with bounding boxes drawn
        annotated_frame = results[0].plot()
        
        # Picamera2 captures in RGB, but OpenCV displays in BGR
        # We must convert it or the colors will look "inverted" (blue people!)
        display_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_RGB2BGR)
        
        cv2.imshow("RPi5 YOLOv11 - Camera Module 3", display_frame)

        # Stop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Always stop the camera gracefully
    picam2.stop()
    cv2.destroyAllWindows()
