import numpy as np
import cv2
from ultralytics import YOLO
from picamera2 import Picamera2
from picamera2.outputs import PILOutput

# 1. Load the YOLO model (Nano version)
model = YOLO("yolo11n.pt") 

# 2. Setup Picamera2
picam2 = Picamera2()
# Configure for a standard 640x480 stream for speed
config = picam2.create_video_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

print("Camera Module 3 started. Press 'q' to quit.")

try:
    while True:
        # Capture a frame as a NumPy array
        frame = picam2.capture_array()

        # Run inference
        # We use stream=True for better memory management
        results = model.predict(frame, conf=0.4, verbose=False, stream=True)

        for r in results:
            # Draw boxes on the frame
            annotated_frame = r.plot()
            
            # Convert RGB (Picamera2) to BGR (OpenCV) for display
            display_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_RGB2BGR)
            
            cv2.imshow("Pi 5 + Cam Module 3", display_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    picam2.stop()
    cv2.destroyAllWindows()
