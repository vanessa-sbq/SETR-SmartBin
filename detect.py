import cv2
from ultralytics import YOLO

# 1. Load the model (Nano is best for Pi 5)
# Use "yolo11n_openvino_model/" here if you exported it earlier
model = YOLO("yolo11n.pt") 

# 2. Initialize Camera
# '0' is usually the default camera. 
cap = cv2.VideoCapture(0)

if not cap.isOpened():
	print("Cant open Camera")

# Optional: Set resolution to improve FPS (320x320 or 640x640)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)

print("Starting stream... Press 'q' to quit.")

while cap.isOpened():
    success, frame = cap.read()

    if success:
        # Run YOLOv11 inference on the frame
        # persist=True helps keep track of objects across frames
        results = model.predict(frame, conf=0.45, show=False, stream=True)

        for r in results:
            annotated_frame = r.plot()
            
            # Display the frame
            cv2.imshow("Pi 5 YOLOv11 Live", annotated_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        break

cap.release()
cv2.destroyAllWindows()
