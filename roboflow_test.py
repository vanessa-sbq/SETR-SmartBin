import numpy as np
import cv2
from ultralytics import YOLO
from picamera2 import Picamera2

# COCO classes that represent common litter/trash
# Full list: https://docs.ultralytics.com/datasets/detect/coco/#categories
LITTER_CLASSES = {
    39: "bottle",
    40: "wine glass",
    41: "cup",
    42: "fork",
    43: "knife",
    44: "spoon",
    45: "bowl",
    46: "banana",
    47: "apple",
    48: "sandwich",
    49: "orange",
    67: "cell phone",
    73: "book",
    74: "clock",
    76: "scissors",
    77: "teddy bear",
    79: "toothbrush",
}

# Color per class for nicer visuals (BGR)
CLASS_COLORS = {
    "bottle":     (0, 255, 0),
    "cup":        (0, 200, 255),
    "bowl":       (255, 100, 0),
    "banana":     (0, 220, 220),
    "apple":      (0, 80, 255),
    "cell phone": (200, 0, 255),
    "book":       (255, 180, 0),
}
DEFAULT_COLOR = (200, 200, 200)

# 1. Load YOLO model
model = YOLO("yolo11n.pt")

# 2. Setup Picamera2
picam2 = Picamera2()
config = picam2.create_video_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

print("Drone trash detector started. Press 'q' to quit.")
print(f"Watching for: {', '.join(LITTER_CLASSES.values())}")

def draw_box(frame, box, label, color):
    x1, y1, x2, y2 = map(int, box)
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    # Label background
    text_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
    cv2.rectangle(frame, (x1, y1 - text_size[1] - 8), (x1 + text_size[0] + 4, y1), color, -1)
    cv2.putText(frame, label, (x1 + 2, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

try:
    while True:
        frame = picam2.capture_array()

        # After - strip alpha, then convert
        frame_rgb = frame[:, :, :3]  # drop the alpha channel
        results = model.predict(frame_rgb, conf=0.35, verbose=False, stream=True)
        display_frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        detection_count = 0

        for r in results:
            for box in r.boxes:
                class_id = int(box.cls[0])

                # Skip anything not in our litter list
                if class_id not in LITTER_CLASSES:
                    continue

                class_name = LITTER_CLASSES[class_id]
                conf = float(box.conf[0])
                color = CLASS_COLORS.get(class_name, DEFAULT_COLOR)
                label = f"{class_name} {conf:.2f}"

                draw_box(display_frame, box.xyxy[0], label, color)
                detection_count += 1

        # HUD overlay
        hud = f"Detections: {detection_count}"
        cv2.putText(display_frame, hud, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        cv2.imshow("Drone Trash Detector", display_frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    picam2.stop()
    cv2.destroyAllWindows()