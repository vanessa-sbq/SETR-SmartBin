import cv2
import numpy as np
from picamera2 import Picamera2

# Tune these for your environment
MIN_AREA = 300       # ignore tiny specks
MAX_AREA = 50000     # ignore huge regions (sky, ground)
MIN_ASPECT = 0.2     # filter out very long thin lines
MAX_ASPECT = 5.0

# HSV color ranges for common trash colors
# Each entry: (name, lower_hsv, upper_hsv, bgr_color_for_display)
COLOR_RANGES = [
    ("blue",   np.array([100, 80, 50]),  np.array([130, 255, 255]), (255, 100, 0)),
    ("red1",   np.array([0,   100, 80]), np.array([10,  255, 255]), (0,   0,   255)),
    ("red2",   np.array([170, 100, 80]), np.array([180, 255, 255]), (0,   0,   255)),
    ("yellow", np.array([20,  100, 100]),np.array([35,  255, 255]), (0,   220, 220)),
    ("white",  np.array([0,   0,   180]),np.array([180, 40,  255]), (200, 200, 200)),
    ("silver", np.array([0,   0,   120]),np.array([180, 30,  200]), (180, 180, 180)),
]

# Morphology kernel to clean up noise
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

picam2 = Picamera2()
config = picam2.create_video_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

print("Color-based trash detector started. Press 'q' to quit.")

try:
    while True:
        frame = picam2.capture_array()
        bgr = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        display = bgr.copy()
        detection_count = 0

        for (name, lower, upper, color) in COLOR_RANGES:
            mask = cv2.inRange(hsv, lower, upper)

            # Clean up the mask
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)  # remove speckles
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # fill gaps

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if not (MIN_AREA < area < MAX_AREA):
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                aspect = w / h
                if not (MIN_ASPECT < aspect < MAX_ASPECT):
                    continue

                # Draw box and label
                cv2.rectangle(display, (x, y), (x + w, y + h), color, 2)
                label = f"{name} ({int(area)}px)"
                cv2.putText(display, label, (x, max(y - 6, 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
                detection_count += 1

        cv2.putText(display, f"Detections: {detection_count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        cv2.imshow("Trash Detector (no model)", display)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    picam2.stop()
    cv2.destroyAllWindows()