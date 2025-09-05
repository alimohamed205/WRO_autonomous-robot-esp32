import cv2
import numpy as np

# GStreamer pipeline for IMX219 (increased to 1920x1080)
gst_pipeline = (
    "nvarguscamerasrc ! "
    "video/x-raw(memory:NVMM), width=1920, height=1080, format=NV12, framerate=30/1 ! "
    "nvvidconv ! video/x-raw, format=BGRx ! "
    "videoconvert ! video/x-raw, format=BGR ! appsink"
)

# Initialize camera
cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("❌ فشل في فتح الكاميرا")
    exit()

# HSV ranges for red (refined for better detection)
lower_red1 = np.array([0, 100, 70])    # Slightly lower saturation threshold
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 100, 70])
upper_red2 = np.array([180, 255, 255])

# Kernel for morphological operations
kernel = np.ones((5, 5), np.uint8)

while True:
    ret, frame = cap.read()
    if not ret:
        print("⚠️ فشل في قراءة الكاميرا")
        break

    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Apply Gaussian blur to reduce noise
    hsv = cv2.GaussianBlur(hsv, (5, 5), 0)

    # Create red masks
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)

    # Clean up mask with morphological operations
    red_mask = cv2.erode(red_mask, kernel, iterations=1)
    red_mask = cv2.dilate(red_mask, kernel, iterations=1)

    # Create original color mask (red regions in original colors, rest black)
    red_original = cv2.bitwise_and(frame, frame, mask=red_mask)

    # Count red pixels for detection feedback
    red_pixel_count = cv2.countNonZero(red_mask)
    total_pixels = frame.shape[0] * frame.shape[1]
    red_percentage = (red_pixel_count / total_pixels) * 100

    # Display stats on frame
    cv2.putText(frame, f"Red Pixels: {red_pixel_count} ({red_percentage:.2f}%)",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Show images
    cv2.imshow("Camera Feed", frame)
    cv2.imshow("Red Mask (Binary)", red_mask)
    cv2.imshow("Red Original Colors", red_original)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
