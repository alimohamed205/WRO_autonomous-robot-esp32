
import cv2
import numpy as np
import serial
import time

# ========== Serial connection ==========
ser = serial.Serial('/dev/ttyTHS1', 115200, timeout=1)
time.sleep(2)
print("✅ Connected to ESP32")

# ========== Get initial yaw ==========
ser.write(b"GET_INITIAL_YAW\n")
initial_yaw = None
start = time.time()
while time.time() - start < 3:
    if ser.in_waiting:
        line = ser.readline().decode().strip()
        print("📥", line)
        if line.startswith("INITIAL_YAW:"):
            initial_yaw = float(line.split(":")[1])
            break
if initial_yaw is None:
    print("❌ Failed to get initial yaw, using 0")
    initial_yaw = 0
print(f"✅ Initial Yaw: {initial_yaw:.2f}")

# ========== GStreamer Pipeline Function ==========
def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

# ========== Camera Setup ==========
gst_pipeline = gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=30,
    flip_method=0
)
cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("❌ Failed to open camera")
    exit()

# ========== Color ranges ==========
lower_orange = np.array([5, 120, 110])
upper_orange = np.array([18, 230, 180])
lower_blue = np.array([110, 50, 50])
upper_blue = np.array([135, 255, 255])
kernel = np.ones((5, 5), np.uint8)

def extract_roi(frame):
    h, w = frame.shape[:2]
    roi = frame[int(h * 0.6):int(h * 0.85), int(w * 0.1):int(w * 1.0)]
    # Draw ROI rectangle on full frame
    cv2.rectangle(frame, (int(w * 0.1), int(h * 0.6)), (int(w * 1.0), int(h * 0.85)), (0, 255, 0), 2)
    return roi

# ========== Main loop ==========
stage = 0
sent_command = None
total_yaw_change = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("⚠️ Frame read failed")
        break

    roi = extract_roi(frame)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
    mask_orange = cv2.erode(mask_orange, kernel, iterations=1)
    mask_orange = cv2.dilate(mask_orange, kernel, iterations=1)

    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_blue = cv2.erode(mask_blue, kernel, iterations=1)
    mask_blue = cv2.dilate(mask_blue, kernel, iterations=1)

    orange_pixels = cv2.countNonZero(mask_orange)
    blue_pixels = cv2.countNonZero(mask_blue)
    detected_orange = orange_pixels > 300
    detected_blue = blue_pixels > 300

    if stage == 0:  # Move forward until orange
        if detected_orange:
            ser.write(b"FORWARD\n")
            print("🟠 Orange detected → FORWARD for 1s")
            time.sleep(1)
            ser.write(b"TURN_RIGHT\n")
            print("↪️ Turning right (30°)")
            time.sleep(1)
            ser.write(b"FORWARD\n")
            print("🚀 Moving forward...")
            stage = 1
            total_yaw_change += 30
        else:
            if sent_command != "FORWARD":
                ser.write(b"FORWARD\n")
                print("🚀 Moving forward...")
                sent_command = "FORWARD"
    elif stage == 1:  # Move forward until blue
        if detected_blue:
            ser.write(b"TURN_RIGHT\n")
            print("🔵 Blue detected → Turning right (30°)")
            time.sleep(1)
            ser.write(b"FORWARD\n")
            print("🚀 Moving forward...")
            stage = 2
            total_yaw_change += 30
    elif stage == 2:  # Move forward until blue disappears
        if not detected_blue:
            ser.write(b"TURN_RIGHT\n")
            print("🔁 Blue gone → Final turn (30°)")
            time.sleep(1)
            # Check if total yaw change is approximately 90°
            ser.write(b"GET_CURRENT_YAW\n")
            current_yaw = None
            start = time.time()
            while time.time() - start < 1:
                if ser.in_waiting:
                    line = ser.readline().decode().strip()
                    if line.startswith("CURRENT_YAW:"):
                        current_yaw = float(line.split(":")[1])
                        break
            if current_yaw is not None:
                yaw_diff = abs(normalize_angle(current_yaw - initial_yaw))
                print(f"📐 Total yaw change: {yaw_diff:.2f}")
                if abs(yaw_diff - 90) < 5:  # Allow 5° tolerance
                    ser.write(b"STOP\n")
                    print("🏁 Task Completed (90° achieved)")
                    stage = 3
                else:
                    print("⚠️ Yaw not 90°, continuing...")
            else:
                print("❌ Failed to get current yaw")
                ser.write(b"STOP\n")
                stage = 3
    elif stage == 3:
        break

    roi_result = cv2.bitwise_and(roi, roi, mask=cv2.bitwise_or(mask_orange, mask_blue))
    cv2.imshow("ROI Detected", roi_result)
    cv2.imshow("Full Frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ========== Cleanup ==========
cap.release()
cv2.destroyAllWindows()
ser.close()

def normalize_angle(angle):
    while angle >= 360:
        angle -= 360
    while angle < 0:
        angle += 360
    return angle

