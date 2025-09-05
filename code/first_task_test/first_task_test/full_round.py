import cv2
import numpy as np
import serial
import time

# Serial to ESP32
ser = serial.Serial('/dev/ttyTHS1', 115200, timeout=0.1)
ser.reset_input_buffer()
time.sleep(1)
print("✅ Connected to ESP32")

def get_yaw(tag="INITIAL_YAW", timeout=1):
    ser.write(f"GET_{tag}\n".encode())
    start = time.time()
    while time.time() - start < timeout:
        if ser.in_waiting:
            line = ser.readline().decode().strip()
            if line.startswith(f"{tag}:"):
                return float(line.split(":")[1])
    print(f"⚠️ Failed to get {tag}")
    return None

def send_command(command):
    ser.write(f"{command}\n".encode())
    ser.flush()
    print(f"➡️ Sent: {command}")

def steer_to_color(delta_angle):
    global initial_yaw
    target_yaw = (initial_yaw + delta_angle) % 360
    send_command(f"TURN_TO:{target_yaw:.2f}")
    initial_yaw = target_yaw

# Initialize yaw
initial_yaw = get_yaw("INITIAL_YAW") or 0
print(f"✅ Initial Yaw: {initial_yaw:.2f}")

# GStreamer camera setup
def gstreamer_pipeline():
    return (
        "nvarguscamerasrc sensor-id=0 ! "
        "video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
        "nvvidconv flip-method=0 ! "
        "video/x-raw, width=640, height=360, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink"
    )

cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("❌ Failed to open camera")
    exit()

# HSV color ranges
lower_orange = np.array([5, 100, 100])
upper_orange = np.array([15, 255, 255])
lower_blue = np.array([100, 100, 50])
upper_blue = np.array([135, 255, 255])
kernel = np.ones((5, 5), np.uint8)

# ROI
def extract_roi(frame):
    height, width = frame.shape[:2]
    start_y = int(height * 0.6)
    end_y = int(height * 0.9)
    start_x = int(width * 0.3)
    end_x = int(width * 0.7)
    roi = frame[start_y:end_y, start_x:end_x]
    cv2.rectangle(frame, (start_x, start_y), (end_x, end_y), (0, 255, 0), 2)
    return roi

# State variables
stage = 0
last_detect_time = time.time()
last_blue_seen = time.time()
color_counter = 0
color_seen = False
MAX_COLORS = 12

# Main loop
while True:
    ret, frame = cap.read()
    if not ret:
        print("⚠️ Frame read failed")
        break

    roi = extract_roi(frame)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    orange_mask = cv2.inRange(hsv, lower_orange, upper_orange)
    orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, kernel)

    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)

    M_orange = cv2.moments(orange_mask)
    M_blue = cv2.moments(blue_mask)
    cy_orange = int(M_orange["m01"] / M_orange["m00"]) if M_orange["m00"] != 0 else None
    cy_blue = int(M_blue["m01"] / M_blue["m00"]) if M_blue["m00"] != 0 else None

    orange_pixels = cv2.countNonZero(orange_mask)
    blue_pixels = cv2.countNonZero(blue_mask)
    detected_orange = orange_pixels > 600
    detected_blue = blue_pixels > 600
    print(f"Stage: {stage} | 🟠 Pixels: {orange_pixels} | 🔵 Pixels: {blue_pixels} | Count: {color_counter}")

    current_time = time.time()

    if stage == 0:
        if detected_orange and (current_time - last_detect_time > 1):
            print("🟠 Detected orange → Turning")
            steer_to_color(30)
            stage = 1
            last_detect_time = current_time
            if not color_seen:
                color_counter += 1
                color_seen = True
                print(f"🧮 Color Count: {color_counter}")

        elif detected_blue and cy_blue is not None and cy_orange is not None:
            if cy_blue < cy_orange + 10:
                print("🔵 Blue ignored")
            else:
                send_command("FORWARD")
                print("🔵 Moving forward")
                last_detect_time = current_time

        elif not detected_orange and not detected_blue and (current_time - last_detect_time > 1):
            print("🚶 Moving forward - no color detected")
            send_command("FORWARD")
            last_detect_time = current_time

    elif stage == 1:
        if detected_blue and (current_time - last_detect_time > 1):
            print("🔵 Detected blue → Turning")
            steer_to_color(30)
            stage = 2
            last_detect_time = current_time
            last_blue_seen = current_time
            if not color_seen:
                color_counter += 1
                color_seen = True
                print(f"🧮 Color Count: {color_counter}")

        elif detected_orange and cy_orange is not None and cy_blue is not None:
            if cy_blue < cy_orange + 10:
                print("🟠 Orange ignored")

    elif stage == 2:
        if detected_blue:
            last_blue_seen = current_time
        if (current_time - last_blue_seen > 0.5):
            print("🔁 Blue gone → Final Turn")
            steer_to_color(30)
            stage = 3
            last_detect_time = current_time
            if not color_seen:
                color_counter += 1
                color_seen = True
                print(f"🧮 Color Count: {color_counter}")

    elif stage == 3:
        if (time.time() - last_detect_time > 1):
            current_yaw = get_yaw("CURRENT_YAW")
            if current_yaw is not None:
                yaw_error = (initial_yaw - current_yaw + 540) % 360 - 180
                print(f"🔍 Final Yaw Difference: {yaw_error:.2f}")
                if abs(yaw_error) > 8:
                    print("🔄 Yaw difference not within range, retrying...")
                    steer_to_color(-yaw_error)
                    last_detect_time = time.time()
                else:
                    print("✅ Yaw difference OK, continuing forward")
                    stage = 0
                    last_detect_time = time.time()

    # Reset flag if no color detected
    if not detected_orange and not detected_blue:
        color_seen = False

    # Stop after 12 colors (3 laps)
    if color_counter >= MAX_COLORS:
        print("🏁 Finished 3 Laps!")
        send_command("STOP")
        break

    # Show debug windows
    combined = cv2.bitwise_or(orange_mask, blue_mask)
    result = cv2.bitwise_and(roi, roi, mask=combined)
    cv2.imshow("Detection", result)
    cv2.imshow("Frame", frame)
    cv2.imshow("Orange Mask", orange_mask)
    cv2.imshow("Blue Mask", blue_mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
ser.close()

