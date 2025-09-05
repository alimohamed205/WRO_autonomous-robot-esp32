#!/usr/bin/env python3
"""
WRO 2025 – ONE-LAP TEST with Wall Detection
Orange → 30° → Blue → 30° → final 30° → STOP
"""

import cv2, numpy as np, serial, threading, queue, time, sys

# ---------- CONFIG ----------
SERIAL_PORT   = '/dev/ttyTHS1'
BAUD_RATE     = 115200
SHOW_GUI      = False
LOCKOUT_S     = 0.1
ALIVE_S       = 1.0
FRAME_W, FRAME_H = 640, 360

ROI_Y1, ROI_Y2   = int(FRAME_H * 0.2), int(FRAME_H * 0.8)  # For color detection
ROI_X1, ROI_X2   = int(FRAME_W * 0.3), int(FRAME_W * 0.7)  # For color detection
RIGHT_ROI_X1, RIGHT_ROI_X2 = int(FRAME_W * 0.80), int(FRAME_W * 0.95)  # Right wall ROI
LEFT_ROI_X1, LEFT_ROI_X2   = int(FRAME_W * 0.05), int(FRAME_W * 0.20)  # Left wall ROI
HSV_ORANGE = (np.array([5, 120, 110]), np.array([15, 230, 180]))
HSV_BLUE   = (np.array([110, 50, 50]), np.array([135, 255, 255]))
HSV_WALL   = (np.array([90, 10, 50]), np.array([165, 65, 85]))  # White walls
THRESH_PIXELS = 100
BLUE_CONFIDENCE_THRESHOLD = 1
WALL_PIXEL_THRESHOLD = 50  # Minimum pixels to consider wall detected
STEER_KP = 0.1  # Proportional gain for steering correction
STEER_INTERVAL = 0.1  # Send STEER every 0.1 seconds
initial_alignment_done = False
alignment_threshold = 500
alignment_centering = False  # علشان نعرف إذا كنا بنرجّع الوضع ولا لسه بنضبط المكان
STEER_KP = 0.25    # الحساسية لتغير البكسل
IMU_KP   = 2.5     # الحساسية لفرق الزاوية
MAX_STEER = 120
MIN_STEER = 60


# ---------------------------------------------

# ---------- Camera Class ----------
class Camera:
    def __init__(self):
        self.cap = cv2.VideoCapture(
            "nvarguscamerasrc sensor-id=0 ! "
            "video/x-raw(memory:NVMM),width=640,height=480,framerate=30/1 ! "
            "nvvidconv flip-method=0 ! "
            "video/x-raw,width=640,height=360,format=BGRx ! "
            "videoconvert ! video/x-raw,format=BGR ! appsink",
            cv2.CAP_GSTREAMER
        )
        if not self.cap.isOpened():
            sys.exit("❌ Cannot open camera")

        self.lock = threading.Lock()
        self.latest_frame = None
        self.running = True
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.latest_frame = frame
            else:
                time.sleep(0.02)  # increased sleep to reduce CPU load

    def read(self):
        with self.lock:
            if self.latest_frame is None:
                return False, None
            return True, self.latest_frame.copy()

    def release(self):
        self.running = False
        time.sleep(0.1)
        self.cap.release()

# ---------- Serial ----------
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
rx_q = queue.Queue()

def uart_reader():
    buf = ''
    while True:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting).decode(errors='ignore')
            while '\n' in buf:
                line, buf = buf.split('\n', 1)
                rx_q.put(line.strip())
        time.sleep(0.001)

threading.Thread(target=uart_reader, daemon=True).start()

def tx(cmd):
    print(f"➡️ {cmd}")
    ser.write((cmd + '\n').encode())

def wait_token(token, tout=1.5):  # reduced timeout
    t0 = time.time()
    while time.time() - t0 < tout:
        try:
            ln = rx_q.get_nowait()
            if ln.startswith(token + ':'):
                return float(ln.split(':', 1)[1])
            if ln.strip() == token:
                return True
        except queue.Empty:
            time.sleep(0.01)
    return None

# ---------- Helpers ----------
def extract_color_roi(frame):
    height, width = frame.shape[:2]
    roi_w = int(width * 0.4)
    roi_h = int(height * 0.4)
    roi_cx = int(width * 0.35)
    roi_cy = int(height * 0.7)
    angle = 30
    rect = ((roi_cx, roi_cy), (roi_w, roi_h), angle)
    box = cv2.boxPoints(rect).astype(np.int32)
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillConvexPoly(mask, box, 255)

    roi_colored = cv2.bitwise_and(frame, frame, mask=mask)
    hsv = cv2.cvtColor(roi_colored, cv2.COLOR_BGR2HSV)
    cv2.polylines(frame, [box], isClosed=True, color=(0, 255, 0), thickness=2)

    return roi_colored, hsv, box

def extract_wall_roi(frame, x1, x2, y1, y2):
    mask = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)
    cv2.rectangle(mask, (x1, y1), (x2, y2), 255, -1)
    roi_colored = cv2.bitwise_and(frame, frame, mask=mask)
    hsv = cv2.cvtColor(roi_colored, cv2.COLOR_BGR2HSV)
    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Draw ROI rectangle
    return hsv

def detect_color(hsv, lo, hi):
    mask = cv2.inRange(hsv, lo, hi)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    return mask, cv2.countNonZero(mask)

def angle_diff(a, b):
    return (a - b + 540) % 360 - 180

# ---------- Start ----------
tx("GET_INITIAL_YAW")
start_yaw = wait_token("INITIAL_YAW")
if start_yaw is None:
    sys.exit("❌ ESP32 not responding")
print(f"🎯 Start yaw = {start_yaw:.1f}")

# ---------- State Engine ----------
camera = Camera()
stage = 0
turn_sent = 0
seen_time = {'orange': 0, 'blue': 0}
blue_confidence = 0
last_processed = 0
stage_5_start_time = 0
last_steer_time = 0


# Initial pause to stabilize camera
tx("PAUSE")
time.sleep(1.0)

while True:
    now = time.time()
    if now - last_processed < 0.05:
        continue
    last_processed = now

    ret, frame = camera.read()
    if not ret:
        print("⚠️ No frame captured")
        continue

    # Color detection ROI (for orange and blue)
    roi_colored, hsv_color, color_box = extract_color_roi(frame)

    # Wall detection ROIs (right and left)
    hsv_right = extract_wall_roi(frame, RIGHT_ROI_X1, RIGHT_ROI_X2, ROI_Y1, ROI_Y2)
    hsv_left = extract_wall_roi(frame, LEFT_ROI_X1, LEFT_ROI_X2, ROI_Y1, ROI_Y2)

    # Detect colors
    omask, opx = detect_color(hsv_color, *HSV_ORANGE)
    bmask, bpx = detect_color(hsv_color, *HSV_BLUE)
    has_orange = opx > THRESH_PIXELS
    has_blue = bpx > THRESH_PIXELS

    # Detect walls
    wmask_right, right_pixels = detect_color(hsv_right, *HSV_WALL)
    wmask_left, left_pixels = detect_color(hsv_left, *HSV_WALL)
    has_right_wall = right_pixels > WALL_PIXEL_THRESHOLD
    has_left_wall = left_pixels > WALL_PIXEL_THRESHOLD

    print(f"📍 Stage {stage} | 🟠 {opx} | 🔵 {bpx} | Right Wall: {right_pixels} | Left Wall: {left_pixels}")

    # Steering correction based on wall distance
    if not initial_alignment_done:
       diff = left_pixels - right_pixels
       print(f"[ALIGN] Left: {left_pixels}, Right: {right_pixels}, diff = {diff}")

       if abs(diff) > alignment_threshold and not alignment_centering:
        # ➊ لسه مش في النص – خليه يوجه نفسه
          if diff > 0:
            print("🔄 Align: steering left")
            tx("STEER_LEFT")
            time.sleep(1.0)
          else:
            print("🔄 Align: steering right")
            tx("STEER_RIGHT")

        #send_command("FORWARD")  # يتحرك للأمام أثناء التوجيه
        #time.sleep(0.3)
         #send_command("STOP")     # يقف ويقرأ الحالة من جديد
         #time.sleep(0.1)

       elif abs(diff) <= alignment_threshold and not alignment_centering:
        # ➋ جاب النص تقريبًا – جهّز الخطوة اللي بعدها (التسنتر)
         alignment_centering = True
         print("✅ Alignment almost done, recentring...")

        # خليه يرجع تاني لزاويته الأصلية (start_yaw)
         tx(f"TURN_TO:{start_yaw}")
         time.sleep(0.8)

        # بعد ما يظبط الزاوية، خليه يوقف شوية قبل دخول stage 0
         tx("PAUSE")
         time.sleep(1)

        # خلاص اعتبرناه اتحاذى تمامًا
         initial_alignment_done = True
         print("✅ Initial alignment complete. Moving to stage 0...")

         continue


    if has_orange:
        seen_time['orange'] = now + ALIVE_S
    if has_blue:
        seen_time['blue'] = now + ALIVE_S
        blue_confidence += 1
    else:
        blue_confidence = max(0, blue_confidence - 1)

    if stage == 0:
        tx(f"FORWARD_CORRECT:{start_yaw:.1f}")
        if now - seen_time['orange'] < ALIVE_S and now - turn_sent > LOCKOUT_S:
            tx(f"TURN_TO:{(start_yaw + 30) % 360:.1f}")
            turn_sent = now
            stage = 1

    elif stage == 1:
        print("⏳ Stage 1: Waiting TURN_DONE after orange")
        if has_blue and now - seen_time['blue'] < ALIVE_S:
            print(f"🔵 Blue detected, confidence={blue_confidence}, pausing to confirm")
            tx("PAUSE")
            time.sleep(0.5)
            if blue_confidence >= BLUE_CONFIDENCE_THRESHOLD:
                print("🔵 Blue confirmed, moving to Stage 2")
                stage = 2
        elif wait_token("TURN_DONE", tout=3.0):
            print("✅ TURN_DONE received, moving to Stage 2")
            stage = 2
        else:
            print("⚠️ No TURN_DONE received, retrying TURN_TO")
            tx(f"TURN_TO:{(start_yaw + 30) % 360:.1f}")
            turn_sent = now

    elif stage == 2:
        tx(f"FORWARD_CORRECT:{(start_yaw + 30) % 360:.1f}")
        if now - seen_time['blue'] < ALIVE_S and now - turn_sent > LOCKOUT_S and blue_confidence >= BLUE_CONFIDENCE_THRESHOLD:
            tx(f"TURN_TO:{(start_yaw + 60) % 360:.1f}")
            turn_sent = now
            stage = 3

    elif stage == 3:
        print("⏳ Stage 3: Waiting TURN_DONE after blue")
        if wait_token("TURN_DONE", tout=3.0):
            print("✅ TURN_DONE received, moving to Stage 4")
            stage = 4
        else:
            print("⚠️ No TURN_DONE received, retrying TURN_TO")
            tx(f"TURN_TO:{(start_yaw + 60) % 360:.1f}")
            turn_sent = now

    elif stage == 4:
        print("🔁 Final correction turn to 90°")
        tx(f"TURN_TO:{(start_yaw + 90) % 360:.1f}")
        stage = 5
        stage_5_start_time = now

    elif stage == 5:
        if now - stage_5_start_time > 10.0:
            print("⏰ Stage 5 timeout, stopping")
            tx("STOP")
            print("🏁 Finished")
            break
        print("⏳ Stage 5: Requesting CURRENT_YAW")
        tx("GET_CURRENT_YAW")
        yaw_now = wait_token("CURRENT_YAW", tout=3.0)
        if yaw_now is None:
            print("⚠️ No CURRENT_YAW received, retrying...")
            continue
        error = angle_diff(start_yaw + 90, yaw_now)
        print(f"📐 Final yaw error = {error:.1f}")
        if abs(error) < 6:
            tx("STOP")
            print("🏁 Finished")
            break
        else:
            print("⚠️ Yaw error too large, retrying TURN_TO")
            tx(f"TURN_TO:{(start_yaw + 90) % 360:.1f}")
            time.sleep(0.5)

    if SHOW_GUI:
        combined = cv2.bitwise_or(
            cv2.bitwise_and(roi_colored, roi_colored, mask=omask),
            cv2.bitwise_and(roi_colored, roi_colored, mask=bmask)
        )
        cv2.imshow("ROI Detection", combined)
        cv2.imshow("Frame", frame)
        cv2.imshow("🟠 Mask", omask)
        cv2.imshow("🔵 Mask", bmask)
        cv2.imshow("Right Wall Mask", wmask_right)
        cv2.imshow("Left Wall Mask", wmask_left)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# ---------- Cleanup ----------
tx("STOP")
camera.release()
if SHOW_GUI:
    cv2.destroyAllWindows()
ser.close()
