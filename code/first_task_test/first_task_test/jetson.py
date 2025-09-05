#!/usr/bin/env python3
"""
WRO 2025 – ONE-LAP TEST (Debug Version)
Orange → 30° → Blue → 30° → final 30° → STOP
"""

import cv2, numpy as np, serial, threading, queue, time, sys

# ---------- CONFIG ----------
SERIAL_PORT   = '/dev/ttyTHS1'
BAUD_RATE     = 115200
SHOW_GUI      = False # disable GUI to reduce lag
LOCKOUT_S     = 0.1
ALIVE_S       = 1.0  # increased for stable blue detection
FRAME_W, FRAME_H = 640, 360
ROI_Y1, ROI_Y2   = int(FRAME_H * 0.2), int(FRAME_H * 0.9)
ROI_X1, ROI_X2   = int(FRAME_W * 0.3), int(FRAME_W * 0.7)
HSV_ORANGE = (np.array([5, 120, 110]), np.array([15, 230, 180]))
HSV_BLUE   = (np.array([110, 50, 50]), np.array([135, 255, 255]))
kernel = np.ones((5, 5), np.uint8)
THRESH_PIXELS = 100
BLUE_CONFIDENCE_THRESHOLD = 1
TURNS_PER_LAP = 12
TOTAL_LAPS = 3
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
def extract_roi(frame):
    height, width = frame.shape[:2]
    roi_w = int(width * 0.4)  # reduced ROI size
    roi_h = int(height * 0.4)
    roi_cx = int(width * 0.45)
    roi_cy = int(height * 0.75)
    angle = 30
    rect = ((roi_cx, roi_cy), (roi_w, roi_h), angle)
    box = cv2.boxPoints(rect).astype(np.int32)
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillConvexPoly(mask, box, 255)

    roi_colored = cv2.bitwise_and(frame, frame, mask=mask)
    hsv = cv2.cvtColor(roi_colored, cv2.COLOR_BGR2HSV)
    cv2.polylines(frame, [box], isClosed=True, color=(0, 255, 0), thickness=2)

    return roi_colored, hsv

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
stage_5_start_time = 0  # Track time in Stage 5
tx("PAUSE")
time.sleep(1.0)


while True:
    now = time.time()
    if now - last_processed < 0.05:
        continue
    last_processed = now

    ret, frame = camera.read()
    if not ret:
        continue
    roi, hsv = extract_roi(frame)

    omask, opx = detect_color(hsv, *HSV_ORANGE)
    bmask, bpx = detect_color(hsv, *HSV_BLUE)
    has_orange = opx > THRESH_PIXELS
    has_blue = bpx > THRESH_PIXELS

    # Log detections
    print(f"📍 Stage {stage} | 🟠 {opx} | 🔵 {bpx}")

    # Record last detection time
    if has_orange:
        seen_time['orange'] = now + ALIVE_S
    if has_blue:
        seen_time['blue'] = now + ALIVE_S
        blue_confidence += 1
    else:
        blue_confidence = max(0, blue_confidence - 1)

    # ---------- Logic ----------
    if stage == 0:
        tx(f"FORWARD_CORRECT:{start_yaw:.1f}")
        if now - seen_time['orange'] < ALIVE_S and now - turn_sent > LOCKOUT_S:
            tx(f"TURN_TO:{(start_yaw + 30) % 360:.1f}")
            turn_sent = now
            stage = 1

    elif stage == 1:
        print("⏳ Stage 1: Waiting TURN_DONE after orange")
        if has_blue and now - seen_time['blue'] < ALIVE_S and blue_confidence >= BLUE_CONFIDENCE_THRESHOLD:
            print("🔵 Blue detected reliably, pausing and moving to Stage 2")
           # tx("PAUSE")
            #time.sleep(0.2)
            stage = 2
        elif wait_token("TURN_DONE", tout=1.5):
            print("✅ TURN_DONE received, moving to Stage 2")
            stage = 2
        else:
            print("⚠️ No TURN_DONE received, retrying TURN_TO")
            tx(f"TURN_TO:{(start_yaw + 30) % 360:.1f}")
            turn_sent = now

    elif stage == 2:
        #tx(f"FORWARD_CORRECT:{(start_yaw + 30) % 360:.1f}")
        if now - seen_time['blue'] < ALIVE_S and now - turn_sent > LOCKOUT_S and blue_confidence >= BLUE_CONFIDENCE_THRESHOLD:
            tx(f"TURN_TO:{(start_yaw + 60) % 360:.1f}")
            turn_sent = now
            stage = 3

    elif stage == 3:
        print("⏳ Stage 3: Waiting TURN_DONE after blue")
        if wait_token("TURN_DONE", tout=1.5):
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
        stage_5_start_time = now  # Start timing Stage 5


    elif stage == 5:
        if now - stage_5_start_time > 10.0:  # Timeout after 10 seconds
            print("⏰ Stage 5 timeout, stopping")
            tx("STOP")
            print("🏁 Finished")
            break
        print("⏳ Stage 5: Requesting CURRENT_YAW")
        tx("GET_CURRENT_YAW")  # Explicitly request CURRENT_YAW
        yaw_now = wait_token("CURRENT_YAW", tout=3.0)
        if yaw_now is None:
            print("⚠️ No CURRENT_YAW received, retrying...")
            continue
        error = angle_diff(start_yaw + 90, yaw_now)
        print(f"📐 Final yaw error = {error:.1f}")
        if abs(error) < 8:
            tx("STOP")
            print("🏁 Finished")
            break
        else:
            print("⚠️ Yaw error too large, retrying TURN_TO")
            tx(f"TURN_TO:{(start_yaw + 90) % 360:.1f}")
            time.sleep(0.5)

    # ---------- GUI ----------
    if SHOW_GUI:
        combined = cv2.bitwise_or(
            cv2.bitwise_and(roi, roi, mask=omask),
            cv2.bitwise_and(roi, roi, mask=bmask)
        )
        cv2.imshow("ROI Detection", combined)
        cv2.imshow("Frame", frame)
        cv2.imshow("🟠 Mask", omask)
        cv2.imshow("🔵 Mask", bmask)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# ---------- Cleanup ----------
tx("STOP")
camera.release()
if SHOW_GUI:
    cv2.destroyAllWindows()
ser.close()
