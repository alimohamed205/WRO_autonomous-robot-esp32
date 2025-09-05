#!/usr/bin/env python3
"""
WRO 2025 - Professional Navigation System (Competition Grade)
------------------------------------------------------------
Key Improvements:
1. Dynamic turn direction handling per competition rules
2. Accurate section/lap counting
3. Start section detection for final stop
4. Enhanced color processing
5. Rule-compliant direction handling
"""

import cv2
import numpy as np
import serial
import threading
import queue
import time
import sys
import argparse
import logging
from datetime import datetime
from collections import deque
from enum import Enum

# ---------- CONFIGURATION ----------
SERIAL_PORT = '/dev/ttyTHS1'
BAUD_RATE = 115200
FRAME_W, FRAME_H = 640, 360
STEER_INTERVAL = 0.05
MIN_STEER = 60
MAX_STEER = 120
ALIVE_S = 1.0
THRESH_PIXELS = 100
BLUE_CONFIDENCE_THRESHOLD = 3
STEER_INVERT = False
TURN_TIMEOUT = 3.0
TURN_COMPLETE_THRESHOLD = 5.0
SECTIONS_PER_LAP = 8
TOTAL_LAPS = 3

# Navigation states
class RobotState(Enum):
    WALL_TRACKING = 0
    TURNING = 1
    LAP_COMPLETE = 2
    STOPPED = 3
    FINDING_START = 4

# Wall Detection Parameters
WALL_DETECTION_HEIGHT = 0.4
EDGE_THRESHOLD1 = 50
EDGE_THRESHOLD2 = 150
MIN_WALL_AREA = 1000

# Perspective Transform
PERSPECTIVE_SRC = np.float32([[120, 80], [520, 80], [30, 280], [610, 280]])
PERSPECTIVE_DST = np.float32([[0, 0], [640, 0], [0, 360], [640, 360]])
M = cv2.getPerspectiveTransform(PERSPECTIVE_SRC, PERSPECTIVE_DST)

# PID Controller
STEER_KP = 0.02
STEER_KI = 0.001
STEER_KD = 0.005

# HSV Ranges
HSV_ORANGE = (np.array([5, 120, 110]), np.array([15, 230, 180]))
HSV_BLUE   = (np.array([110, 50, 50]), np.array([135, 255, 255]))
BASE_HSV_WALL = (np.array([0, 0, 0]), np.array([180, 255, 60]))

# ---------- LOGGING SETUP ----------
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler(f'wro_advanced_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log')
    ]
)
logger = logging.getLogger(__name__)

# ---------- UTILITY FUNCTIONS ----------
def is_valid_image(img):
    return img is not None and img.size > 0 and img.shape[0] > 0 and img.shape[1] > 0

def safe_imshow(window_name, img):
    if is_valid_image(img):
        cv2.imshow(window_name, img)

def extract_roi(frame):
    height, width = frame.shape[:2]
    roi_w = int(width * 0.2)
    roi_h = int(height * 0.2)
    roi_cx = int(width * 0.50)
    roi_cy = int(height * 0.50)
    angle = 0
    rect = ((roi_cx, roi_cy), (roi_w, roi_h), angle)
    box = cv2.boxPoints(rect).astype(np.int32)
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillConvexPoly(mask, box, 255)

    roi_colored = cv2.bitwise_and(frame, frame, mask=mask)
    hsv = cv2.cvtColor(roi_colored, cv2.COLOR_BGR2HSV)
    cv2.polylines(frame, [box], isClosed=True, color=(0, 255, 0), thickness=2)

    return roi_colored, hsv, box

def detect_color(hsv, lo, hi):
    mask = cv2.inRange(hsv, lo, hi)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    return mask, cv2.countNonZero(mask)

# ---------- SECTION TRACKER CLASS ----------
class SectionTracker:
    def __init__(self):
        self.section_count = 0
        self.lap_count = 0
        self.initial_yaw = None
        self.start_section_detected = False
        self.start_section_features = None
        
    def update_on_turn(self):
        self.section_count += 1
        if self.section_count % SECTIONS_PER_LAP == 0:
            self.lap_count += 1
            logger.info(f"Lap completed: {self.lap_count}/{TOTAL_LAPS}")
            
    def should_stop(self):
        return self.lap_count >= TOTAL_LAPS
    
    def test_start_section_capture(self, tracker):
       dummy = np.zeros((100, 100, 3), dtype=np.uint8)
       tracker.capture_start_section(dummy)
       assert tracker.start_section_features is not None
       assert tracker.start_section_features.shape == (256,)
        
    def capture_start_section(self, frame):
        """Extract features from start section for later recognition"""
        if frame is None:
            return
            
        # Use simple color histogram as placeholder
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hist = cv2.calcHist([gray], [0], None, [256], [0, 256])
        self.start_section_features = hist.flatten()
        logger.info("Start section features captured")

# ---------- CAMERA CLASS ----------
class Camera:
    def __init__(self):
        self.cap = self.initialize_camera()
        self.lock = threading.Lock()
        self.latest_frame = None
        self.running = True
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def initialize_camera(self):
        sources = [
            ("nvarguscamerasrc sensor-id=0 ! "
             "video/x-raw(memory:NVMM),width=640,height=480,framerate=30/1 ! "
             "nvvidconv flip-method=0 ! "
             "video/x-raw,width=640,height=360,format=BGRx ! "
             "videoconvert ! video/x-raw,format=BGR ! appsink", cv2.CAP_GSTREAMER),
            (0, cv2.CAP_V4L2),
            (0, cv2.CAP_ANY)
        ]
        
        for source, backend in sources:
            try:
                cap = cv2.VideoCapture(source, backend)
                if cap.isOpened():
                    ret, frame = cap.read()
                    if ret and is_valid_image(frame):
                        logger.info(f"Camera initialized: {source}")
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
                        return cap
                    else:
                        cap.release()
            except:
                continue
        
        logger.error("Camera initialization failed")
        sys.exit(1)

    def _reader(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret and is_valid_image(frame):
                with self.lock:
                    self.latest_frame = frame
            time.sleep(0.001)

    def read(self):
        with self.lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None

    def release(self):
        self.running = False
        self.thread.join(timeout=0.5)
        self.cap.release()
        logger.info("Camera released")

# ---------- SERIAL COMMUNICATION ----------
class SerialComm:
    def __init__(self, port, baud):
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            logger.info(f"Serial port {port} initialized")
            self.rx_q = queue.Queue()
            self.thread = threading.Thread(target=self._reader, daemon=True)
            self.thread.start()
        except serial.SerialException as e:
            logger.warning(f"Serial error: {e} - Simulation mode")
            self.ser = None

    def _reader(self):
        if not self.ser:
            return
            
        buf = ''
        while True:
            try:
                if self.ser.in_waiting:
                    buf += self.ser.read(self.ser.in_waiting).decode(errors='ignore')
                    while '\n' in buf:
                        line, buf = buf.split('\n', 1)
                        self.rx_q.put(line.strip())
                time.sleep(0.005)
            except Exception as e:
                logger.error(f"Serial read error: {e}")
                break

    def tx(self, cmd):
        if self.ser:
            try:
                self.ser.write((cmd + '\n').encode())
                logger.info(f"TX: {cmd}")
            except Exception as e:
                logger.error(f"Serial write error: {e}")
        else:
            logger.info(f"SIM TX: {cmd}")

    def wait_token(self, token, timeout=1.5):
        if not self.ser:
            return 0.0
            
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                ln = self.rx_q.get_nowait()
                if ln.startswith(token + ':'):
                    return float(ln.split(':', 1)[1])
                if ln.strip() == token:
                    return True
            except queue.Empty:
                time.sleep(0.01)
        logger.warning(f"Timeout waiting for {token}")
        return None

    def close(self):
        if self.ser:
            self.ser.close()
        logger.info("Serial port closed")

# ---------- ROBUST WALL TRACKER ----------
class RobustWallTracker:
    def __init__(self):
        self.adaptive_v_threshold = 50
        self.wall_history = deque(maxlen=10)
        self.fallback_active = False
        self.detection_stats = {"advanced": 0, "fallback": 0}
        
    def adaptive_hsv_range(self, frame):
        if not is_valid_image(frame):
            return BASE_HSV_WALL
            
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        brightness = np.median(gray)
        
        if brightness < 50:
            self.adaptive_v_threshold = 70
        elif brightness > 200:
            self.adaptive_v_threshold = 30
        else:
            self.adaptive_v_threshold = 50
            
        return (BASE_HSV_WALL[0], 
                np.array([180, 255, self.adaptive_v_threshold]))

    def detect_walls(self, frame):
        if not is_valid_image(frame):
            return 0.0, np.zeros((FRAME_H, FRAME_W), dtype=np.uint8), np.zeros((FRAME_H, FRAME_W), dtype=np.uint8)
        
        try:
            hsv_range = self.adaptive_hsv_range(frame)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            color_mask = cv2.inRange(hsv, *hsv_range)
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blurred, EDGE_THRESHOLD1, EDGE_THRESHOLD2)
            
            filtered_edges = cv2.bitwise_and(edges, edges, mask=color_mask)
            combined_mask = cv2.bitwise_or(color_mask, filtered_edges)

            contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            filtered_combined_mask = np.zeros_like(combined_mask)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > MIN_WALL_AREA:
                    cv2.drawContours(filtered_combined_mask, [cnt], -1, 255, cv2.FILLED)
            combined_mask = filtered_combined_mask

            warped = cv2.warpPerspective(combined_mask, M, (FRAME_W, FRAME_H))
            
            height, width = warped.shape
            roi_height = int(height * WALL_DETECTION_HEIGHT)
            roi = warped[height - roi_height:height, :]
            
            contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            valid_walls = []
            brightness = np.median(gray)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < MIN_WALL_AREA:
                    continue
                    
                rect = cv2.minAreaRect(cnt)
                (x, y), (w, h), angle = rect
                
                aspect_threshold = max(0.15, 0.3 - (brightness/255)*0.15)
                aspect_ratio = min(w, h) / max(w, h) if max(w, h) > 0 else 0
                
                if aspect_ratio > aspect_threshold:
                    valid_walls.append((x, y, w, h))
            
            if len(valid_walls) >= 2:
                self.fallback_active = False
                self.detection_stats["advanced"] += 1
                
                valid_walls.sort(key=lambda p: p[0])
                left_wall = valid_walls[0]
                right_wall = valid_walls[-1]
                
                center = (left_wall[0] + right_wall[0]) / 2
                error = center - (width / 2)
                
                self.wall_history.append(error)
                filtered_error = np.median(list(self.wall_history)[-3:]) if len(self.wall_history) >= 3 else error
                
                return filtered_error, combined_mask, warped
            else:
                self.fallback_active = True
                self.detection_stats["fallback"] += 1
                return self.enhanced_fallback(roi, brightness), combined_mask, warped
                
        except Exception as e:
            logger.error(f"Wall detection error: {e}")
            return 0.0, np.zeros((FRAME_H, FRAME_W), dtype=np.uint8), np.zeros((FRAME_H, FRAME_W), dtype=np.uint8)

    def enhanced_fallback(self, roi, brightness):
        if not is_valid_image(roi):
            return 0.0
            
        height, width = roi.shape
        roi_height_factor = 0.3 + (brightness/255 * 0.2)
        roi_height = int(height * roi_height_factor)
        
        left_width = int(width * (0.25 + (1 - brightness/255)*0.1))
        left_roi = roi[-roi_height:, :left_width]
        
        right_start = width - int(width * (0.25 + (1 - brightness/255)*0.1))
        right_roi = roi[-roi_height:, right_start:]
        
        left_px = cv2.countNonZero(left_roi)
        right_px = cv2.countNonZero(right_roi)
        
        raw_error = left_px - right_px
        weight = 0.7 if abs(raw_error) > 500 else 0.3
        weighted_error = raw_error * weight
        
        if len(self.wall_history) > 0:
            filtered_error = (0.5 * weighted_error + 
                             0.3 * self.wall_history[-1] + 
                             0.2 * self.wall_history[-2] if len(self.wall_history) > 1 else weighted_error)
            self.wall_history.append(filtered_error)
            return filtered_error
        else:
            self.wall_history.append(weighted_error)
            return weighted_error

# ---------- DYNAMIC DIRECTION HANDLER ----------
class DirectionHandler:
    def __init__(self):
        self.round_direction = None  # 'cw' or 'ccw'
        self.last_orange_pos = None
        self.last_blue_pos = None
        
    def determine_turn_direction(self, orange_pos, blue_pos):
        """Determine turn direction based on relative position of colors"""
        if orange_pos is None or blue_pos is None:
            return None
            
        # Calculate horizontal positions
        orange_x = orange_pos[0]
        blue_x = blue_pos[0]
        
        # Rule: Orange on left = CW, Blue on left = CCW
        if orange_x < blue_x:
            return 'cw'
        else:
            return 'ccw'
            
    def update_color_positions(self, frame, omask, bmask):
        """Update positions of detected colors"""
        self.last_orange_pos = self.find_centroid(omask, frame)
        self.last_blue_pos = self.find_centroid(bmask, frame)
        
        if self.last_orange_pos and self.last_blue_pos:
            self.round_direction = self.determine_turn_direction(
                self.last_orange_pos, self.last_blue_pos
            )
            
    def find_centroid(self, mask, frame):
        """Find centroid of largest contour in mask"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
            
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return None
            
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return (cX, cY)

# ---------- STABLE NAVIGATION CONTROLLER ----------
class StableNavigationController:
    def __init__(self):
        self.pid_integral = 0
        self.last_error = 0
        self.last_time = time.time()
        self.oscillation_count = 0
        self.last_steer = 90.0
        
    def update(self, error):
        now = time.time()
        dt = now - self.last_time
        
        if dt > 0 and abs(error - self.last_error) / dt > 50:
            self.oscillation_count += 1
        else:
            self.oscillation_count = max(0, self.oscillation_count - 1)
            
        kd_multiplier = 1.0 + min(3.0, self.oscillation_count * 0.5)
        
        p = STEER_KP * error
        self.pid_integral += error * dt
        i = STEER_KI * self.pid_integral
        d = STEER_KD * kd_multiplier * (error - self.last_error) / max(0.01, dt)
        
        steer_adjust = p + i + d
        
        if self.oscillation_count > 2:
            steer_adjust = 0.7 * steer_adjust + 0.3 * (self.last_steer - 90)
        
        self.last_error = error
        self.last_time = now
        self.last_steer = 90 + steer_adjust
        
        return steer_adjust

# ---------- MAIN APPLICATION ----------
def main():
    parser = argparse.ArgumentParser(description="WRO 2025 Advanced Navigation")
    parser.add_argument("--show-gui", action="store_true", help="Enable GUI display")
    parser.add_argument("--log-video", type=str, help="Save video output to file")
    args = parser.parse_args()

    # Initialize system
    camera = Camera()
    comm = SerialComm(SERIAL_PORT, BAUD_RATE)
    wall_tracker = RobustWallTracker()
    nav_controller = StableNavigationController()
    section_tracker = SectionTracker()
    direction_handler = DirectionHandler()
    video_writer = None
    
    # Initialize GUI
    if args.show_gui:
        cv2.namedWindow("Main View", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Main View", 800, 450)
        cv2.namedWindow("Processing", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Processing", 800, 300)
        cv2.namedWindow("Warped View", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Warped View", 800, 300)
        cv2.namedWindow("Color ROI", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Color ROI", 800, 300)

    # Get initial yaw
    comm.tx("GET_INITIAL_YAW")
    start_yaw = comm.wait_token("INITIAL_YAW") or 0
    logger.info(f"Initial Yaw: {start_yaw:.1f}°")
    section_tracker.initial_yaw = start_yaw

    # Main loop variables
    current_state = RobotState.WALL_TRACKING
    last_steer_time = 0
    blue_confidence = 0
    current_yaw = start_yaw
    turn_start_time = 0
    target_turn_yaw = 0

    try:
        while True:
            # Get frame
            frame = camera.read()
            if frame is None:
                continue
                
            roi, hsv, box = extract_roi(frame)    
            
            # Process frame
            error, combined_mask, warped = wall_tracker.detect_walls(frame)
            omask, opx = detect_color(hsv, *HSV_ORANGE)
            bmask, bpx = detect_color(hsv, *HSV_BLUE)
            has_orange = opx > THRESH_PIXELS
            has_blue = bpx > THRESH_PIXELS
            
            # Update direction handler with color positions
            direction_handler.update_color_positions(frame, omask, bmask)
            
            # State machine
            now = time.time()
            
            if current_state == RobotState.WALL_TRACKING:
                # Capture start section features at beginning
                if section_tracker.section_count == 0 and not section_tracker.start_section_features:
                    section_tracker.capture_start_section(frame)
                
                # Navigation control
                if now - last_steer_time > STEER_INTERVAL:
                    comm.tx("GET_CURRENT_YAW")
                    current_yaw = comm.wait_token("CURRENT_YAW") or current_yaw
                    
                    steer_correction = nav_controller.update(error)
                    steer_angle = 90 + steer_correction
                    steer_angle = max(MIN_STEER, min(MAX_STEER, steer_angle))
                    
                    if STEER_INVERT:
                        steer_angle = 180 - steer_angle
                        
                    comm.tx(f"STEER:{steer_angle:.1f}")
                    last_steer_time = now
                    comm.tx("FORWARD")
                
                # Turn logic
                if direction_handler.round_direction:
                    turn_required = False
                    
                    if direction_handler.round_direction == 'cw' and has_orange:
                        logger.info("Orange detected. Initiating CW turn.")
                        turn_required = True
                    elif direction_handler.round_direction == 'ccw' and has_blue:
                        logger.info("Blue detected. Initiating CCW turn.")
                        turn_required = True
                    
                    if turn_required:
                        target_turn_yaw = (current_yaw + 90) % 360 if direction_handler.round_direction == 'cw' else (current_yaw - 90) % 360
                        comm.tx(f"TURN_TO:{target_turn_yaw:.1f}")
                        turn_start_time = now
                        current_state = RobotState.TURNING

            elif current_state == RobotState.TURNING:
                comm.tx("GET_CURRENT_YAW")
                current_yaw = comm.wait_token("CURRENT_YAW") or current_yaw
                
                yaw_diff = abs((current_yaw - target_turn_yaw + 180) % 360 - 180)
                
                if yaw_diff < TURN_COMPLETE_THRESHOLD:
                    logger.info("Turn complete. Resuming wall tracking.")
                    section_tracker.update_on_turn()
                    
                    # Check if we should stop after completing laps
                    if section_tracker.should_stop():
                        current_state = RobotState.FINDING_START
                        logger.info("All laps completed. Finding start section.")
                    else:
                        current_state = RobotState.WALL_TRACKING
                        comm.tx("FORWARD")
                elif now - turn_start_time > TURN_TIMEOUT:
                    logger.warning("Turn timed out. Resuming wall tracking.")
                    section_tracker.update_on_turn()
                    current_state = RobotState.WALL_TRACKING
                    comm.tx("FORWARD")

            elif current_state == RobotState.FINDING_START:
                # Check if we're in the start section
                if section_tracker.is_in_start_section(frame):
                    comm.tx("STOP")
                    current_state = RobotState.STOPPED
                    logger.info("Start section reached. Stopping robot.")
                else:
                    # Simple homing behavior (enhance with visual navigation)
                    comm.tx("FORWARD_SLOW")
                    
            elif current_state == RobotState.STOPPED:
                # Competition complete
                break

            # GUI display
            if args.show_gui:
                if is_valid_image(frame):
                    display_frame = frame.copy()
                    cv2.polylines(display_frame, [box], isClosed=True, color=(0, 255, 0), thickness=2)
                    
                    # Display navigation info
                    state_info = f"State: {current_state.name}"
                    section_info = f"Sections: {section_tracker.section_count} (Lap {section_tracker.lap_count+1}/{TOTAL_LAPS})"
                    direction_info = f"Direction: {direction_handler.round_direction or 'Unknown'}"
                    color_info = f"Orange: {has_orange}, Blue: {has_blue}"
                    
                    cv2.putText(display_frame, state_info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(display_frame, section_info, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(display_frame, direction_info, (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(display_frame, color_info, (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
                    # Show color positions if available
                    if direction_handler.last_orange_pos:
                        cv2.circle(display_frame, direction_handler.last_orange_pos, 10, (0, 165, 255), -1)
                    if direction_handler.last_blue_pos:
                        cv2.circle(display_frame, direction_handler.last_blue_pos, 10, (255, 0, 0), -1)
                        
                    safe_imshow("Main View", display_frame)

                safe_imshow("Processing", combined_mask)
                safe_imshow("Warped View", warped)
                safe_imshow("Color ROI", omask + bmask)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    except KeyboardInterrupt:
        logger.info("Program interrupted by user.")
    except Exception as e:
        logger.error(f"An unexpected error occurred: {e}")
    finally:
        logger.info("Cleaning up...")
        comm.tx("STOP")
        time.sleep(0.1)
        comm.close()
        camera.release()
        if video_writer:
            video_writer.release()
        if args.show_gui:
            cv2.destroyAllWindows()
        logger.info("Cleanup complete.")

if __name__ == "__main__":
    main()