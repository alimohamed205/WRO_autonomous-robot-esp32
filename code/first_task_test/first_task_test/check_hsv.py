import cv2
def nothing(x): pass
cap = cv2.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=640, height=480, framerate=30/1 ! nvvidconv ! video/x-raw, format=BGR ! appsink", cv2.CAP_GSTREAMER)
cv2.namedWindow("Trackbars")
cv2.createTrackbar("H Min", "Trackbars", 0, 179, nothing)
cv2.createTrackbar("H Max", "Trackbars", 179, 179, nothing)
cv2.createTrackbar("S Min", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("S Max", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("V Min", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("V Max", "Trackbars", 255, 255, nothing)
while True:
    ret, frame = cap.read()
    if not ret: break
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h_min = cv2.getTrackbarPos("H Min", "Trackbars")
    h_max = cv2.getTrackbarPos("H Max", "Trackbars")
    s_min = cv2.getTrackbarPos("S Min", "Trackbars")
    s_max = cv2.getTrackbarPos("S Max", "Trackbars")
    v_min = cv2.getTrackbarPos("V Min", "Trackbars")
    v_max = cv2.getTrackbarPos("V Max", "Trackbars")
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(frame, frame, mask=mask)
    cv2.imshow("Mask", mask)
    cv2.imshow("Result", result)
    if cv2.waitKey(1) & 0xFF == ord('q'): break
cap.release()
cv2.destroyAllWindows()
