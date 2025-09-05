import cv2
import numpy as np

# GStreamer pipeline for Jetson IMX219 camera
gst_pipeline = (
    "nvarguscamerasrc ! "
    "video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! "
    "nvvidconv flip-method=0 ! "
    "video/x-raw, format=BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! appsink"
)

cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("❌ فشل في فتح الكاميرا")
    exit()

# حدود اللون البرتقالي
lower_orange = np.array([5, 120, 110])
upper_orange = np.array([18, 230, 180])

# حدود اللون الأزرق
lower_blue = np.array([110, 50, 50])
upper_blue = np.array([135, 255, 255])

kernel = np.ones((5, 5), np.uint8)

while True:
    ret, frame = cap.read()
    if not ret:
        print("⚠️ فشل في قراءة الإطار")
        break

    height, width, _ = frame.shape

    # تحديد منطقة ROI حسب طلبك
    start_y = int(height * 0.4)
    end_y = int(height * 0.80)
    start_x = int(width * 0.1)
    end_x = int(width * 1.0)

    roi = frame[start_y:end_y, start_x:end_x]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # الماسكات
    mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # تحسين الماسك
    mask_orange = cv2.erode(mask_orange, kernel, iterations=1)
    mask_orange = cv2.dilate(mask_orange, kernel, iterations=1)
    mask_blue = cv2.erode(mask_blue, kernel, iterations=1)
    mask_blue = cv2.dilate(mask_blue, kernel, iterations=1)

    # دمج الماسكات
    combined_mask = cv2.bitwise_or(mask_orange, mask_blue)

    # رسم الـ ROI على الصورة الأصلية
    cv2.rectangle(frame, (start_x, start_y), (end_x, end_y), (0, 255, 0), 2)

    # حساب النسب
    orange_pixels = cv2.countNonZero(mask_orange)
    blue_pixels = cv2.countNonZero(mask_blue)
    total_pixels = mask_orange.shape[0] * mask_orange.shape[1]
    orange_ratio = (orange_pixels / total_pixels) * 100
    blue_ratio = (blue_pixels / total_pixels) * 100

    # عرض النسب
    text = f"Orange: {orange_ratio:.2f}% | Blue: {blue_ratio:.2f}%"
    cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    # عرض الألوان المكتشفة
    detected_colors = cv2.bitwise_and(roi, roi, mask=combined_mask)

    # العرض
    cv2.imshow("📷 Original", frame)
    cv2.imshow("🎯 Mask", combined_mask)
    cv2.imshow("🎨 Detected Colors", detected_colors)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
