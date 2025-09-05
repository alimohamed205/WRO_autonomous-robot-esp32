import serial
import time

# فتح السيريال مع الـ ESP32
ser = serial.Serial('/dev/ttyTHS1', 115200)
time.sleep(2)  # استني للتأكد من الاتصال

try:
    while True:
        # ادخل خطأ وهمي يدويًا
        error = float(input("ادخل الخطأ (مثلًا -100 إلى 100): "))
        
        # ابعت الأمر للـ ESP32
        command = f"E:{error}\n"
        ser.write(command.encode())
        print(f"✅ Command sent: {command.strip()}")
        
        # تأخير صغير
        time.sleep(0.5)
except KeyboardInterrupt:
    print("⏹️ تم إيقاف البرنامج")
    # أمر توقف للـ ESC
    ser.write(b"STOP\n")
    ser.close()
