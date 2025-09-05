import serial
import time

# افتح السيريال مع الـ ESP32
ser = serial.Serial('/dev/ttyTHS1', 115200)
time.sleep(2)  # استنى شويه للتأكد من الاتصال

# ابعت أمر يمين
ser.write(b'RIGHT\n')
print("✅ Command sent: RIGHT")
time.sleep(2)

# ابعت أمر شمال
ser.write(b'LEFT\n')
print("✅ Command sent: LEFT")
time.sleep(2)

# ابعت أمر مستقيم
ser.write(b'STRAIGHT\n')
print("✅ Command sent: STRAIGHT")

