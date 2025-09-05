```python
import serial
import time

# افتح السيريال مع الـ ESP32
ser = serial.Serial('/dev/ttyTHS1', 115200)
time.sleep(2)  # استنى للتأكد من الاتصال

while True:
    # ابعت أمر يمين
    ser.write(b'RIGHT\n')
    print("✅ Command sent: RIGHT")
    time.sleep(2)

    # ابعت أمر شمال
    ser.write(b'LEFT\n')
    print("✅ Command sent: LEFT")
    time.sleep(2)

    # ابعت أمر وسط
    ser.write(b'CENTER\n')
    print("✅ Command sent: CENTER")
    time.sleep(2)

    # ابعت أمر لقدام
    ser.write(b'FORWARD\n')
    print("✅ Command sent: FORWARD")
    time.sleep(2)

    # ابعت أمر توقف
    ser.write(b'STOP\n')
    print("✅ Command sent: STOP")
    time.sleep(2)
```
