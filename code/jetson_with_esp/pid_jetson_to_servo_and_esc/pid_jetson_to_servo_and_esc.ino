#include <Arduino.h>
#include <ESP32Servo.h>

#define IN1 25
#define IN2 26
#define ENA 27
#define SERVO_PIN 14
#define RX_PIN 16
#define TX_PIN 17

Servo myServo;

// متغيرات PID
float setpoint = 90; // الزاوية المطلوبة (default: وسط)
float currentAngle = 90; // الزاوية الحالية
float error = 0, prevError = 0, integral = 0, derivative = 0;
float Kp = 0.7, Ki = 0.002, Kd = 0.15; // معاملات PID للحركة السلسة
float output = 0;
unsigned long lastTime = 0;
float dt = 0.03; // فترة التحديث (30ms)
float integralLimit = 50.0; // حد أقصى للـ integral

void setup() {
  Serial.begin(115200);  // للمراقبة
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);  // مع Jetson Nano
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  ledcAttachChannel(ENA, 1000, 8, 2); // قناة 2 لتجنب التداخل
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  ledcWrite(ENA, 0); // توقف في البداية
  myServo.attach(SERVO_PIN);
  myServo.write(90); // ضبط السيرفو في المنتصف
  delay(3000); // تأخير لاستقرار السيرفو
  Serial.println("✅ النظام جاهز");
}

void updatePID() {
  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0; // الوقت بالثواني
  if (dt < 0.03) return; // تحديث كل 30ms فقط
  lastTime = now;

  error = setpoint - currentAngle; // الخطأ
  integral += error * dt; // التكامل
  // حد أقصى للـ integral
  if (integral > integralLimit) integral = integralLimit;
  if (integral < -integralLimit) integral = -integralLimit;
  derivative = (error - prevError) / dt; // المشتق
  output = Kp * error + Ki * integral + Kd * derivative; // إشارة التحكم

  // تحديث الزاوية الحالية
  currentAngle += output * dt;
  if (currentAngle < 0) currentAngle = 0;
  if (currentAngle > 180) currentAngle = 180;

  myServo.write((int)currentAngle); // تحريك السيرفو
  prevError = error;

  // طباعة للتصحيح
  Serial.print("زاوية الحالية: ");
  Serial.println(currentAngle);
}

void loop() {
  if (Serial1.available()) {
    String cmd = Serial1.readStringUntil('\n');
    cmd.trim();
    Serial.print("📥 الأمر اللي اتبعت: "); Serial.println(cmd);

    if (cmd == "FORWARD") {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      ledcWrite(ENA, 180); // بداية بسرعة عالية
      delay(500);
      ledcWrite(ENA, 100); // سرعة 100/255
      Serial.println("🏎️ الموتور لقدام");
    } else if (cmd == "STOP") {
      ledcWrite(ENA, 0);
      Serial.println("⛔ الموتور واقف");
    } else if (cmd == "LEFT") {
      ledcWrite(ENA, 0); // توقف الموتور عند التوجيه
      setpoint = 30; // زاوية 30 درجة (يسار)
      Serial.println("↩️ السيرفو يسار");
    } else if (cmd == "RIGHT") {
      ledcWrite(ENA, 0); // توقف الموتور عند التوجيه
      setpoint = 150; // زاوية 150 درجة (يمين)
      Serial.println("↪️ السيرفو يمين");
    } else if (cmd == "CENTER") {
      ledcWrite(ENA, 0); // توقف الموتور عند التوجيه
      setpoint = 90; // زاوية 90 درجة (منتصف)
      Serial.println("🎯 السيرفو في المنتصف");
    } else {
      Serial.println("❓ أمر مش معروف");
    }
  }

  // تحديث PID للسيرفو
  updatePID();
}
