ذ#include <Arduino.h>
#include <ESP32Servo.h>

#define IN1 25
#define IN2 26
#define ENA 27
#define SERVO_PIN 14
#define RX_PIN 16
#define TX_PIN 17

Servo myServo;

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
  myServo.write(90); // السيرفو في المنتصف
  Serial.println("✅ النظام جاهز");
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
      myServo.write(50); // يسار
      Serial.println("↩️ السيرفو يسار");
    } else if (cmd == "RIGHT") {
      ledcWrite(ENA, 0); // توقف الموتور عند التوجيه
      myServo.write(130); // يمين
      Serial.println("↪️ السيرفو يمين");
    } else if (cmd == "CENTER") {
      ledcWrite(ENA, 0); // توقف الموتور عند التوجيه
      myServo.write(90); // وسط
      Serial.println("🎯 السيرفو في المنتصف");
    } else {
      Serial.println("❓ أمر مش معروف");
    }
  }
}

