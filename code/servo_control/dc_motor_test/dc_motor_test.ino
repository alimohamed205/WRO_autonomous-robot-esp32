#include <ESP32Servo.h>

Servo esc; // متغير لربط ESC

#define ESC_PIN 12  // البن اللي متوصل عليه الإشارة من ESP32

void setup() {
  Serial.begin(115200);

  esc.attach(ESC_PIN);
  Serial.println("Starting DC Motor Test (via ESC)...");

  // تهيئة الـ ESC (Neutral)
  esc.writeMicroseconds(1500); // الوضع المحايد (الموتور واقف)
  delay(3000); // فترة انتظار لتفعيل الـ ESC
}

void loop() {
  // 1. تشغيل الموتور للأمام بسرعة منخفضة
  esc.writeMicroseconds(1600);
  Serial.println("Motor: Forward (Low Speed)");
  delay(2000);

  // 2. تشغيل الموتور بسرعة أعلى
  esc.writeMicroseconds(1700);
  Serial.println("Motor: Forward (Medium Speed)");
  delay(2000);

  // 3. إيقاف الموتور (Neutral)
  esc.writeMicroseconds(1500);
  Serial.println("Motor: Stop");
  delay(2000);

  // 4. حركة للخلف (لو مدعوم من ESC)
  esc.writeMicroseconds(1400);
  Serial.println("Motor: Reverse (Low)");
  delay(2000);

  // 5. إيقاف الموتور تاني
  esc.writeMicroseconds(1500);
  Serial.println("Motor: Stop");
  delay(2000);
}
