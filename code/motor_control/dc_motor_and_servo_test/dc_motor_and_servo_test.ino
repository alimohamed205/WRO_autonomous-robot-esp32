#include <Arduino.h>
#include <ESP32Servo.h>
#define IN1 25
#define IN2 26
#define ENA 27
#define SERVO_PIN 14

Servo myServo;

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
   ledcAttachChannel(ENA, 1000, 8, 2); // استخدمنا ledcAttach
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  ledcWrite(ENA, 180);
    delay(500);
  ledcWrite(ENA, 100);


  Serial.println("✅ الموتور DC شغال لقدام بسرعة 100");
  myServo.attach(SERVO_PIN);
  myServo.write(90);
  Serial.println("✅ السيرفو شغال على 90 درجة");
}

void loop() {
  
 myServo.write(0);
  Serial.println("السيرفو على 0 درجة");
  delay(1000);
  myServo.write(90);
  Serial.println("السيرفو على 90 درجة");
  delay(1000);
  myServo.write(0);
  Serial.println("السيرفو على 180 درجة");
  delay(1000);
}
