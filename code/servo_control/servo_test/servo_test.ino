
// code for testing servo motor steering in loop 


#include <ESP32Servo.h>

Servo steeringServo;

#define SERVO_PIN 13

// زوايا الستيرنج حسب الروبوت بتاعك
#define ANGLE_LEFT   30
#define ANGLE_CENTER 88
#define ANGLE_RIGHT  150

void setup() {
  Serial.begin(115200);
  steeringServo.attach(SERVO_PIN);
  Serial.println("Starting Steering Servo Test...");
  delay(1000);  // تأخير مبدئي بسيط
}

void loop() {
  // 1. توجيه يمين
  steeringServo.write(ANGLE_RIGHT);
  Serial.println("Steering → Right");
  delay(1000);  // انتظر 1 ثانية

  // 2. توجيه منتصف
  steeringServo.write(ANGLE_CENTER);
  Serial.println("Steering → Center");
  delay(1000);

  // 3. توجيه شمال
  steeringServo.write(ANGLE_LEFT);
  Serial.println("Steering → Left");
  delay(1000);

  // 4. توجيه منتصف
  steeringServo.write(ANGLE_CENTER);
  Serial.println("Steering → Center");
  delay(1000);
}
