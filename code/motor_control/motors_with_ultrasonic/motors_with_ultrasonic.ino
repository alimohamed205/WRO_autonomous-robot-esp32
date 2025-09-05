#include <ESP32Servo.h>

Servo steeringServo;  // السيرفو للتوجيه
Servo esc;            // ESC للموتور

#define SERVO_PIN 13   // البن المتوصل بالـ Servo
#define ESC_PIN   12   // البن المتوصل بالـ ESC

// زوايا التوجيه (عدّل حسب الروبوت بتاعك)
#define ANGLE_LEFT   50
#define ANGLE_CENTER 88
#define ANGLE_RIGHT  130

// قيم السرعة
#define PWM_NEUTRAL 1500
#define PWM_FORWARD 1600

void setup() {
  Serial.begin(115200);

  // تهيئة السيرفو والـ ESC
  steeringServo.attach(SERVO_PIN);
  esc.attach(ESC_PIN);

  Serial.println("Initializing ESC...");
  esc.writeMicroseconds(PWM_NEUTRAL);
  delay(3000);  // فترة تهيئة الـ ESC

  Serial.println("Setup Complete.");
}

void moveForward(int pwmVal) {
  esc.writeMicroseconds(pwmVal);
  Serial.print("Moving Forward → PWM: ");
  Serial.println(pwmVal);
}

void stopMotor() {
  esc.writeMicroseconds(PWM_NEUTRAL);
  Serial.println("Motor Stopped.");
}

void loop() {
  // توجيه يمين
  steeringServo.write(ANGLE_RIGHT);
  Serial.println("Steering → Right");
  delay(500);

  moveForward(PWM_FORWARD);
  delay(1500);

  stopMotor();
  delay(1000);

  // توجيه شمال
  steeringServo.write(ANGLE_LEFT);
  Serial.println("Steering → Left");
  delay(500);

  moveForward(PWM_FORWARD);
  delay(1500);

  stopMotor();
  delay(1000);

  // توجيه في المنتصف
  steeringServo.write(ANGLE_CENTER);
  Serial.println("Steering → Center");
  delay(500);

  moveForward(PWM_FORWARD);
  delay(2000);

  stopMotor();
  delay(2000);
}
