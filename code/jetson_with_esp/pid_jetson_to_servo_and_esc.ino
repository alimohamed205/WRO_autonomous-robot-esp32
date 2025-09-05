#include <ESP32Servo.h>

Servo steeringServo;  // سيرفو التوجيه
Servo esc;            // الموتور عبر ESC

#define SERVO_PIN 13   // بن السيرفو
#define ESC_PIN   12   // بن الـ ESC

#define ANGLE_LEFT   50
#define ANGLE_CENTER 88
#define ANGLE_RIGHT  130
#define PWM_NEUTRAL 1500
#define PWM_FORWARD 1600

// معاملات PID
float Kp = 0.5;  // معامل التناسب
float Ki = 0.01; // معامل التكامل
float Kd = 0.1;  // معامل التفاضل

float integral = 0;
float previous_error = 0;
unsigned long last_time = 0;

void setup() {
  Serial.begin(115200);                            // للمراقبة التسلسلية
  Serial1.begin(115200, SERIAL_8N1, 16, 17);       // التواصل مع الجيتسون Nano
  steeringServo.attach(SERVO_PIN);
  esc.attach(ESC_PIN);

  Serial.println("🔧 Initializing ESC...");
  esc.writeMicroseconds(PWM_NEUTRAL);
  delay(3000); // Arm time

  Serial.println("✅ Setup Complete.");
}

float pid_controller(float error) {
  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0; // الزمن بالثواني

  // التكامل
  integral += error * dt;

  // التفاضل
  float derivative = (error - previous_error) / dt;

  // حساب خرج PID
  float output = Kp * error + Ki * integral + Kd * derivative;

  // تحويل الخرج لزاوية سيرفو
  float servo_angle = ANGLE_CENTER + output;

  // تحديد حدود الزاوية
  servo_angle = constrain(servo_angle, ANGLE_LEFT, ANGLE_RIGHT);

  // تحديث المتغيرات
  previous_error = error;
  last_time = current_time;

  return servo_angle;
}

void loop() {
  if (Serial1.available()) {
    String command = Serial1.readStringUntil('\n');
    command.trim();

    if (command.startsWith("E:")) {
      // استخراج الخطأ من الأمر
      float error = command.substring(2).toFloat();
      float servo_angle = pid_controller(error);
      steeringServo.write(servo_angle);
      Serial.print("📐 Error: ");
      Serial.print(error);
      Serial.print(", Servo Angle: ");
      Serial.println(servo_angle);
    } else if (command == "FORWARD") {
      esc.writeMicroseconds(PWM_FORWARD);
      Serial.println("🏎️ Command: FORWARD");
    } else if (command == "STOP") {
      esc.writeMicroseconds(PWM_NEUTRAL);
      Serial.println("⛔ Command: STOP");
    } else {
      Serial.print("⚠️ Unknown command: ");
      Serial.println(command);
    }
  }
}
