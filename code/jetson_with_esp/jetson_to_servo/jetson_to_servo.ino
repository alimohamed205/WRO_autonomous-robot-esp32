#include <ESP32Servo.h>

Servo steeringServo;

#define SERVO_PIN 13

String command = "";

void setup() {
  Serial1.begin(115200, SERIAL_8N1, 16, 17);  // توصيل UART مع Jetson Nano
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(88);  // وضع مستقيم
}

void loop() {
  if (Serial1.available()) {
    command = Serial1.readStringUntil('\n');
    command.trim();

    if (command == "LEFT") {
      steeringServo.write(50);
      Serial1.println("↪️  Command: LEFT");
    } else if (command == "RIGHT") {
      steeringServo.write(130);
      Serial1.println("↩️  Command: RIGHT");
    } else if (command == "STRAIGHT") {
      steeringServo.write(88);
      Serial1.println("⬆️  Command: STRAIGHT");
    }
  }
}

