#include <Wire.h>
#include <DFRobot_BNO055.h>
#include <ESP32Servo.h>

// IMU & Servo Pins
#define I2C_SDA 21
#define I2C_SCL 22
#define STEERING_PIN 13
#define ESC_PIN 12

DFRobot_BNO055_IIC bno(&Wire, 0x28);
Servo steeringServo;
Servo esc;

// PID parameters
float Kp = 2.0;
float Ki = 0.0;
float Kd = 0.5;

float error = 0, previous_error = 0, integral = 0;
unsigned long last_time = 0;

// Yaw tracking
float initial_yaw = 0;
float target_yaw = 0;
bool turning = false;

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  // Init IMU
  while (bno.begin() != bno.eStatusOK) {
    Serial.println("❌ BNO055 init failed");
    delay(1000);
  }
  Serial.println("✅ BNO055 ready");

  // Init servo and ESC
  steeringServo.attach(STEERING_PIN);
  steeringServo.write(88);  // Center

  esc.attach(ESC_PIN);
  esc.writeMicroseconds(1500); // Neutral
  delay(2000); // Arm ESC
  Serial.println("✅ ESC armed");

  // Set initial yaw and target yaw
  delay(500);
  initial_yaw = getYaw();
  target_yaw = normalizeAngle(initial_yaw + 30);  // turn right 30 degrees
  Serial.print("📌 Initial Yaw: "); Serial.println(initial_yaw);
  Serial.print("🎯 Target Yaw: "); Serial.println(target_yaw);

  last_time = millis();
  turning = true;

  // Start motor forward
  esc.writeMicroseconds(1600);  // Forward
}

void loop() {
  if (turning) {
    float current_yaw = getYaw();
    float diff = angleError(target_yaw, current_yaw);

    Serial.print("↪️ Current Yaw: "); Serial.print(current_yaw);
    Serial.print(" | Error: "); Serial.println(diff);

    // PID
    unsigned long now = millis();
    float dt = (now - last_time) / 1000.0;

    error = diff;
    integral += error * dt;
    float derivative = (error - previous_error) / dt;
    float output = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;
    last_time = now;

    // Map output to servo angle
    float angle = 88 + output;
    angle = constrain(angle, 50, 130);
    steeringServo.write(angle);

    // Stop condition
    if (abs(diff) < 2.0) {
      Serial.println("✅ Turn complete!");
      steeringServo.write(88);  // Center
      esc.writeMicroseconds(1500); // Stop motor
      turning = false;
    }

    delay(50);
  }
}

// Helpers
float getYaw() {
  auto eul = bno.getEul();
  float yaw = eul.head;
  if (yaw < 0) yaw += 360;
  return yaw;
}

float normalizeAngle(float angle) {
  while (angle >= 360) angle -= 360;
  while (angle < 0) angle += 360;
  return angle;
}

float angleError(float target, float current) {
  float err = target - current;
  if (err > 180) err -= 360;
  if (err < -180) err += 360;
  return err;
}
