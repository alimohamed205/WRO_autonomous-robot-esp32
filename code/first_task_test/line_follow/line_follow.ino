#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_BNO055.h>
#include <ESP32Servo.h>

// Pin definitions
#define IN1 25
#define IN2 26
#define ENA 27
#define STEERING_PIN 13
#define RX_PIN 16
#define TX_PIN 17
#define SDA_PIN 21
#define SCL_PIN 22

// IMU
typedef DFRobot_BNO055_IIC BNO;
BNO myIMU(&Wire, 0x28);

// Servo
Servo steeringServo;

// PID variables
float Kp = 0.5, Ki = 0.001, Kd = 0.2;
float error = 0, previous_error = 0, integral = 0;
unsigned long last_time = 0;
float dt = 0.04;
float integral_limit = 50.0;
float initial_yaw = 0, target_yaw = 0;
bool pid_active = false;
bool steering_smooth = false;
unsigned long steer_start_time = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  // Motor setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  ledcAttachChannel(ENA, 1000, 8, 2);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  ledcWrite(ENA, 0);

  // IMU setup
  Wire.begin(SDA_PIN, SCL_PIN);
  while (myIMU.begin() != myIMU.eStatusOK) {
    Serial.println("❌ BNO055 init failed");
    delay(1000);
  }
  Serial.println("✅ BNO055 ready");
  delay(2000);

  // Servo setup
  steeringServo.attach(STEERING_PIN);
  steeringServo.write(88);
  delay(2000);

  Serial.println("✅ System ready");

  initial_yaw = getYaw();
  last_time = millis();
}

void loop() {
  if (Serial1.available()) {
    String command = Serial1.readStringUntil('\n');
    command.trim();
    Serial.print("📥 Received: "); Serial.println(command);

    if (command == "GET_INITIAL_YAW") {
      initial_yaw = getYaw();
      Serial1.println("INITIAL_YAW:" + String(initial_yaw));
      Serial.print("📤 Sent initial yaw: "); Serial.println(initial_yaw);
    }
    else if (command == "GET_CURRENT_YAW") {
      float yaw = getYaw();
      Serial1.println("CURRENT_YAW:" + String(yaw));
      Serial.print("📤 Sent current yaw: "); Serial.println(yaw);
    }
    else if (command == "FORWARD") {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      ledcWrite(ENA, 100);
      Serial.println("🏎️ Moving forward...");
    }
    else if (command == "STOP") {
      ledcWrite(ENA, 0);
      Serial.println("⛔ Stopped");
    }
    else if (command == "TURN_RIGHT") {
      target_yaw = normalizeAngle(initial_yaw + 30);
      steeringServo.write(130);
      steering_smooth = true;
      steer_start_time = millis();
      Serial.println("↪️ Smooth turning right (30°)");
    }
    else if (command == "TURN_LEFT") {
      target_yaw = normalizeAngle(initial_yaw - 30);
      steeringServo.write(50);
      steering_smooth = true;
      steer_start_time = millis();
      Serial.println("↩️ Smooth turning left (30°)");
    }
  }

  if (steering_smooth && (millis() - steer_start_time > 1000)) {
    steering_smooth = false;
    pid_active = true;
    Serial.println("🔄 Switching to PID control");
  }

  if (pid_active) {
    unsigned long now = millis();
    dt = (now - last_time) / 1000.0;
    if (dt < 0.04) return;
    last_time = now;

    float current_yaw = getYaw();
    float diff = angleError(target_yaw, current_yaw);

    error = diff;
    integral += error * dt;
    if (integral > integral_limit) integral = integral_limit;
    if (integral < -integral_limit) integral = -integral_limit;
    float derivative = (error - previous_error) / dt;
    float output = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;

    float angle = 88 + output;
    angle = constrain(angle, 50, 130);
    steeringServo.write(angle);

    Serial.print("📐 Current: "); Serial.print(current_yaw);
    Serial.print(" | Target: "); Serial.print(target_yaw);
    Serial.print(" | Output: "); Serial.println(angle);

    if (abs(diff) < 2.0) {
      steeringServo.write(88);
      pid_active = false;
      initial_yaw = current_yaw;
      Serial.println("✅ Turn complete!");
    }
  }
}

float getYaw() {
  auto eul = myIMU.getEul();
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
