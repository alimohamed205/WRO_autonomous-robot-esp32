#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_BNO055.h>
#include <ESP32Servo.h>

#define IN1 25
#define IN2 26
#define ENA 27
#define STEERING_PIN 14
#define RX_PIN 16
#define TX_PIN 17
#define SDA_PIN 21
#define SCL_PIN 22

typedef DFRobot_BNO055_IIC BNO;
BNO bno(&Wire, 0x28);
Servo steeringServo;

// ✅ PID Params (حسب أفضل تجربة)
float Kp = 1.2, Ki = 0.0, Kd = 0.3;
float error = 0, previous_error = 0, integral = 0;
float dt = 0.04;
unsigned long last_time = 0;
float integral_limit = 50.0;
float max_output = 30.0;

float initial_yaw = 0;
float target_yaw = 0;
bool pid_active = false;
bool forward_correction_active = false;
float forward_target_yaw = 0;

// أضف هذه الدالة قبل setup()
void softStartToSpeed(int target_pwm) {
  int speed = 200;
  int step = -10;
  int interval = 70;

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  while (speed >= target_pwm) {
    ledcWrite(ENA, speed);
    delay(interval);
    speed += step;
  }

  ledcWrite(ENA, target_pwm); // ثبت على السرعة المطلوبة
}


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial1.setTimeout(20);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  ledcAttachChannel(ENA, 1000, 8, 2);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  ledcWrite(ENA, 0);

  Wire.begin(SDA_PIN, SCL_PIN);
  while (bno.begin() != bno.eStatusOK) {
    Serial.println("❌ BNO055 init failed");
    delay(1000);
  }
  Serial.println("✅ BNO055 ready");

  steeringServo.attach(STEERING_PIN);
  steeringServo.write(90);
  delay(1000);
  Serial.println("✅ System ready");

  initial_yaw = getYaw();
  last_time = millis();
}

// ---------- loop ----------
void loop() {
  if (Serial1.available()) {
    String cmd = Serial1.readStringUntil('\n');
    cmd.trim();
    Serial.println("📥 " + cmd);

    if (cmd == "GET_INITIAL_YAW") {
      Serial1.println("INITIAL_YAW:" + String(getYaw()));
    }
    else if (cmd == "GET_CURRENT_YAW") {
      Serial1.println("CURRENT_YAW:" + String(getYaw()));
    }
    else if (cmd == "FORWARD") {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      ledcWrite(ENA, 170);
      pid_active = false;
      steeringServo.write(90);
    }
    else if (cmd.startsWith("FORWARD_CORRECT:")) {
      forward_target_yaw = normalizeAngle(cmd.substring(16).toFloat());
      forward_correction_active = true;
      pid_active = false;
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      ledcWrite(ENA, 140);
    }
    else if (cmd.startsWith("TURN_TO:")) {
      target_yaw = normalizeAngle(cmd.substring(8).toFloat());
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      ledcWrite(ENA, 100);
      pid_active = true;
      forward_correction_active = false;
      previous_error = integral = 0;
      last_time = millis();
    }
    else if (cmd == "STOP") {
      ledcWrite(ENA, 0);
      pid_active = forward_correction_active = false;
      steeringServo.write(90);
    }
  }

  if (pid_active || forward_correction_active) {
    unsigned long now = millis();
    dt = (now - last_time) / 1000.0;
    if (dt < 0.04) return;
    last_time = now;

    float current = getYaw();
    float diff = angleError(pid_active ? target_yaw : forward_target_yaw, current);
    float out = 90.0;
    if (abs(diff) > 10) {
      out = constrain(90 + diff * 1.2, 50, 130);
      integral = 0;
    } else {
      error = diff;
      integral += error * dt;
      integral = constrain(integral, -integral_limit, integral_limit);
      float derivative = (error - previous_error) / dt;
      float pid = Kp * error + Ki * integral + Kd * derivative;
      previous_error = error;
      out = constrain(90 + pid, 50, 130);
    }
    steeringServo.write(out);

    if (pid_active && abs(diff) < 3.0) {
      steeringServo.write(90);
      pid_active = false;
      Serial1.println("TURN_DONE");
    }
  }
}
// ----------------------
float getYaw() {
  auto eul = bno.getEul();
  float yaw = eul.head;
  return yaw < 0 ? yaw + 360 : yaw;
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
