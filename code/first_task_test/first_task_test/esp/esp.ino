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

float Kp = 2.5, Ki = 0.0, Kd = 0.15;  // tuned for faster and stable response
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
float current_servo_angle = 90;
float desired_servo_angle = 90;
float angle_step = 3 ;
unsigned long last_servo_update = 0;
int servo_update_interval = 30;

void softStartToSpeed(int target_pwm) {
  int speed = 200;
  int step = -20;
  int interval = 5;

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  while (speed >= target_pwm) {
    ledcWrite(ENA, speed);
    delay(interval);
    speed += step;
  }

  ledcWrite(ENA, target_pwm);
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

void loop() {
  if (Serial1.available()) {
    String command = Serial1.readStringUntil('\n');
    command.trim();
    Serial.print("📥 Command: "); Serial.println(command);

    if (command == "GET_INITIAL_YAW") {
      initial_yaw = getYaw();
      Serial1.println("INITIAL_YAW:" + String(initial_yaw));
    }
    else if (command == "GET_CURRENT_YAW") {
      float yaw = getYaw();
      Serial1.println("CURRENT_YAW:" + String(yaw));
    }
    else if (command == "FORWARD") {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      ledcWrite(ENA, 170);
      forward_correction_active = false;
      pid_active = false;
      desired_servo_angle = 90;
    }
    else if (command.startsWith("FORWARD_CORRECT:")) {
      forward_target_yaw = normalizeAngle(command.substring(16).toFloat());
      forward_correction_active = true;
      pid_active = false;
      softStartToSpeed(120);
    }
    else if (command.startsWith("TURN_TO:")) {
      target_yaw = normalizeAngle(command.substring(8).toFloat());
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      ledcWrite(ENA, 120);
      pid_active = true;
      forward_correction_active = false;
      previous_error = 0;
      integral = 0;
      last_time = millis();
      Serial.print("🎯 Target yaw: "); Serial.println(target_yaw);
    }
    else if (command == "CENTER") {
      desired_servo_angle = 90;
      pid_active = false;
      forward_correction_active = false;
    }
    else if (command == "PAUSE") {
      ledcWrite(ENA, 0);
      desired_servo_angle = 90;
      steeringServo.write(90);
    }
    else if (command == "STOP") {
      ledcWrite(ENA, 0);
      pid_active = false;
      forward_correction_active = false;
      desired_servo_angle = 90;
    }
    Serial1.flush(); 
  }

  if (pid_active || forward_correction_active) {
    unsigned long now = millis();
    dt = (now - last_time) / 1000.0;
    if (dt < 0.04) return;
    last_time = now;

    float current_yaw = getYaw();
    float diff = angleError(pid_active ? target_yaw : forward_target_yaw, current_yaw);
    Serial.print("Current yaw: "); Serial.print(current_yaw); Serial.print(" | Diff: "); Serial.println(diff);
    float angle;

    if (abs(diff) > 10.0) {
      angle = constrain(90 + diff * 1.2, 50, 130);
      integral = 0;
      previous_error = diff;
    } else {
      error = diff;
      integral += error * dt;
      integral = constrain(integral, -integral_limit, integral_limit);
      float derivative = (error - previous_error) / dt;
      float output = Kp * error + Ki * integral + Kd * derivative;
      previous_error = error;
      output = constrain(output, -max_output, max_output);
      angle = constrain(90 + output, 50, 130);
    }

    desired_servo_angle = angle;

    if (pid_active && abs(diff) < 10.0) {
      desired_servo_angle = 90;
      pid_active = false;
      initial_yaw = current_yaw;
      delay(100);  // short delay to stabilize
      Serial.println("✅ TURN_DONE");
      Serial1.println("TURN_DONE");
    }
  }

  if (millis() - last_servo_update >= servo_update_interval) {
    if (abs(current_servo_angle - desired_servo_angle) > angle_step) {
      current_servo_angle += (desired_servo_angle > current_servo_angle) ? angle_step : -angle_step;
      current_servo_angle = constrain(current_servo_angle, 50, 130);
      steeringServo.write(current_servo_angle);
    }
    last_servo_update = millis();
  }
}

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
