#include <Wire.h>
#include "DFRobot_BNO055.h"
#include <Servo.h>


int buzzer = 13;
int start_button = 12;



#define VELOCITY_TEMP(temp) ((331.5 + 0.6 * (float)(temp)) * 100 / 1000000.0)  // cm/us

typedef DFRobot_BNO055_IIC BNO;
BNO bno(&Wire, 0x28);  // BNO055 on I2C address 0x28

// --- Pins ---
int ultrasonicPins[] = { 2, 3, 4, 5 };  // Shared trig/echo pins for ultrasonic sensors
Servo esc;                              // ESC motor control
Servo myservo;                          // Steering servo

// --- Variables ---
uint32_t pulseWidthUs;
uint16_t distance;
int16_t temp = 20;  // Default temp for ultrasonic speed

int pos = 60;  // For steering sweep

// --- Functions ---
void printLastOperateStatus(BNO::eStatus_t eStatus) {
  switch (eStatus) {
    case BNO::eStatusOK: Serial.println("everything ok"); break;
    case BNO::eStatusErr: Serial.println("unknown error"); break;
    case BNO::eStatusErrDeviceNotDetect: Serial.println("device not detected"); break;
    case BNO::eStatusErrDeviceReadyTimeOut: Serial.println("device ready timeout"); break;
    case BNO::eStatusErrDeviceStatus: Serial.println("device internal status error"); break;
    default: Serial.println("unknown status"); break;
  }
}

void stopMotor() {
  esc.writeMicroseconds(1500);  // Neutral
}

void moveReverse(int pwmVal) {
  esc.writeMicroseconds(pwmVal);
  delay(300);  // ESC interprets this as brake

  // Back to neutral briefly
  stopMotor();
  delay(40);  // Pause before real reverse

  // Second tap = real reverse
  esc.writeMicroseconds(pwmVal);
}

void moveForward(int pwmVal) {
  //stopMotor();
  //delay(100);                   // Optional safety delay
  esc.writeMicroseconds(pwmVal);
}

void printUltrasonicDistances() {
  Serial.print("Distances: ");
  for (int i = 0; i < 4; i++) {
    int pin = ultrasonicPins[i];
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delayMicroseconds(2);
    digitalWrite(pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(pin, LOW);

    pinMode(pin, INPUT);
    pulseWidthUs = pulseIn(pin, HIGH);
    distance = pulseWidthUs * VELOCITY_TEMP(temp) / 2.0;

    Serial.print("S");
    Serial.print(i + 1);
    Serial.print(":");
    Serial.print(distance);
    Serial.print("cm  ");

    //  delay(50);
  }
}

float getUltrasonicDistance(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin, LOW);

  pinMode(pin, INPUT);
   pulseWidthUs =  pulseIn(pin, HIGH);
  distance = pulseWidthUs * VELOCITY_TEMP(temp) / 2.0;
  return distance;
}


void printIMUData() {
  BNO::sEulAnalog_t sEul = bno.getEul();

  Serial.print(" | pitch:");
  Serial.print(sEul.pitch, 3);
  Serial.print(" roll:");
  Serial.print(sEul.roll, 3);
  Serial.print(" yaw:");
  Serial.print(sEul.head, 3);
  Serial.println();
}








void driveStraightWithCorrection(float targetYaw, int durationMs, int forwardPWM) {
  unsigned long startTime = millis();
  float currentYaw, yawError;
  int steerAngle;

  while (millis() - startTime < durationMs) {
    currentYaw = bno.getEul().head;
    currentYaw = fmod(currentYaw + 360, 360);
    targetYaw = fmod(targetYaw + 360, 360);

    yawError = targetYaw - currentYaw;

    if (yawError > 180) yawError -= 360;
    if (yawError < -180) yawError += 360;

    // Increase the correction factor to speed up the correction
    steerAngle = 88 + constrain(yawError * 1.2, -40, 40);  // Adjusted factor (1.5)
    steerAngle = constrain(steerAngle, 50, 130);

    myservo.write(steerAngle);
    esc.writeMicroseconds(forwardPWM);

    Serial.print("Current Yaw: ");
    Serial.print(currentYaw);
    Serial.print(" | Error: ");
    Serial.print(yawError);
    Serial.print(" | Steer: ");
    Serial.println(steerAngle);

    delay(50);
  }

 // stopMotor();
  myservo.write(88);  // Center steering
}


void rotateToAnglePID(float targetYaw, float Kp = 2.0, float Ki = 0.0, float Kd = 0.5, int drivePWM = 1600, int timeoutMs = 3000) {
  unsigned long startTime = millis();
  float currentYaw, error, lastError = 0;
  float integral = 0, derivative;
  float steerAngle;
  int stableCount = 0;

  targetYaw = fmod(targetYaw + 360, 360);

  while (millis() - startTime < timeoutMs) {
    currentYaw = bno.getEul().head;
    currentYaw = fmod(currentYaw + 360, 360);

    // Compute shortest error
    error = targetYaw - currentYaw;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    // PID calculations
    integral += error;
    derivative = error - lastError;
    float output = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;

    // Convert PID output to steering angle
    steerAngle = 88 + constrain(output, -38, 42);
    steerAngle = constrain(steerAngle, 50, 130);
    myservo.write(steerAngle);
    esc.writeMicroseconds(drivePWM);  // Apply motor

    Serial.print("Target: "); Serial.print(targetYaw);
    Serial.print(" | Current: "); Serial.print(currentYaw);
    Serial.print(" | Error: "); Serial.print(error);
    Serial.print(" | Steer: "); Serial.println(steerAngle);

    // Check if error is within ±2 degrees for at least 3 cycles
    if (abs(error) < 1) {
      stableCount++;
      if (stableCount >= 1) {
        break;  // Angle reached and stable
      }
    } else {
      stableCount = 0;  // Reset if error grows
    }

    delay(50);
  }

  stopMotor();
  myservo.write(88);  // Center steering
  Serial.println("Target angle reached. Robot stopped.");
}






// --- Setup ---
void setup() {
  Serial.begin(115200);
  myservo.write(88);  // Center steering

  pinMode(start_button, INPUT_PULLUP);
  pinMode(buzzer, OUTPUT);

  // --- BNO055 Initialization ---
  bno.reset();
  while (bno.begin() != BNO::eStatusOK) {
    Serial.println("BNO055 begin failed");
    printLastOperateStatus(bno.lastOperateStatus);
    //delay(2000);
  }
  Serial.println("BNO055 begin success");

  // --- ESC Motor Initialization ---
  esc.attach(11);               // Motor ESC pin
  esc.writeMicroseconds(1500);  // Arm ESC
  delay(3000);

  // --- Steering Servo Initialization ---
  myservo.attach(9);  // Steering servo pin
}

// --- Loop ---
void loop() {
  // --- Print sensor data ---
  printUltrasonicDistances();
  printIMUData();
float frontDistance = getUltrasonicDistance(2);
float rightDistance = getUltrasonicDistance(3);
float leftDistance = getUltrasonicDistance(4);
float backDistance = getUltrasonicDistance(5);

//Serial.println(frontDistance);

rotateToAnglePID(80, .9, 0.0, 0.4, 1600);  // Turn right gently to 90°
  delay(1000);
driveStraightWithCorrection(90, 1000, 1600);
  stopMotor();
  delay(1000);

rotateToAnglePID(170, .9, 0.0, 0.4, 1600);  // Turn right gently to 90°
  delay(1000);
driveStraightWithCorrection(180, 1000, 1600);
  stopMotor();
  delay(1000);

  rotateToAnglePID(260, .9, 0.0, 0.4, 1600);  // Turn right gently to 90°
  delay(1000);
  driveStraightWithCorrection(270, 1000, 1600);
  stopMotor();
  delay(1000);

  rotateToAnglePID(350, .9, 0.0, 0.4, 1600);  // Turn right gently to 90°
  delay(1000);
  driveStraightWithCorrection(355, 1000, 1600);
  stopMotor();
  delay(1000);

  /*driveStraightWithCorrection(180, 3500, 1590);  // Drive straight for 5s with correction
  stopMotor();

  driveStraightWithCorrection(270, 3500, 1590);  // Drive straight for 5s with correction
  stopMotor();

  driveStraightWithCorrection(360, 3500, 1590);  // Drive straight for 5s with correction
  stopMotor();*/

  /*rotateToAbsoluteYaw(65, 120, 1590); // Turn right
  delay(1000);
  driveStraightWithCorrection(90, 2000, 1590); // Drive straight for 5s with correction
  delay(1000);

  rotateToAbsoluteYaw(165, 120, 1590); // Turn right
  delay(1000);
  driveStraightWithCorrection(180, 2000, 1590); // Drive straight for 5s with correction
  delay(1000);


    rotateToAbsoluteYaw(255, 120, 1590); // Turn right
  delay(1000);
  driveStraightWithCorrection(270, 2000, 1590); // Drive straight for 5s with correction
  delay(1000);


    rotateToAbsoluteYaw(345, 120, 1590); // Turn right
  delay(1000);
  driveStraightWithCorrection(358, 2000, 1590); // Drive straight for 5s with correction
  delay(1000);
 */
  //moveForward(1590);  // Adjust value for speed


  //moveReverse(1350);  // Adjust value for reverse speed
  //delay(500);

  // stopMotor();
  // delay(50);
  //BNO::sEulAnalog_t sEul = bno.getEul();  // get yaw

  //steerBasedOnYaw(sEul.head);            // adjust steering


  // --- Sweep Servo ---
  //for (pos = 50; pos <= 130; pos += 1) {
  //myservo.write(88);
}