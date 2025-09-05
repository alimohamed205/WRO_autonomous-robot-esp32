#include <Wire.h>
#include "DFRobot_BNO055.h"

#define SDA_PIN 21
#define SCL_PIN 22

typedef DFRobot_BNO055_IIC BNO;
BNO myIMU(&Wire, 0x28);

float imu_initial_yaw = 0;
bool imu_initial_set = false;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  while (myIMU.begin() != BNO::eStatusOK) {
    Serial.println("❌ BNO055 Initialization Failed");
    delay(2000);
  }
  Serial.println("✅ BNO055 Ready");

  delay(1000);
}

void loop() {
  BNO::sEulAnalog_t eul = myIMU.getEul();
  float current_yaw = eul.head;

  if (!imu_initial_set) {
    imu_initial_yaw = current_yaw;
    imu_initial_set = true;

    Serial.print("📍 Initial Yaw set to: ");
    Serial.println(imu_initial_yaw);
  }

  Serial.print("🔁 Current Yaw: ");
  Serial.println(current_yaw);
  Serial.print("🔄 Yaw Diff: ");
  Serial.println(current_yaw - imu_initial_yaw);

  delay(500);
}
