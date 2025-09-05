#include <Arduino.h>
#define IN1 25
#define IN2 26
#define ENA 27

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  ledcAttachChannel(ENA, 1000, 8, 0);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void loop() {
  ledcWrite(ENA, 100);  // سرعة منخفضة
  Serial.println("سرعة منخفضة");
  delay(2000);
  ledcWrite(ENA, 128); // سرعة متوسطة
  Serial.println("سرعة متوسطة");
  delay(2000);
  ledcWrite(ENA, 150); // سرعة كاملة
  Serial.println("سرعة كاملة");
  delay(2000);
}
