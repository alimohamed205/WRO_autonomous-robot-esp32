#define VELOCITY_TEMP(temp) ((331.5 + 0.6 * (float)(temp)) * 100 / 1000000.0)  // cm/us
int ultrasonicPins[] = { 2, 3, 4, 5 };  // Shared trig/echo pins for ultrasonic sensors

uint32_t pulseWidthUs;
uint16_t distance;
int16_t temp = 20;  // Default temp for ultrasonic speed


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
    Serial.println("cm  ") ; 

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


void setup() {
  Serial.begin(115200);


  // --- BNO055 Initialization ---
  
// --- Loop ---
}
void loop() {
  // --- Print sensor data ---
  printUltrasonicDistances();
float frontDistance = getUltrasonicDistance(2);
float rightDistance = getUltrasonicDistance(3);
float leftDistance = getUltrasonicDistance(4);
float backDistance = getUltrasonicDistance(5);}
