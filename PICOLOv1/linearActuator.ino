/*
_____________________________________________________________
Code for the reaction control system controlled by a Linear Actuator
Using SparkFun TB6612FNG driver and BNO055 IMU
Code by: Andrew Schmit
Last modified: 6/30/2025
_____________________________________________________________


void Actuatorsetup() {
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  
  digitalWrite(STBY, HIGH); // Wake up motor driver

  Serial.println("Motor driver awake\n");

}

void loop() {
  // Extend actuator
  Serial.println("Extending...");
  moveActuator(true, moveSpeed);
  printPositionDuringMove(moveTime);

  // Stop
  stopActuator();
  delay(1000);

  // Retract actuator
  Serial.println("Retracting...");
  moveActuator(false, moveSpeed);
  printPositionDuringMove(moveTime);

  // Stop
  stopActuator();
  delay(2000);
}

// Moves actuator in a direction
void moveActuator(bool extend, int speed) {
  digitalWrite(BIN1, extend ? HIGH : LOW);
  digitalWrite(BIN2, extend ? LOW : HIGH);
  analogWrite(PWMB, speed);
}

// Stops the actuator
void stopActuator() {
  analogWrite(PWMB, 0);
}

// Normalize heading error to range [-180, 180]
float calculateHeadingErrorZ(float currentZ, float targetZ) {
  float errorZ = targetZ - currentZ;
  while (errorZ > 180) error -= 360;
  while (errorZ < -180) error += 360;
  return errorZ;
}

float calculateP(float errorZ) {
  float proportionalZ = KPZ * errorZ;
 
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0;
 
  float outputZ = proportionalZ

  previousErrorZ = errorZ;
  previousTime = currentTime;

  return constrain(outputZ, -255, 255);
}

void updateLinearActuator(float targetHeadingZ) {
  float currentHeadingZ = orientation[2];  
  float errorZ = calculateHeadingErrorZ(currentHeadingZ, targetHeadingZ);
  float pOutput = calculateP(errorZ);
  moveActuator(////// )

// Reads and prints position every 100 ms for `duration` milliseconds
void printPositionDuringMove(int duration) {
  int start = millis();
  while (millis() - start < duration) {
    int pos = analogRead(feedbackPin); // 0–4095 (12-bit)
    Serial.print("Position: ");
    Serial.println(pos);
    delay(100);
  }
}
*/