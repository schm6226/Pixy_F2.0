/*
_____________________________________________________________
Code for the reaction control system controled by a continous rotational servo
Code by: Drew Miler
Modified by: Andrew Schmit
Date of last modification: 14 July 2025
_____________________________________________________________

*/
void Controlwheelsetup() {

  Serial.println("Reaction Control Wheel System Initializing...");
  
  // Initialize reaction wheel servo
  pinMode(SERVO_PIN, OUTPUT);
  analogWriteFreq(50);  // Set PWM frequency to 50 Hz (20ms period)
  delay(1000);  // Allow servo to initialize
  
     printOLED("Starting sweep test...");

    // Full reverse (~1ms pulse → ~5% duty cycle)
    analogWrite(SERVO_PIN, 26);
    Serial.println("Full reverse");
    delay(2000);

    // Stop (~1.5ms pulse → ~7.5% duty cycle)
    analogWrite(SERVO_PIN, 38);
    Serial.println("Stop");
    delay(2000);

    // Full forward (~2ms pulse → ~10% duty cycle)
    analogWrite(SERVO_PIN, 51);
    Serial.println("Full forward");
    delay(2000);

    // Stop again
    analogWrite(SERVO_PIN, 38);
    Serial.println("Stop");
    delay(2000);

    printOLED("Servo test complete.");
}

void setServoSpeed(int speed) {
    // Map speed (-100 to 100) to pulse width (1000 to 2000 µs)
    int pulseWidth = map(speed, -100, 100, 1000, 2000);
    
    // Convert pulse width to duty cycle (0-255 for Pico's analogWrite)
    int dutyCycle = map(pulseWidth, 1000, 2000, 26, 51);  // ~5% to ~10% duty cycle

    analogWrite(14, dutyCycle);  // Set PWM output
}
/*
float calculateHeadingError(float current, float target) {
  // Normalize the error to -180 to 180 range
  float error = target - current;
  while (error > 180) error -= 360;
  while (error < -180) error += 360;
  return error;
}
*/
float calculatePID(float error) {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0;  // Convert to seconds
  
  // Proportional term
  float proportional = 0;
  proportional = KP * error;
  
  // Integral term (with anti-windup)
  integral += KI * error * deltaTime;
  integral = constrain(integral, -MAX_SERVO_SPEED, MAX_SERVO_SPEED);
  
  // Derivative term
  float derivative = 0;
  if (deltaTime > 0) {
    derivative = KD * (error - previousError) / deltaTime;
  }
  
  // Calculate total output
  float output = proportional + integral + derivative;
  
  // Update variables for next iteration
  previousError = error;
  previousTime = currentTime;
  
  return output;
}

int mapPIDToServo(float pidOutput) {
  // Constrain PID output to servo range
  pidOutput = constrain(pidOutput, -MAX_SERVO_SPEED, MAX_SERVO_SPEED);
  
  // Map PID output to servo command
  int servoCommand = NEUTRAL_SERVO + pidOutput;
  servoCommand = constrain(servoCommand, MIN_SERVO_SPEED, MAX_SERVO_SPEED);
  
  return servoCommand;
}