/*
_____________________________________________________________
Code for the reaction control system controlled by a Linear Actuator
Using SparkFun TB6612FNG driver and BNO055 IMU
Code by: Andrew Schmit
Last modified: 6/30/2025
_____________________________________________________________
*/
// gonna add some defines in this for actuator setup then they can  be moved
#define SET(port, bit) ((port) |= (1 << (bit))) // this is supposed to make SET turn on LED
#define CLR(port, bit) ((port) &= ~(1 << (bit))) // turns off LED, this is imported code, can be changed if issues, or if wrong.

void Actuatorsetup() {

  analogWrite(PWMB, moveDuty); 

  printOLED("Starting Actuator test.");
/*
  //creates a value to use for position... does what analogRead does apparently
  // Configure ADMUX register
  // Select AVCC as reference (REFS bits)
  // Select A0 as input channel (MUX bits)
  ADMUX = (1 << REFS0) | (0 << MUX28); // change MUX0 to whatever anaolog pin is the input.

  // Configure ADCSRA register
  // Enable ADC (ADEN bit)
  // Set prescaler for a suitable ADC clock (ADPS bits)
  ADCSRA = (1 << ADEN) | (1 << ADPS2); // Example: Enable ADC, prescaler 16
*/
  moveActuator(false);
  for (int i = 0; i < 50; i++){
    delay(100);
    pos = analogRead(feedbackPin);
      if (pos > maxVal) {
        maxVal = pos;
      }
    //pos = newAnalogRead();
    printOLED(String(pos));
  }
  maxPos = maxVal;

  moveActuator(true);
  for (int i = 0; i < 50; i++){
    delay(100);
    pos = analogRead(feedbackPin);
      if (pos < minVal) {
        minVal = pos;
      }
    //pos = newAnalogRead();
    printOLED(String(pos));
  }
  minPos = minVal;
  
  moveActuator(false);
  for (int i = 0; i < 50; i++){
    delay(100);
    pos = analogRead(feedbackPin);
    //pos = newAnalogRead();
    truePos = posMap(pos,minPos,maxPos);
    printOLED(String(truePos));
  }
  
  resetActuator();
  printOLED(String(truePos));

  printOLED("Actuator test complete.");
}

float posMap(int pos,int min,int max){
  return (float)(pos - min) * (5.0 / (max - min)); // only change that might be meaningful
}
// Moves actuator in a direction at duty cycle
// Stops the actuator
void stopActuator() { 
  analogWrite(PWMB, 0);
}

void resetActuator() {
  // Continuously move actuator until it's within the desired range
  while (true) {
    pos = analogRead(feedbackPin);
    //pos = newAnalogRead();
    truePos = posMap(pos, minPos, maxPos);

    if (truePos > targetPos + tolerance) {
      moveActuator(true);   // Move toward retracted
    }
    else if (truePos < targetPos - tolerance) {
      moveActuator(false);  // Move toward extended
    }
    else {
      stopActuator(); 
      delay(100);
      Serial.println("centered");
      break;  // Exit loop when within range
    }
    delay(50);  // Give time for motor to move
  }
}

void updateLinearActuator() {  
  if(pixy.ccc.numBlocks > 0){
     proportionalZ = KPZ * errorZ; // find proportional term
     Serial.println("linear Actuator stuff: " + String(proportionalZ) + ", " + String(errorZ) + ", " + String(tiltOffset));
      if (proportionalZ < -5){
        Serial.println("move true");
        moveActuator(true);
      }
      else if (proportionalZ > 5) {
        Serial.println("move false");
        moveActuator(false);
      }
      else {
        Serial.println("move stop");
        stopActuator();
      }
    }
  else{
    stopActuator();
  }
}
// Reads and prints position every 100 ms for `duration` milliseconds
/*
void moveActuator(bool extend) {
  digitalWrite(BIN1, extend ? HIGH : LOW); // 12 pin
  digitalWrite(BIN2, extend ? LOW : HIGH); //11 pin
  analogWrite(PWMB, moveDuty);  // 20% 
  Serial.println("movibg");
}
*/
void moveActuator(bool extend) { // new MOVEACTUATOR
  if(extend) {
    SET(PORTB, 12);
    CLR(PORTB, 11);
  }
  else {
    CLR(PORTB, 12);
    SET(PORTB, 11);
  }
  analogWrite(PWMB, moveDuty);
  Serial.println("movibg");
}
/*
void newAnalogRead() {  // gets analog Position
  // Start conversion (ADSC bit)
  ADCSRA |= (1 << ADSC);

  // Wait for conversion to complete (ADIF bit)
  while (!(ADCSRA & (1 << ADIF)));

  // Clear conversion complete flag (ADIF bit)
  ADCSRA |= (1 << ADIF);

  // Read the 10-bit result from ADCL and ADCH registers
  // Note: ADCL must be read first, then ADCH
  int sensorValue = ADCL | (ADCH << 8); // ADCL lower bits ADCH higher bits
  return sensorValue;

  // Now 'sensorValue' contains the digital representation of the analog input on A0
  // You can use this value as needed in your program
  // For example, Serial.println(sensorValue);
}
*/

// float calculatePitch(float pitch){
//   sensors_event_t event;
//   bno.getEvent(&event);
//   pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
//   return pitch;
// }
