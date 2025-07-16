// Actuonix L12-50-50-6-P Test Sketch
// Hardware: Arduino Uno (or similar), TB6621FMG driver, Actuonix L12 actuator with potentiometer feedback

// Pin Definitions
const int BIN1_PIN    = 8;    // TB6621 input 1
const int BIN2_PIN    = 9;    // TB6621 input 2
const int PWMB_PIN    = 10;    // TB6621 PWM enable pin (connected to ENABLE)
const int FEEDBACK_PIN = 28;  // Actuonix feedback pot output

// Control Parameters
const float KP = 0.5;         // Proportional gain, adjust as needed

// Actuator Properties
const int FEEDBACK_MIN = 0;    // Minimum analog value (fully retracted)
const int FEEDBACK_MAX = 1023; // Maximum analog value (fully extended)

// Target position (0.0 to 1.0 corresponds to 0% to 100% extension)
float targetPosition = 0.5;    // Default to mid-stroke

void setup() {
  // Initialize pins
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);
  pinMode(FEEDBACK_PIN, INPUT);

  // Initialize Serial for debugging
  Serial.begin(9600);
  while (!Serial) { ; }
  Serial.println("Actuonix L12 Test Initialized");
}

void loop() {
  // Read feedback
  int raw = analogRead(FEEDBACK_PIN);
  float currentPosition = (raw - FEEDBACK_MIN) / float(FEEDBACK_MAX - FEEDBACK_MIN);
  currentPosition = constrain(currentPosition, 0.0, 1.0);

  // Compute error
  float error = targetPosition - currentPosition;

  // Compute control output
  float control = KP * error;
  control = constrain(control, -1.0, 1.0);

  // Drive motor
  if (abs(control) < 0.05) {
    // Within deadband: stop
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, LOW);
    analogWrite(PWMB_PIN, 0);
  } else if (control > 0) {
    // Extend actuator
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, LOW);
    analogWrite(PWMB_PIN, int(control * 255));
  } else {
    // Retract actuator
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, HIGH);
    analogWrite(PWMB_PIN, int(-control * 255));
  }

  // Debug output
  Serial.print("Target: "); Serial.print(targetPosition, 2);
  Serial.print(" | Current: "); Serial.print(currentPosition, 2);
  Serial.print(" | Error: "); Serial.print(error, 2);
  Serial.print(" | PWM: "); Serial.println(int(abs(control) * 255));

  // Simple serial command to update target
  if (Serial.available()) {
    float input = Serial.parseFloat();
    if (input >= 0.0 && input <= 1.0) {
      targetPosition = input;
      Serial.print("New target set: "); Serial.println(targetPosition, 2);
    }
  }

  delay(20); // Loop delay (~50 Hz)
}
