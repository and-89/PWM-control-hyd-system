// Control system for two hydraulic motors with four PWM valves
// The code reads a two-axis joystick, mixes throttle and direction,
// scales the results to avoid exceeding PWM limits, and drives forward
// and reverse channels for each motor controller.

const byte joysticYA = A0; // Analog joystick Y axis (throttle)
const byte joysticXA = A1; // Analog joystick X axis (direction)

const byte PWMleftFA = 10; // PWM FORWARD PIN for OSMC Controller A (left motor)
const byte PWMleftRA = 9;  // PWM REVERSE PIN for OSMC Controller A (left motor)
const byte PWMrightFB = 6; // PWM FORWARD PIN for OSMC Controller B (right motor)
const byte PWMrightRB = 5; // PWM REVERSE PIN for OSMC Controller B (right motor)
const byte disablePin = 2; // OSMC disable, pull LOW to enable motor controller

const int analogMidpoint = 512; // Midpoint value of the joystick inputs
const int pwmMax = 255;         // Maximum PWM value for 8-bit timers
const int deadZone = 5;         // Joystick dead zone to filter noise

int throttle = 0;
int direction = 0;

int leftMotor = 0;
int leftMotorScaled = 0; // Left motor command after scaling and limiting

int rightMotor = 0;
int rightMotorScaled = 0; // Right motor command after scaling and limiting

float maxMotorScale = 0; // Holds the mixed output scaling factor

int readAxisDelta(byte analogPin) {
  // Read the analog pin and convert the 0..1023 range to roughly -255..255
  const int raw = analogRead(analogPin);
  return (analogMidpoint - raw) / 2;
}

void mixInputs(int throttleInput, int directionInput, int &leftOutput, int &rightOutput) {
  // Differential mixing: throttle moves both motors, direction skews them
  leftOutput = throttleInput + directionInput;
  rightOutput = throttleInput - directionInput;
}

float calculateScaleFactor(int leftOutput, int rightOutput) {
  // Determine the largest magnitude as a fraction of the PWM range
  const float leftScale = abs(leftOutput) / static_cast<float>(pwmMax);
  const float rightScale = abs(rightOutput) / static_cast<float>(pwmMax);

  // Never allow the scale to drop below 1.0 so small signals are unaffected
  return max(1.0f, max(leftScale, rightScale));
}

int scaleAndConstrain(int rawValue, float scaleFactor, int minOutput, int maxOutput) {
  // Apply scaling to prevent exceeding the PWM range and clamp to hardware-safe bounds
  const float scaled = rawValue / scaleFactor;
  return constrain(static_cast<int>(scaled), minOutput, maxOutput);
}

void printMixDebug(const char *label, int rawValue, float scaleValue, int outputValue) {
  // Provide a compact debug line for serial monitoring
  Serial.print(label);
  Serial.print(" IN:");
  Serial.print(rawValue);
  Serial.print(" SCALE:");
  Serial.print(scaleValue, 2);
  Serial.print(" OUT:");
  Serial.print(outputValue);
  Serial.print(" | ");
}

void applyMotorOutput(byte forwardPin, byte reversePin, int command) {
  // Apply the signed command to forward/reverse PWM channels with dead-zone handling
  if (abs(command) <= deadZone) {
    analogWrite(forwardPin, 0);
    analogWrite(reversePin, 0);
    Serial.print("IDLE | ");
    return;
  }

  if (command > 0) {
    analogWrite(reversePin, 0);
    analogWrite(forwardPin, abs(command));
    Serial.print("F");
    Serial.print(abs(command));
    Serial.print(" | ");
  } else {
    analogWrite(forwardPin, 0);
    analogWrite(reversePin, abs(command));
    Serial.print("R");
    Serial.print(abs(command));
    Serial.print(" | ");
  }
}

void setup() {
  // Initialize serial monitor for diagnostics
  Serial.begin(19200);

  // Configure PWM pins for both motors
  pinMode(PWMleftFA, OUTPUT);
  pinMode(PWMleftRA, OUTPUT);
  pinMode(PWMrightFB, OUTPUT);
  pinMode(PWMrightRB, OUTPUT);

  // Enable the motor controller by pulling the disable pin LOW
  pinMode(disablePin, OUTPUT);
  digitalWrite(disablePin, LOW);
}

void loop() {
  // Read joystick inputs and convert to signed values
  throttle = readAxisDelta(joysticYA);
  delayMicroseconds(100);
  direction = readAxisDelta(joysticXA);

  // Mix throttle and direction for differential control
  mixInputs(throttle, direction, leftMotor, rightMotor);

  // Calculate a scale factor so neither channel exceeds PWM range
  maxMotorScale = calculateScaleFactor(leftMotor, rightMotor);

  // Apply scaling and constrain to hardware-safe limits
  leftMotorScaled = scaleAndConstrain(leftMotor, maxMotorScale, -120, 185);  // Left joystick: back x, forward y
  rightMotorScaled = scaleAndConstrain(rightMotor, maxMotorScale, -65, 70);  // Right joystick: back x, forward y

  // Print diagnostic information for tuning
  printMixDebug("L", leftMotor, maxMotorScale, leftMotorScaled);
  printMixDebug("R", rightMotor, maxMotorScale, rightMotorScaled);

  // Drive motors with the scaled outputs
  applyMotorOutput(PWMleftFA, PWMleftRA, leftMotorScaled);
  applyMotorOutput(PWMrightFB, PWMrightRB, rightMotorScaled);

  Serial.println();
  delay(10);
}
