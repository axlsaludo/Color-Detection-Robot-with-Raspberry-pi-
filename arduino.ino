// -------------------------------------
// Arduino Code for Color Detector Robot
// by Logic Laboratories
// -------------------------------------
// This code is for the Arduino board that controls the robot.
// It receives commands from the Raspberry Pi via serial communication.
// The commands are used to control the motors of the robot.
// -------------------------------------

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    // Check if command is followed by a PWM value
    if (command == 'F' || command == 'B') {
      int pwmValue = readPwmValue();  // Function to read PWM value
      if (command == 'F') {
        moveForward(pwmValue);
      } else {
        moveBackward(pwmValue);
      }
    } else if (command == 'L') {
      int pwmValue = readPwmValue();
      turnLeft(pwmValue);
    } else if (command == 'R') {
      int pwmValue = readPwmValue();
      turnRight(pwmValue);
    } else if (command == 'S') {
      stopMotors();
    }

    // Optional: Print status for debugging
    Serial.print("Command received: ");
    Serial.println(command);
  }
}

// Read the PWM value from serial
int readPwmValue() {
  String pwmString = "";
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') break;  // End of the PWM value
    pwmString += c;  // Build the PWM string
  }
  return pwmString.toInt();  // Convert to integer
}

void moveForward(int pwmValue) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  
  analogWrite(PWM1, pwmValue); // Use the received PWM value
  analogWrite(PWM2, pwmValue); // Use the received PWM value
}

void moveBackward(int pwmValue) {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  
  analogWrite(PWM1, pwmValue);
  analogWrite(PWM2, pwmValue);
}

void turnLeft(int pwmValue) {
  analogWrite(PWM1, pwmValue / 2);  // Reduce speed on left motor
  analogWrite(PWM2, pwmValue);      // Full speed on right motor
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void turnRight(int pwmValue) {
  analogWrite(PWM1, pwmValue);
  analogWrite(PWM2, pwmValue / 2);  // Reduce speed on right motor
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}
