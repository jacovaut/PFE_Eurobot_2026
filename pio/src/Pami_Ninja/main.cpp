#include <Arduino.h>
#include <ESP32Encoder.h>
#include <ESP32Servo.h>
#include "pins_pami_ninja.h"

// Encoder objects
ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;
ESP32Encoder encoder4;

// Servo objects
Servo servo1;
Servo servo2;

// Track motor speeds
int motorSpeeds[4] = {0, 0, 0, 0};
int previousMotorSpeeds[4] = {0, 0, 0, 0};  // Track previous speeds for change detection

// Valve and pump states
bool valveState = false;
bool pumpState = false;

// Encoder reading control
bool encoderReadingEnabled = false;

// Servo angle tracking for incremental movement
int currentServoAngle = 90;  // Start at middle position
const int SERVO_INCREMENT = 10;  // Degrees per increment
const unsigned long SERVO_UPDATE_PERIOD = 50;  // milliseconds between angle updates
unsigned long lastServoUpdateTime = 0;

// Speed control for robot movement
int currentMaxSpeed = 100;  // Start at default speed for linear motion (vx, vy)
int currentOmegaSpeed = 90;  // Separate speed for rotational motion (omega)
const int MIN_SPEED = 50;  // Minimum speed
const int MAX_SPEED_LIMIT = MAX_SPEED;  // Maximum speed (PWM limit)
const int SPEED_INCREMENT = 5;  // Speed change per keypress
unsigned long lastSpeedChangeTime = 0;
const unsigned long SPEED_CHANGE_DEBOUNCE = 200;  // milliseconds between speed changes

// Keyboard control - track currently pressed keys
bool keyPressed[256] = {false};  // Track state of each ASCII key
unsigned long keyLastReceived[256] = {0};  // Track last time each key was received

// Key press/release timeout (in milliseconds)
#define KEY_TIMEOUT 150  // If key not received for 150ms, consider it released


// Functions declaration
void setMecanumSpeeds(float vx, float vy, float omega);
void setMotor(int motor, int speed);
void setServo(int servo, int angle);
void setPump(bool state);
void setValve(bool state);
void resetEncoders();
void printKeyboardHelp();
void processKeyboardInput();
void pickupBlock();
void releaseBlock();
void togglePump();
void toggleEncoderReading();
void setSpeed(int speed);


void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(MOTOR_FL_IN1, OUTPUT);
  pinMode(MOTOR_FL_IN2, OUTPUT);
  pinMode(MOTOR_FR_IN1, OUTPUT);
  pinMode(MOTOR_FR_IN2, OUTPUT);
  pinMode(MOTOR_RL_IN1, OUTPUT);
  pinMode(MOTOR_RL_IN2, OUTPUT);
  pinMode(MOTOR_RR_IN1, OUTPUT);
  pinMode(MOTOR_RR_IN2, OUTPUT);

  // Pump
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);

  // Valve (Solenoid)
  pinMode(SOLENOID_PIN, OUTPUT);
  digitalWrite(SOLENOID_PIN, LOW);

  // PWM setup for all 8 motor control pins (IN1 and IN2 for each motor)
  ledcSetup(CH_FL_IN1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_FL_IN2, PWM_FREQ, PWM_RES);
  ledcSetup(CH_FR_IN1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_FR_IN2, PWM_FREQ, PWM_RES);
  ledcSetup(CH_RL_IN1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_RL_IN2, PWM_FREQ, PWM_RES);
  ledcSetup(CH_RR_IN1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_RR_IN2, PWM_FREQ, PWM_RES);

  // Attach PWM pins for both IN1 and IN2
  ledcAttachPin(MOTOR_FL_IN1, CH_FL_IN1);
  ledcAttachPin(MOTOR_FL_IN2, CH_FL_IN2);
  ledcAttachPin(MOTOR_FR_IN1, CH_FR_IN1);
  ledcAttachPin(MOTOR_FR_IN2, CH_FR_IN2);
  ledcAttachPin(MOTOR_RL_IN1, CH_RL_IN1);
  ledcAttachPin(MOTOR_RL_IN2, CH_RL_IN2);
  ledcAttachPin(MOTOR_RR_IN1, CH_RR_IN1);
  ledcAttachPin(MOTOR_RR_IN2, CH_RR_IN2);

  // Encoders
  encoder1.attachHalfQuad(ENC_FL_A, ENC_FL_B);
  encoder2.attachHalfQuad(ENC_FR_A, ENC_FR_B);
  encoder3.attachHalfQuad(ENC_RL_A, ENC_RL_B);
  encoder4.attachHalfQuad(ENC_RR_A, ENC_RR_B);

  // Servos
  servo1.attach(SERVO_LEFT_PIN);
  servo2.attach(SERVO_RIGHT_PIN);
  Serial.println("Robot initialized");
}


void loop() {
  // Print encoder values (if enabled) - only display if values changed
  if (encoderReadingEnabled) {
    static long lastEnc1 = 0, lastEnc2 = 0, lastEnc3 = 0, lastEnc4 = 0;
    
    long enc1 = encoder1.getCount();
    long enc2 = encoder2.getCount();
    long enc3 = encoder3.getCount();
    long enc4 = encoder4.getCount();
    
    if (enc1 != lastEnc1 || enc2 != lastEnc2 || enc3 != lastEnc3 || enc4 != lastEnc4) {
      Serial.print("Encoders: ");
      Serial.print(enc1);
      Serial.print(", ");
      Serial.print(enc2);
      Serial.print(", ");
      Serial.print(enc3);
      Serial.print(", ");
      Serial.println(enc4);
      
      lastEnc1 = enc1;
      lastEnc2 = enc2;
      lastEnc3 = enc3;
      lastEnc4 = enc4;
    }
  }

  // Process keyboard input - hold keys for continuous movement
  processKeyboardInput();

  delay(10);
}

void setMotor(int motor, int speed) {
  int in1, in2, ch_in1, ch_in2;

  switch (motor) {
    case 1:
      in1 = MOTOR_FL_IN1; in2 = MOTOR_FL_IN2; ch_in1 = CH_FL_IN1; ch_in2 = CH_FL_IN2; break;
    case 2:
      in1 = MOTOR_FR_IN1; in2 = MOTOR_FR_IN2; ch_in1 = CH_FR_IN1; ch_in2 = CH_FR_IN2; break;
    case 3:
      in1 = MOTOR_RL_IN1; in2 = MOTOR_RL_IN2; ch_in1 = CH_RL_IN1; ch_in2 = CH_RL_IN2; break;
    case 4:
      in1 = MOTOR_RR_IN1; in2 = MOTOR_RR_IN2; ch_in1 = CH_RR_IN1; ch_in2 = CH_RR_IN2; break;
    default:
      return;
  }

  // Only update and print if speed changed
  if (motorSpeeds[motor - 1] != speed) {
    motorSpeeds[motor - 1] = speed;
    int pwm = abs(speed);

    if (speed > 0) {
      // Forward: PWM on IN1, LOW on IN2
      ledcWrite(ch_in1, pwm);    // Write PWM to IN1 channel
      ledcWrite(ch_in2, 0);       // Set IN2 to 0
      Serial.printf("Motor %d - IN1: %d, IN2: 0\n", motor, pwm);

    } else if (speed < 0) {
      // Reverse: PWM on IN2, LOW on IN1
      ledcWrite(ch_in1, 0);       // Set IN1 to 0
      ledcWrite(ch_in2, pwm);     // Write PWM to IN2 channel
      Serial.printf("Motor %d - IN1: 0, IN2: %d\n", motor, pwm);

    } else {
      // Stop: PWM to 0, both pins LOW
      ledcWrite(ch_in1, 0);       // Set IN1 to 0
      ledcWrite(ch_in2, 0);       // Set IN2 to 0
    }
  }
}

void setServo(int servo, int angle) {
  if (servo == 1) {
    servo1.write(angle);
  } else if (servo == 2) {
    servo2.write(angle);
  }
}

void setPump(bool state) {
  pumpState = state;
  digitalWrite(PUMP_PIN, state ? HIGH : LOW);
}

void setValve(bool state) {
  valveState = state;
  digitalWrite(SOLENOID_PIN, state ? HIGH : LOW);
}

void resetEncoders() {
  encoder1.clearCount();
  encoder2.clearCount();
  encoder3.clearCount();
  encoder4.clearCount();
  Serial.println("All encoders reset");
}

void printKeyboardHelp() {
  Serial.println("\n========================================");
  Serial.println("KEYBOARD CONTROL - Hold keys to move");
  Serial.println("========================================");
  Serial.println("MOVEMENT:");
  Serial.println("  W - Forward");
  Serial.println("  S - Backward");
  Serial.println("  A - Strafe Left");
  Serial.println("  D - Strafe Right");
  Serial.println("  Q - Rotate Counter-Clockwise");
  Serial.println("  E - Rotate Clockwise");
  Serial.println("  SPACE - Stop all motion");
  Serial.println("\nBLOCK HANDLING:");
  Serial.println("  P - Pick up block (macro: lower → pump ON → raise)");
  Serial.println("  R - Release block (macro: lower → pump OFF → raise)");
  Serial.println("\nSERVO CONTROL:");
  Serial.println("  U - Raise servos (hold)");
  Serial.println("  O - Lower servos (hold)");
  Serial.println("\nPUMP & SOLENOID:");
  Serial.println("  J - Toggle Pump ON/OFF");
  Serial.println("\nSPEED CONTROL:");
  Serial.println("  + - Increase linear speed (50-255) [W/S/A/D]");
  Serial.println("  - - Decrease linear speed (50-255) [W/S/A/D]");
  Serial.println("  Note: Rotation speed (Q/E) is set separately to 150");
  Serial.println("\nENCODER CONTROL:");
  Serial.println("  N - Toggle Encoder Reading ON/OFF");
  Serial.println("  L - Reset All Encoders");
  Serial.println("========================================\n");
}

void processKeyboardInput() {
  unsigned long now = millis();
  
  // Check if there's data available to read
  if (Serial.available()) {
    char key = Serial.read();
    key = toupper(key);  // Convert to uppercase
    
    if (key >= 32 && key <= 126) {  // Valid ASCII range
      keyPressed[key] = true;  // Mark key as pressed
      keyLastReceived[key] = now;  // Update timestamp
    }
  }
  
  // Check for key timeouts (key release detection) for all keys
  if (keyPressed[KEY_FORWARD] && (now - keyLastReceived[KEY_FORWARD] > KEY_TIMEOUT)) {
    keyPressed[KEY_FORWARD] = false;
  }
  if (keyPressed[KEY_BACKWARD] && (now - keyLastReceived[KEY_BACKWARD] > KEY_TIMEOUT)) {
    keyPressed[KEY_BACKWARD] = false;
  }
  if (keyPressed[KEY_LEFT] && (now - keyLastReceived[KEY_LEFT] > KEY_TIMEOUT)) {
    keyPressed[KEY_LEFT] = false;
  }
  if (keyPressed[KEY_RIGHT] && (now - keyLastReceived[KEY_RIGHT] > KEY_TIMEOUT)) {
    keyPressed[KEY_RIGHT] = false;
  }
  if (keyPressed[KEY_ROTATE_CW] && (now - keyLastReceived[KEY_ROTATE_CW] > KEY_TIMEOUT)) {
    keyPressed[KEY_ROTATE_CW] = false;
  }
  if (keyPressed[KEY_ROTATE_CCW] && (now - keyLastReceived[KEY_ROTATE_CCW] > KEY_TIMEOUT)) {
    keyPressed[KEY_ROTATE_CCW] = false;
  }
  if (keyPressed[KEY_SERVO_UP] && (now - keyLastReceived[KEY_SERVO_UP] > KEY_TIMEOUT)) {
    keyPressed[KEY_SERVO_UP] = false;
  }
  if (keyPressed[KEY_SERVO_DOWN] && (now - keyLastReceived[KEY_SERVO_DOWN] > KEY_TIMEOUT)) {
    keyPressed[KEY_SERVO_DOWN] = false;
  }
  if (keyPressed[KEY_PICK] && (now - keyLastReceived[KEY_PICK] > KEY_TIMEOUT)) {
    keyPressed[KEY_PICK] = false;
  }
  if (keyPressed[KEY_RELEASE] && (now - keyLastReceived[KEY_RELEASE] > KEY_TIMEOUT)) {
    keyPressed[KEY_RELEASE] = false;
  }
  if (keyPressed[KEY_PUMP_TOGGLE] && (now - keyLastReceived[KEY_PUMP_TOGGLE] > KEY_TIMEOUT)) {
    keyPressed[KEY_PUMP_TOGGLE] = false;
  }
  if (keyPressed[KEY_ENC_TOGGLE] && (now - keyLastReceived[KEY_ENC_TOGGLE] > KEY_TIMEOUT)) {
    keyPressed[KEY_ENC_TOGGLE] = false;
  }
  if (keyPressed[KEY_ENC_RESET] && (now - keyLastReceived[KEY_ENC_RESET] > KEY_TIMEOUT)) {
    keyPressed[KEY_ENC_RESET] = false;
  }
  if (keyPressed[KEY_SPEED_UP] && (now - keyLastReceived[KEY_SPEED_UP] > KEY_TIMEOUT)) {
    keyPressed[KEY_SPEED_UP] = false;
  }
  if (keyPressed[KEY_SPEED_DOWN] && (now - keyLastReceived[KEY_SPEED_DOWN] > KEY_TIMEOUT)) {
    keyPressed[KEY_SPEED_DOWN] = false;
  }
  
  // Process movement keys
  if (keyPressed[KEY_FORWARD]) {
    setMecanumSpeeds(currentMaxSpeed / 255.0, 0, 0);
  }
  else if (keyPressed[KEY_BACKWARD]) {
    setMecanumSpeeds(-currentMaxSpeed / 255.0, 0, 0);
  }
  else if (keyPressed[KEY_LEFT]) {
    setMecanumSpeeds(0, currentMaxSpeed / 255.0, 0);
  }
  else if (keyPressed[KEY_RIGHT]) {
    setMecanumSpeeds(0, -currentMaxSpeed / 255.0, 0);
  }
  else if (keyPressed[KEY_ROTATE_CW]) {
    setMecanumSpeeds(0, 0, currentOmegaSpeed / 255.0);
  }
  else if (keyPressed[KEY_ROTATE_CCW]) {
    setMecanumSpeeds(0, 0, -currentOmegaSpeed / 255.0);
  }
  else {
    setMecanumSpeeds(0, 0, 0);
  }

  // Servo control - one-shot per key press with debounce
  static unsigned long lastServoUpTime = 0;
  static unsigned long lastServoDownTime = 0;
  const unsigned long SERVO_DEBOUNCE = 200;  // milliseconds between servo movements

  if (keyPressed[KEY_SERVO_UP] && (now - lastServoUpTime > SERVO_DEBOUNCE)) {
    currentServoAngle = constrain(currentServoAngle + SERVO_INCREMENT, 0, 180);
    setServo(1, currentServoAngle);
    setServo(2, currentServoAngle);
    Serial.printf("Servos UP - Angle: %d\n", currentServoAngle);
    lastServoUpTime = now;
  }

  if (keyPressed[KEY_SERVO_DOWN] && (now - lastServoDownTime > SERVO_DEBOUNCE)) {
    currentServoAngle = constrain(currentServoAngle - SERVO_INCREMENT, 0, 180);
    setServo(1, currentServoAngle);
    setServo(2, currentServoAngle);
    Serial.printf("Servos DOWN - Angle: %d\n", currentServoAngle);
    lastServoDownTime = now;
  }

  // Block handling - one-shot execution
  static unsigned long lastPickTime = 0;
  static unsigned long lastReleaseTime = 0;
  const unsigned long MIN_ACTION_INTERVAL = 2500;  // Minimum time between pick/release actions

  if (keyPressed[KEY_PICK] && (now - lastPickTime > MIN_ACTION_INTERVAL)) {
    pickupBlock();
    lastPickTime = now;
  }

  if (keyPressed[KEY_RELEASE] && (now - lastReleaseTime > MIN_ACTION_INTERVAL)) {
    releaseBlock();
    lastReleaseTime = now;
  }

  // Pump toggle
  static unsigned long lastPumpToggleTime = 0;
  const unsigned long PUMP_TOGGLE_DEBOUNCE = 300;

  if (keyPressed[KEY_PUMP_TOGGLE] && (now - lastPumpToggleTime > PUMP_TOGGLE_DEBOUNCE)) {
    togglePump();
    lastPumpToggleTime = now;
  }

  // Encoder reading toggle
  static unsigned long lastEncToggleTime = 0;
  const unsigned long ENC_TOGGLE_DEBOUNCE = 300;

  if (keyPressed[KEY_ENC_TOGGLE] && (now - lastEncToggleTime > ENC_TOGGLE_DEBOUNCE)) {
    toggleEncoderReading();
    lastEncToggleTime = now;
  }

  // Encoder reset
  static unsigned long lastEncResetTime = 0;
  const unsigned long ENC_RESET_DEBOUNCE = 300;

  if (keyPressed[KEY_ENC_RESET] && (now - lastEncResetTime > ENC_RESET_DEBOUNCE)) {
    resetEncoders();
    lastEncResetTime = now;
  }

  // Speed control
  if (keyPressed[KEY_SPEED_UP] && (now - lastSpeedChangeTime > SPEED_CHANGE_DEBOUNCE)) {
    setSpeed(currentMaxSpeed + SPEED_INCREMENT);
    lastSpeedChangeTime = now;
  }

  if (keyPressed[KEY_SPEED_DOWN] && (now - lastSpeedChangeTime > SPEED_CHANGE_DEBOUNCE)) {
    setSpeed(currentMaxSpeed - SPEED_INCREMENT);
    lastSpeedChangeTime = now;
  }
}

void setMecanumSpeeds(float vx, float vy, float omega) {
  float L = WHEELBASE_LENGTH / 2.0;
  float W = WHEELBASE_WIDTH / 2.0;
  float R = WHEEL_RADIUS;
 
  float w1 = (vx - vy - (L + W) * omega) / R;
  float w2 = (vx + vy + (L + W) * omega) / R;
  float w3 = (vx + vy - (L + W) * omega) / R;
  float w4 = (vx - vy + (L + W) * omega) / R;
 
  float scale = currentMaxSpeed / 1.0;

  // Use proper rounding instead of truncation
  int pwm1 = constrain((int)round(w1 * scale), -currentMaxSpeed, currentMaxSpeed);
  int pwm2 = constrain((int)round(w2 * scale), -currentMaxSpeed, currentMaxSpeed);
  int pwm3 = constrain((int)round(w3 * scale), -currentMaxSpeed, currentMaxSpeed);
  int pwm4 = constrain((int)round(w4 * scale), -currentMaxSpeed, currentMaxSpeed);

  setMotor(1, pwm1);
  setMotor(2, pwm2);
  setMotor(3, pwm3);
  setMotor(4, pwm4);
  
  // Only print summary if any motor speed changed
  bool anyChanged = (pwm1 != previousMotorSpeeds[0] || pwm2 != previousMotorSpeeds[1] || 
                     pwm3 != previousMotorSpeeds[2] || pwm4 != previousMotorSpeeds[3]);
  
  if (anyChanged) {
    previousMotorSpeeds[0] = pwm1;
    previousMotorSpeeds[1] = pwm2;
    previousMotorSpeeds[2] = pwm3;
    previousMotorSpeeds[3] = pwm4;
     if(vx != 0 || vy != 0 || omega != 0) {
      Serial.printf("Raw wheel speeds: w1=%.3f, w2=%.3f, w3=%.3f, w4=%.3f\n", w1, w2, w3, w4);
      Serial.printf("Calculating speeds for vx=%.2f, vy=%.2f, omega=%.2f\n", vx, vy, omega);
    }
    
    if(pwm1 == 0 && pwm2 == 0 && pwm3 == 0 && pwm4 == 0) {
      Serial.println("Stopping all motors");
    } else {
      Serial.printf("Setting motors - FL: %d, FR: %d, RL: %d, RR: %d\n", pwm1, pwm2, pwm3, pwm4);
    }
  }
}

// ============================================================
//  PUMP AND BLOCK HANDLING MACROS
// ============================================================

/**
 * Pick up block sequence:
 * 1. Lower servos to ARM_DOWN_ANGLE
 * 2. Wait for ARM_MOVE_DELAY for servos to reach position
 * 3. Activate pump
 * 4. Wait for pump to suction (PUMP_SUCTION_TIME)
 * 5. Raise servos to ARM_UP_ANGLE
 */
void pickupBlock() {
  Serial.println("PICKUP: Lowering servos...");
  setServo(1, ARM_DOWN_ANGLE);
  setServo(2, ARM_DOWN_ANGLE);
  currentServoAngle = ARM_DOWN_ANGLE;  // Update tracked angle
  delay(ARM_MOVE_DELAY);

  Serial.println("PICKUP: Activating pump...");
  setPump(true);
  delay(300);  // Wait for pump to build suction

  Serial.println("PICKUP: Raising servos...");
  setServo(1, ARM_UP_ANGLE);
  setServo(2, ARM_UP_ANGLE);
  currentServoAngle = ARM_UP_ANGLE;  // Update tracked angle
  delay(ARM_MOVE_DELAY);

  Serial.println("PICKUP: Complete - block secured");
}

/**
 * Release block sequence:
 * 1. Lower servos to ARM_DOWN_ANGLE
 * 2. Wait for ARM_MOVE_DELAY for servos to reach position
 * 3. Deactivate pump
 * 4. Wait for block to drop (PUMP_RELEASE_WAIT)
 * 5. Raise servos to ARM_UP_ANGLE
 */
void releaseBlock() {
  Serial.println("RELEASE: Lowering servos...");
  setServo(1, ARM_DOWN_ANGLE);
  setServo(2, ARM_DOWN_ANGLE);
  currentServoAngle = ARM_DOWN_ANGLE;  // Update tracked angle
  delay(ARM_MOVE_DELAY);

  Serial.println("RELEASE: Deactivating pump...");
  setPump(false);
  delay(300);  // Wait for block to drop

  Serial.println("RELEASE: Raising servos...");
  setServo(1, ARM_UP_ANGLE);
  setServo(2, ARM_UP_ANGLE);
  currentServoAngle = ARM_UP_ANGLE;  // Update tracked angle
  delay(ARM_MOVE_DELAY);

  Serial.println("RELEASE: Complete - block released");
}

/**
 * Toggle pump on/off and print status
 */
void togglePump() {
  pumpState = !pumpState;
  setPump(pumpState);
  Serial.printf("PUMP: %s\n", pumpState ? "ON" : "OFF");
}

/**
 * Toggle encoder reading on/off and print status
 */
void toggleEncoderReading() {
  encoderReadingEnabled = !encoderReadingEnabled;
  Serial.printf("ENCODER READING: %s\n", encoderReadingEnabled ? "ON" : "OFF");
}

/**
 * Set the maximum speed for robot movement
 * Constrains speed between MIN_SPEED and MAX_SPEED_LIMIT
 */
void setSpeed(int speed) {
  currentMaxSpeed = constrain(speed, MIN_SPEED, MAX_SPEED_LIMIT);
  Serial.printf("SPEED: %d/255\n", currentMaxSpeed);
}