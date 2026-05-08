#include <Arduino.h>
#include <ESP32Encoder.h>
#include <ESP32Servo.h>
#include "pins_pami_ninja.h"
#include "serial_keyboard_setup.h"

ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;
ESP32Encoder encoder4;

Servo servo1;
Servo servo2;

int motorSpeeds[4] = {0, 0, 0, 0};
int previousMotorSpeeds[4] = {0, 0, 0, 0};

void error_loop() {
  pinMode(2, OUTPUT);
  while (true) {
    digitalWrite(2, !digitalRead(2));
    delay(100);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_FL_IN1, OUTPUT);
  pinMode(MOTOR_FL_IN2, OUTPUT);
  pinMode(MOTOR_FR_IN1, OUTPUT);
  pinMode(MOTOR_FR_IN2, OUTPUT);
  pinMode(MOTOR_RL_IN1, OUTPUT);
  pinMode(MOTOR_RL_IN2, OUTPUT);
  pinMode(MOTOR_RR_IN1, OUTPUT);
  pinMode(MOTOR_RR_IN2, OUTPUT);

  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);

  pinMode(SOLENOID_PIN, OUTPUT);
  digitalWrite(SOLENOID_PIN, LOW);

  ledcSetup(CH_FL_IN1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_FL_IN2, PWM_FREQ, PWM_RES);
  ledcSetup(CH_FR_IN1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_FR_IN2, PWM_FREQ, PWM_RES);
  ledcSetup(CH_RL_IN1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_RL_IN2, PWM_FREQ, PWM_RES);
  ledcSetup(CH_RR_IN1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_RR_IN2, PWM_FREQ, PWM_RES);

  ledcAttachPin(MOTOR_FL_IN1, CH_FL_IN1);
  ledcAttachPin(MOTOR_FL_IN2, CH_FL_IN2);
  ledcAttachPin(MOTOR_FR_IN1, CH_FR_IN1);
  ledcAttachPin(MOTOR_FR_IN2, CH_FR_IN2);
  ledcAttachPin(MOTOR_RL_IN1, CH_RL_IN1);
  ledcAttachPin(MOTOR_RL_IN2, CH_RL_IN2);
  ledcAttachPin(MOTOR_RR_IN1, CH_RR_IN1);
  ledcAttachPin(MOTOR_RR_IN2, CH_RR_IN2);

  encoder1.attachHalfQuad(ENC_FL_A, ENC_FL_B);
  encoder2.attachHalfQuad(ENC_FR_A, ENC_FR_B);
  encoder3.attachHalfQuad(ENC_RL_A, ENC_RL_B);
  encoder4.attachHalfQuad(ENC_RR_A, ENC_RR_B);

  servo1.attach(SERVO_LEFT_PIN);
  servo2.attach(SERVO_RIGHT_PIN);

  printKeyboardHelp();
  Serial.println("Pami Ninja Test: serial keyboard control only");

  xTaskCreatePinnedToCore(
    core1,
    "Control",
    12000,
    NULL,
    3,
    &core1_handle,
    1
  );
  Serial.println("Pami Ninja serial control initialized");
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}

void core1(void* pvParameters) {
  (void)pvParameters;

  const TickType_t period = pdMS_TO_TICKS(5);
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    processSerialKeyboardInput();

    if (serialMovementActive()) {
      applySerialMovementControl();
    } else {
      setMecanumSpeeds(0, 0, 0);
    }
    applySerialActionControl();

    if (encoderReadingEnabled) {
      static long lastEnc1 = 0, lastEnc2 = 0, lastEnc3 = 0, lastEnc4 = 0;
      long enc1 = encoder1.getCount();
      long enc2 = encoder2.getCount();
      long enc3 = encoder3.getCount();
      long enc4 = encoder4.getCount();

      if (enc1 != lastEnc1 || enc2 != lastEnc2 || enc3 != lastEnc3 || enc4 != lastEnc4) {
        Serial.printf("Encoders: %ld, %ld, %ld, %ld\n", enc1, enc2, enc3, enc4);
        lastEnc1 = enc1;
        lastEnc2 = enc2;
        lastEnc3 = enc3;
        lastEnc4 = enc4;
      }
    }

    vTaskDelayUntil(&lastWake, period);
  }
}

void printKeyboardHelp() {
  Serial.println("\n========================================");
  Serial.println("SERIAL KEYBOARD CONTROL");
  Serial.println("========================================");
  Serial.println("MOVEMENT:");
  Serial.println("  W - Forward");
  Serial.println("  S - Backward");
  Serial.println("  A - Strafe Left");
  Serial.println("  D - Strafe Right");
  Serial.println("  Q - Rotate Counter-Clockwise");
  Serial.println("  E - Rotate Clockwise");
  Serial.println("  SPACE - Stop all motion");
  Serial.println("BLOCK HANDLING:");
  Serial.println("  P - Pick up block");
  Serial.println("  R - Release block");
  Serial.println("SERVO:");
  Serial.println("  U - Raise servos");
  Serial.println("  O - Lower servos");
  Serial.println("  I - Toggle continuous servo sweep");
  Serial.println("PUMP / ENCODERS / SPEED:");
  Serial.println("  J - Toggle pump");
  Serial.println("  N - Toggle encoder print");
  Serial.println("  L - Reset encoders");
  Serial.println("  + - Increase serial speed");
  Serial.println("  - - Decrease serial speed");
  Serial.println("  ] - Increase omega speed");
  Serial.println("  [ - Decrease omega speed");
  Serial.println("Serial-only mode: release movement keys to stop.");
  Serial.println("========================================\n");
}

void processSerialKeyboardInput() {
  unsigned long now = millis();

  while (Serial.available()) {
    char key = toupper(Serial.read());

    if (key >= 32 && key <= 126) {
      keyPressed[(uint8_t)key] = true;
      keyLastReceived[(uint8_t)key] = now;
    }
  }

  const char trackedKeys[] = {
    KEY_FORWARD, KEY_BACKWARD, KEY_LEFT, KEY_RIGHT,
    KEY_ROTATE_CW, KEY_ROTATE_CCW, KEY_STOP,
    KEY_PICK, KEY_RELEASE, KEY_SERVO_UP, KEY_SERVO_DOWN,
    KEY_SERVO_SWEEP, KEY_PUMP_TOGGLE, KEY_ENC_TOGGLE, KEY_ENC_RESET,
    KEY_SPEED_UP, KEY_SPEED_DOWN, KEY_OMEGA_SPEED_UP, KEY_OMEGA_SPEED_DOWN
  };

  for (size_t i = 0; i < sizeof(trackedKeys); i++) {
    uint8_t key = (uint8_t)trackedKeys[i];
    if (keyPressed[key] && (now - keyLastReceived[key] > KEY_TIMEOUT)) {
      keyPressed[key] = false;
    }
  }
}

bool serialMovementActive() {
  return keyPressed[(uint8_t)KEY_FORWARD] ||
         keyPressed[(uint8_t)KEY_BACKWARD] ||
         keyPressed[(uint8_t)KEY_LEFT] ||
         keyPressed[(uint8_t)KEY_RIGHT] ||
         keyPressed[(uint8_t)KEY_ROTATE_CW] ||
         keyPressed[(uint8_t)KEY_ROTATE_CCW] ||
         keyPressed[(uint8_t)KEY_STOP];
}

void applySerialMovementControl() {
  if (keyPressed[(uint8_t)KEY_STOP]) {
    setMecanumSpeeds(0, 0, 0);
  } else if (keyPressed[(uint8_t)KEY_FORWARD]) {
    setMecanumSpeeds(0, currentMaxSpeed / 255.0, 0);
  } else if (keyPressed[(uint8_t)KEY_BACKWARD]) {
    setMecanumSpeeds(0, -currentMaxSpeed / 255.0, 0);
  } else if (keyPressed[(uint8_t)KEY_LEFT]) {
    setMecanumSpeeds(currentMaxSpeed / 255.0, 0, 0);
  } else if (keyPressed[(uint8_t)KEY_RIGHT]) {
    setMecanumSpeeds(-currentMaxSpeed / 255.0, 0, 0);
  } else if (keyPressed[(uint8_t)KEY_ROTATE_CW]) {
    setMecanumSpeeds(0, 0, -currentOmegaSpeed / 255.0);
  } else if (keyPressed[(uint8_t)KEY_ROTATE_CCW]) {
    setMecanumSpeeds(0, 0, currentOmegaSpeed / 255.0);
  }
}

void applySerialActionControl() {
  unsigned long now = millis();

  static unsigned long lastServoUpTime = 0;
  static unsigned long lastServoDownTime = 0;
  static unsigned long lastServoSweepTime = 0;
  const unsigned long SERVO_DEBOUNCE = 200;
  const unsigned long SERVO_SWEEP_DEBOUNCE = 300;

  if (keyPressed[(uint8_t)KEY_SERVO_UP] && (now - lastServoUpTime > SERVO_DEBOUNCE)) {
    servoSweepEnabled = false;
    currentServoAngle = constrain(currentServoAngle + SERVO_INCREMENT, 0, 180);
    setServo(1, currentServoAngle);
    setServo(2, currentServoAngle);
    Serial.printf("Servos UP - Angle: %d\n", currentServoAngle);
    lastServoUpTime = now;
  }

  if (keyPressed[(uint8_t)KEY_SERVO_DOWN] && (now - lastServoDownTime > SERVO_DEBOUNCE)) {
    servoSweepEnabled = false;
    currentServoAngle = constrain(currentServoAngle - SERVO_INCREMENT, 0, 180);
    setServo(1, currentServoAngle);
    setServo(2, currentServoAngle);
    Serial.printf("Servos DOWN - Angle: %d\n", currentServoAngle);
    lastServoDownTime = now;
  }

  if (keyPressed[(uint8_t)KEY_SERVO_SWEEP] && (now - lastServoSweepTime > SERVO_SWEEP_DEBOUNCE)) {
    toggleServoSweep();
    keyPressed[(uint8_t)KEY_SERVO_SWEEP] = false;
    lastServoSweepTime = millis();
  }

  updateServoSweep();

  static unsigned long lastPickTime = 0;
  static unsigned long lastReleaseTime = 0;
  const unsigned long MIN_ACTION_INTERVAL = 2500;

  if (keyPressed[(uint8_t)KEY_PICK] && (now - lastPickTime > MIN_ACTION_INTERVAL)) {
    pickupBlock();
    lastPickTime = now;
  }

  if (keyPressed[(uint8_t)KEY_RELEASE] && (now - lastReleaseTime > MIN_ACTION_INTERVAL)) {
    releaseBlock();
    lastReleaseTime = now;
  }

  static unsigned long lastPumpToggleTime = 0;
  const unsigned long PUMP_TOGGLE_DEBOUNCE = 300;

  if (keyPressed[(uint8_t)KEY_PUMP_TOGGLE] && (now - lastPumpToggleTime > PUMP_TOGGLE_DEBOUNCE)) {
    togglePump();
    lastPumpToggleTime = now;
  }

  static unsigned long lastEncToggleTime = 0;
  const unsigned long ENC_TOGGLE_DEBOUNCE = 300;

  if (keyPressed[(uint8_t)KEY_ENC_TOGGLE] && (now - lastEncToggleTime > ENC_TOGGLE_DEBOUNCE)) {
    toggleEncoderReading();
    lastEncToggleTime = now;
  }

  static unsigned long lastEncResetTime = 0;
  const unsigned long ENC_RESET_DEBOUNCE = 300;

  if (keyPressed[(uint8_t)KEY_ENC_RESET] && (now - lastEncResetTime > ENC_RESET_DEBOUNCE)) {
    resetEncoders();
    lastEncResetTime = now;
  }

  static unsigned long lastSpeedChangeTime = 0;
  static unsigned long lastOmegaSpeedChangeTime = 0;

  if (keyPressed[(uint8_t)KEY_SPEED_UP] && (now - lastSpeedChangeTime > SPEED_CHANGE_DEBOUNCE)) {
    setSpeed(currentMaxSpeed + SPEED_INCREMENT);
    lastSpeedChangeTime = now;
  }

  if (keyPressed[(uint8_t)KEY_SPEED_DOWN] && (now - lastSpeedChangeTime > SPEED_CHANGE_DEBOUNCE)) {
    setSpeed(currentMaxSpeed - SPEED_INCREMENT);
    lastSpeedChangeTime = now;
  }

  if (keyPressed[(uint8_t)KEY_OMEGA_SPEED_UP] && (now - lastOmegaSpeedChangeTime > SPEED_CHANGE_DEBOUNCE)) {
    setOmegaSpeed(currentOmegaSpeed + SPEED_INCREMENT);
    lastOmegaSpeedChangeTime = now;
  }

  if (keyPressed[(uint8_t)KEY_OMEGA_SPEED_DOWN] && (now - lastOmegaSpeedChangeTime > SPEED_CHANGE_DEBOUNCE)) {
    setOmegaSpeed(currentOmegaSpeed - SPEED_INCREMENT);
    lastOmegaSpeedChangeTime = now;
  }
}

void setMotor(int motor, int speed) {
  int ch_in1;
  int ch_in2;

  switch (motor) {
    case 1:
      ch_in1 = CH_FL_IN1; ch_in2 = CH_FL_IN2; break;
    case 2:
      ch_in1 = CH_FR_IN1; ch_in2 = CH_FR_IN2; break;
    case 3:
      ch_in1 = CH_RL_IN1; ch_in2 = CH_RL_IN2; break;
    case 4:
      ch_in1 = CH_RR_IN1; ch_in2 = CH_RR_IN2; break;
    default:
      return;
  }

  if (motorSpeeds[motor - 1] != speed) {
    motorSpeeds[motor - 1] = speed;
    int pwm = abs(speed);

    if (speed > 0) {
      ledcWrite(ch_in1, pwm);
      ledcWrite(ch_in2, 0);
    } else if (speed < 0) {
      ledcWrite(ch_in1, 0);
      ledcWrite(ch_in2, pwm);
    } else {
      ledcWrite(ch_in1, 0);
      ledcWrite(ch_in2, 0);
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

void setMecanumSpeeds(float vx, float vy, float omega) {
  float L = WHEELBASE_LENGTH / 2.0;
  float W = WHEELBASE_WIDTH / 2.0;
  float R = WHEEL_RADIUS;

  float w1 = (vx - vy + (L + W) * omega) / R;
  float w2 = (vx + vy + (L + W) * omega) / R;
  float w3 = (vx + vy - (L + W) * omega) / R;
  float w4 = (vx - vy - (L + W) * omega) / R;

  float scale = currentMaxSpeed / 1.0;

  int pwm1 = constrain((int)round(w1 * scale), -currentMaxSpeed, currentMaxSpeed);
  int pwm2 = constrain((int)round(w2 * scale), -currentMaxSpeed, currentMaxSpeed);
  int pwm3 = constrain((int)round(w3 * scale), -currentMaxSpeed, currentMaxSpeed);
  int pwm4 = constrain((int)round(w4 * scale), -currentMaxSpeed, currentMaxSpeed);

  setMotor(1, pwm1);
  setMotor(2, pwm2);
  setMotor(3, pwm3);
  setMotor(4, pwm4);

  bool anyChanged = (pwm1 != previousMotorSpeeds[0] || pwm2 != previousMotorSpeeds[1] ||
                     pwm3 != previousMotorSpeeds[2] || pwm4 != previousMotorSpeeds[3]);

  if (anyChanged) {
    previousMotorSpeeds[0] = pwm1;
    previousMotorSpeeds[1] = pwm2;
    previousMotorSpeeds[2] = pwm3;
    previousMotorSpeeds[3] = pwm4;

    if (pwm1 == 0 && pwm2 == 0 && pwm3 == 0 && pwm4 == 0) {
      Serial.println("Stopping all motors");
    } else {
      Serial.printf("serial motion vx=%.2f vy=%.2f w=%.2f -> FL:%d FR:%d RL:%d RR:%d\n",
                    vx, vy, omega, pwm1, pwm2, pwm3, pwm4);
    }
  }
}

void pickupBlock() {

  Serial.println("PICKUP: Activating pump...");
  setPump(true);

  Serial.println("PICKUP: Lowering servos...");
  setServo(1, ARM_DOWN_ANGLE);
  setServo(2, ARM_DOWN_ANGLE);
  currentServoAngle = ARM_DOWN_ANGLE;
  delay(ARM_MOVE_DELAY);

  delay(ARM_MOVE_DELAY);

  Serial.println("PICKUP: Raising servos...");
  setServo(1, ARM_UP_ANGLE);
  setServo(2, ARM_UP_ANGLE);
  currentServoAngle = ARM_UP_ANGLE;
  delay(ARM_MOVE_DELAY);

  Serial.println("PICKUP: Complete - block secured");
}

void releaseBlock() {
  Serial.println("RELEASE: Lowering servos...");
  setServo(1, ARM_RELEASE_DOWN_ANGLE);
  setServo(2, ARM_RELEASE_DOWN_ANGLE);
  currentServoAngle = ARM_RELEASE_DOWN_ANGLE;
  delay(ARM_MOVE_DELAY);

  Serial.println("RELEASE: Deactivating pump...");
  setPump(false);
  delay(ARM_MOVE_DELAY);

  Serial.println("RELEASE: Raising servos...");
  setServo(1, ARM_UP_ANGLE);
  setServo(2, ARM_UP_ANGLE);
  currentServoAngle = ARM_UP_ANGLE;
  delay(ARM_MOVE_DELAY);

  Serial.println("RELEASE: Complete - block released");
}

void toggleServoSweep() {
  const int minAngle = ARM_DOWN_ANGLE < ARM_UP_ANGLE ? ARM_DOWN_ANGLE : ARM_UP_ANGLE;
  const int maxAngle = ARM_DOWN_ANGLE > ARM_UP_ANGLE ? ARM_DOWN_ANGLE : ARM_UP_ANGLE;

  servoSweepEnabled = !servoSweepEnabled;

  if (servoSweepEnabled) {
    currentServoAngle = constrain(currentServoAngle, minAngle, maxAngle);
    servoSweepDirection = currentServoAngle >= maxAngle ? -1 : 1;
    lastServoSweepStepTime = 0;
    Serial.printf("SERVO SWEEP: ON (%d <-> %d)\n", minAngle, maxAngle);
  } else {
    Serial.printf("SERVO SWEEP: OFF at %d\n", currentServoAngle);
  }
}

void updateServoSweep() {
  if (!servoSweepEnabled) {
    return;
  }

  unsigned long now = millis();
  if (now - lastServoSweepStepTime < ARM_SWEEP_DELAY) {
    return;
  }

  const int minAngle = ARM_DOWN_ANGLE < ARM_UP_ANGLE ? ARM_DOWN_ANGLE : ARM_UP_ANGLE;
  const int maxAngle = ARM_DOWN_ANGLE > ARM_UP_ANGLE ? ARM_DOWN_ANGLE : ARM_UP_ANGLE;
  const int step = ARM_SWEEP_STEP > 0 ? ARM_SWEEP_STEP : 1;

  currentServoAngle += servoSweepDirection * step;

  if (currentServoAngle >= maxAngle) {
    currentServoAngle = maxAngle;
    servoSweepDirection = -1;
  } else if (currentServoAngle <= minAngle) {
    currentServoAngle = minAngle;
    servoSweepDirection = 1;
  }

  setServo(1, currentServoAngle);
  setServo(2, currentServoAngle);
  lastServoSweepStepTime = now;
}

void togglePump() {
  pumpState = !pumpState;
  setPump(pumpState);
  Serial.printf("PUMP: %s\n", pumpState ? "ON" : "OFF");
}

void toggleEncoderReading() {
  encoderReadingEnabled = !encoderReadingEnabled;
  Serial.printf("ENCODER READING: %s\n", encoderReadingEnabled ? "ON" : "OFF");
}

void setSpeed(int speed) {
  currentMaxSpeed = constrain(speed, MIN_SPEED, MAX_SPEED_LIMIT);
  Serial.printf("SPEED: %d/255\n", currentMaxSpeed);
}

void setOmegaSpeed(int speed) {
  currentOmegaSpeed = constrain(speed, MIN_SPEED, MAX_SPEED_LIMIT);
  Serial.printf("OMEGA SPEED: %d/255\n", currentOmegaSpeed);
}
