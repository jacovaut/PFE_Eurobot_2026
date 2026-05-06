#include <Arduino.h>
#include <ESP32Encoder.h>
#include <ESP32Servo.h>
#include "pins_pami_ninja.h"
#include "wifi_server_setup.h"

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

void cmdvel_callback(const void* msgin) {
  const geometry_msgs__msg__Twist* msg =
    static_cast<const geometry_msgs__msg__Twist*>(msgin);

  cmdvel.seq++;
  cmdvel.vx = msg->linear.x;
  cmdvel.vy = msg->linear.y;
  cmdvel.w = msg->angular.z;
  lastCmdVelTime = millis();
  cmdvelReceived = true;
  cmdvel.seq++;
}

void keyboard_callback(const void* msgin) {
  const std_msgs__msg__String* msg =
    static_cast<const std_msgs__msg__String*>(msgin);

  if (msg->data.size == 0) {
    return;
  }

  char key = toupper(msg->data.data[0]);
  if (key >= 32 && key <= 126) {
    keyPressed[(uint8_t)key] = true;
    keyLastReceived[(uint8_t)key] = millis();
  }
}

void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);

  if (timer == NULL) {
    return;
  }

  if (cmdvelReceived && (millis() - lastCmdVelTime > CMDVEL_TIMEOUT_MS)) {
    cmdvel.seq++;
    cmdvel.vx = 0.0f;
    cmdvel.vy = 0.0f;
    cmdvel.w = 0.0f;
    cmdvelReceived = false;
    cmdvel.seq++;
    Serial.println("cmd_vel timeout: stopping motors");
  }
}

int chooseControlMethod() {
#if PAMI_CONTROL_METHOD == PAMI_CONTROL_SERIAL
  return PAMI_CONTROL_SERIAL;
#elif PAMI_CONTROL_METHOD == PAMI_CONTROL_ROS
  return PAMI_CONTROL_ROS;
#else
  const unsigned long timeoutMs = 10000;
  const unsigned long start = millis();

  Serial.println("\n========================================");
  Serial.println("PAMI NINJA CONTROL MODE");
  Serial.println("========================================");
  Serial.println("Press S: Serial keyboard only");
  Serial.println("Press R: ROS / micro-ROS WiFi");
  Serial.println("Default: ROS after 10 seconds");
  Serial.println("========================================");

  while (millis() - start < timeoutMs) {
    if (Serial.available()) {
      char key = toupper(Serial.read());
      if (key == 'S') {
        Serial.println("Selected: serial keyboard only");
        return PAMI_CONTROL_SERIAL;
      }
      if (key == 'R') {
        Serial.println("Selected: ROS / micro-ROS WiFi");
        return PAMI_CONTROL_ROS;
      }
    }
    delay(10);
  }

  Serial.println("No mode selected: defaulting to ROS / micro-ROS WiFi");
  return PAMI_CONTROL_ROS;
#endif
}

void setupMicroRos() {
#if PAMI_CONTROL_METHOD != PAMI_CONTROL_SERIAL
  allocator = rcl_get_default_allocator();

  set_microros_wifi_transports(
    MICROROS_WIFI_SSID,
    MICROROS_WIFI_PASSWORD,
    MICROROS_AGENT_IP,
    MICROROS_AGENT_PORT
  );

  Serial.printf(
    "micro-ROS WiFi target: ssid=%s agent=%s:%u\n",
    MICROROS_WIFI_SSID,
    MICROROS_AGENT_IP.toString().c_str(),
    MICROROS_AGENT_PORT
  );
  Serial.println("Waiting for micro-ROS agent...");
  while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
    delay(1000);
  }
  Serial.println("micro-ROS agent connected");

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(
    &node,
    "pami_ninja_node",
    "",
    &support
  ));

  RCCHECK(rclc_subscription_init_default(
    &cmdvel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel_smoothed"
  ));

  keyboard_msg.data.data = keyboard_msg_buffer;
  keyboard_msg.data.size = 0;
  keyboard_msg.data.capacity = sizeof(keyboard_msg_buffer);

  RCCHECK(rclc_subscription_init_default(
    &keyboard_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "pami_ninja/keyboard"
  ));

  RCCHECK(rclc_timer_init_default2(
    &timer,
    &support,
    RCL_MS_TO_NS(100),
    timer_callback,
    true
  ));

  RCCHECK(rclc_executor_init(
    &executor,
    &support.context,
    3,
    &allocator
  ));

  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &cmdvel_sub,
    &cmdvel_msg,
    &cmdvel_callback,
    ON_NEW_DATA
  ));

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &keyboard_sub,
    &keyboard_msg,
    &keyboard_callback,
    ON_NEW_DATA
  ));
#else
  Serial.println("micro-ROS disabled: using serial keyboard control only");
#endif
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

  activeControlMethod = chooseControlMethod();
  printKeyboardHelp();

  if (activeControlMethod == PAMI_CONTROL_ROS) {
    setupMicroRos();
  } else {
    Serial.println("Pami Ninja control method: serial keyboard");
  }

  xTaskCreatePinnedToCore(
    core1,
    "Control",
    12000,
    NULL,
    3,
    &core1_handle,
    1
  );

  if (activeControlMethod == PAMI_CONTROL_ROS) {
    xTaskCreatePinnedToCore(
      core2,
      "ROS",
      6000,
      NULL,
      1,
      &core2_handle,
      0
    );

    Serial.println("Pami Ninja micro-ROS initialized");
  } else {
    Serial.println("Pami Ninja serial control initialized");
  }
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}

void core1(void* pvParameters) {
  (void)pvParameters;

  const TickType_t period = pdMS_TO_TICKS(5);
  TickType_t lastWake = xTaskGetTickCount();
  CmdVel local;
  uint32_t s1;
  uint32_t s2;

  for (;;) {
    processSerialKeyboardInput();

    do {
      s1 = cmdvel.seq;
      local.vx = cmdvel.vx;
      local.vy = cmdvel.vy;
      local.w = cmdvel.w;
      s2 = cmdvel.seq;
    } while (s1 != s2 || (s1 & 1));

    if (serialMovementActive()) {
      applySerialMovementControl();
    } else if (activeControlMethod == PAMI_CONTROL_ROS) {
      setMecanumSpeeds(local.vx, local.vy, local.w);
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

void core2(void* pvParameters) {
  (void)pvParameters;

#if PAMI_CONTROL_METHOD != PAMI_CONTROL_SERIAL
  for (;;) {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, 0));
    vTaskDelay(1);
  }
#else
  vTaskDelete(NULL);
#endif
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
  if (activeControlMethod == PAMI_CONTROL_ROS) {
    Serial.println("ROS cmd_vel is used whenever no serial movement key is active.");
  } else {
    Serial.println("Serial-only mode: release movement keys to stop.");
  }
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
    KEY_SPEED_UP, KEY_SPEED_DOWN
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
    setMecanumSpeeds(currentMaxSpeed / 255.0, 0, 0);
  } else if (keyPressed[(uint8_t)KEY_BACKWARD]) {
    setMecanumSpeeds(-currentMaxSpeed / 255.0, 0, 0);
  } else if (keyPressed[(uint8_t)KEY_LEFT]) {
    setMecanumSpeeds(0, currentMaxSpeed / 255.0, 0);
  } else if (keyPressed[(uint8_t)KEY_RIGHT]) {
    setMecanumSpeeds(0, -currentMaxSpeed / 255.0, 0);
  } else if (keyPressed[(uint8_t)KEY_ROTATE_CW]) {
    setMecanumSpeeds(0, 0, currentOmegaSpeed / 255.0);
  } else if (keyPressed[(uint8_t)KEY_ROTATE_CCW]) {
    setMecanumSpeeds(0, 0, -currentOmegaSpeed / 255.0);
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

  if (keyPressed[(uint8_t)KEY_SPEED_UP] && (now - lastSpeedChangeTime > SPEED_CHANGE_DEBOUNCE)) {
    setSpeed(currentMaxSpeed + SPEED_INCREMENT);
    lastSpeedChangeTime = now;
  }

  if (keyPressed[(uint8_t)KEY_SPEED_DOWN] && (now - lastSpeedChangeTime > SPEED_CHANGE_DEBOUNCE)) {
    setSpeed(currentMaxSpeed - SPEED_INCREMENT);
    lastSpeedChangeTime = now;
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

  float w1 = (vx - vy - (L + W) * omega) / R;
  float w2 = (vx + vy + (L + W) * omega) / R;
  float w3 = (vx + vy - (L + W) * omega) / R;
  float w4 = (vx - vy + (L + W) * omega) / R;

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
      Serial.printf("cmd_vel vx=%.2f vy=%.2f w=%.2f -> FL:%d FR:%d RL:%d RR:%d\n",
                    vx, vy, omega, pwm1, pwm2, pwm3, pwm4);
    }
  }
}

void pickupBlock() {
  Serial.println("PICKUP: Lowering servos...");
  setServo(1, ARM_DOWN_ANGLE);
  setServo(2, ARM_DOWN_ANGLE);
  currentServoAngle = ARM_DOWN_ANGLE;
  delay(ARM_MOVE_DELAY);

  Serial.println("PICKUP: Activating pump...");
  setPump(true);
  delay(300);

  Serial.println("PICKUP: Raising servos...");
  setServo(1, ARM_UP_ANGLE);
  setServo(2, ARM_UP_ANGLE);
  currentServoAngle = ARM_UP_ANGLE;
  delay(ARM_MOVE_DELAY);

  Serial.println("PICKUP: Complete - block secured");
}

void releaseBlock() {
  Serial.println("RELEASE: Lowering servos...");
  setServo(1, ARM_DOWN_ANGLE);
  setServo(2, ARM_DOWN_ANGLE);
  currentServoAngle = ARM_DOWN_ANGLE;
  delay(ARM_MOVE_DELAY);

  Serial.println("RELEASE: Deactivating pump...");
  setPump(false);
  delay(300);

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
