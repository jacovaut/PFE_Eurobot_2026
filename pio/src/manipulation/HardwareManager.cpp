#include "HardwareManager.h"

#include <ESP32PWM.h>

namespace {
constexpr uint32_t kMotionTimeoutMs = 10000;
constexpr int MOTOR_TURN1 = 2;
constexpr int MOTOR_TURN2 = 15;
constexpr int MOTOR_LIFT1 = 5;
constexpr int MOTOR_LIFT2 = 4;

constexpr int ENCODER_LIFT_A = 19;
constexpr int ENCODER_LIFT_B = 21;
constexpr int ENCODER_LIFT_X = 18;
constexpr int ENCODER_TURN_A = 23;
constexpr int ENCODER_TURN_B = 22;
constexpr int ENCODER_TURN_X = 32;

constexpr int PWM_CHANNEL_LIFT1 = 4;
constexpr int PWM_CHANNEL_LIFT2 = 5;
constexpr int PWM_CHANNEL_TURN1 = 6;
constexpr int PWM_CHANNEL_TURN2 = 7;
}

constexpr int HardwareManager::PUMP_PINS[4];

HardwareManager::HardwareManager(BlockManager* blockList)
    : BlockList(blockList),
      armMotor(
          1.0f, 0.0f, 0.0f,
          MOTOR_LIFT1, MOTOR_LIFT2,
          ENCODER_LIFT_A, ENCODER_LIFT_B, ENCODER_LIFT_X,
          PWM_CHANNEL_LIFT1, PWM_CHANNEL_LIFT2),
      turnMotor(
          0.8f, 0.05f, 0.01f,
          MOTOR_TURN1, MOTOR_TURN2,
          ENCODER_TURN_A, ENCODER_TURN_B, ENCODER_TURN_X,
          PWM_CHANNEL_TURN1, PWM_CHANNEL_TURN2) {}

void HardwareManager::init() {
    if (initialized) {
        return;
    }

    for (int pin : PUMP_PINS) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    stopperServo.setPeriodHertz(50);
    flipperServo.setPeriodHertz(50);
    stopperServo.attach(STOP_PIN, 500, 2400);
    flipperServo.attach(FLIP_PIN, 500, 2400);
    setStopperPosition(false);
    setFlipperStop();

    armMotor.setup();
    turnMotor.setup();
    armMotor.disable();
    turnMotor.disable();

    initialized = true;
}

void HardwareManager::updateMotors() {
    armMotor.update();
    turnMotor.update();
}

bool HardwareManager::pauseWithUpdate(uint32_t duration_ms) {
    const uint32_t start = millis();
    while (millis() - start < duration_ms) {
        updateMotors();
        delay(2);
    }
    return true;
}

bool HardwareManager::waitForArmTarget(uint32_t timeout_ms) {
    const uint32_t start = millis();
    while (!armMotor.isAtTarget()) {
        updateMotors();
        if (millis() - start > timeout_ms) {
            return false;
        }
        delay(2);
    }
    return true;
}

bool HardwareManager::waitForTurnTarget(uint32_t timeout_ms) {
    const uint32_t start = millis();
    while (!turnMotor.isAtTarget()) {
        updateMotors();
        if (millis() - start > timeout_ms) {
            return false;
        }
        delay(2);
    }
    return true;
}

void HardwareManager::setPumpState(uint8_t pump_number, bool on) {
    if (pump_number < 1 || pump_number > 4) {
        return;
    }
    digitalWrite(PUMP_PINS[pump_number - 1], on ? HIGH : LOW);
}

void HardwareManager::allPumpsOff() {
    for (uint8_t pump = 1; pump <= 4; ++pump) {
        setPumpState(pump, false);
    }
}

void HardwareManager::setStopperPosition(bool open) {
    stopperServo.write(open ? STOP_OPEN_ANGLE : STOP_CLOSED_ANGLE);
}

void HardwareManager::setFlipperStop() {
    flipperServo.write(FLIP_STOP);
}

bool HardwareManager::runPickupSequence(const bool requested_pumps[4]) {
    armMotor.resetEncoder();
    turnMotor.resetEncoder();
    armMotor.enable();
    turnMotor.enable();

    armMotor.setTarget(500);
    if (!waitForArmTarget(kMotionTimeoutMs) || !pauseWithUpdate(500)) {
        return false;
    }

    turnMotor.setTarget(450);
    if (!waitForTurnTarget(kMotionTimeoutMs) || !pauseWithUpdate(500)) {
        return false;
    }

    armMotor.setTarget(1100);
    if (!waitForArmTarget(kMotionTimeoutMs) || !pauseWithUpdate(500)) {
        return false;
    }

    turnMotor.setTarget(1050);
    if (!waitForTurnTarget(kMotionTimeoutMs) || !pauseWithUpdate(500)) {
        return false;
    }

    armMotor.setTarget(1350);
    if (!waitForArmTarget(kMotionTimeoutMs) || !pauseWithUpdate(500)) {
        return false;
    }

    for (int i = 0; i < 4; ++i) {
        if (requested_pumps[i]) {
            setPumpState(i + 1, true);
        }
    }
    if (!pauseWithUpdate(100)) {
        return false;
    }

    armMotor.setTarget(1450);
    if (!waitForArmTarget(kMotionTimeoutMs) || !pauseWithUpdate(1000)) {
        return false;
    }

    armMotor.setTarget(1000);
    if (!waitForArmTarget(kMotionTimeoutMs) || !pauseWithUpdate(500)) {
        return false;
    }

    turnMotor.setTarget(800);
    if (!waitForTurnTarget(kMotionTimeoutMs) || !pauseWithUpdate(500)) {
        return false;
    }

    armMotor.setTarget(800);
    if (!waitForArmTarget(kMotionTimeoutMs) || !pauseWithUpdate(500)) {
        return false;
    }

    turnMotor.setTarget(500);
    if (!waitForTurnTarget(kMotionTimeoutMs) || !pauseWithUpdate(500)) {
        return false;
    }

    armMotor.setTarget(500);
    if (!waitForArmTarget(kMotionTimeoutMs) || !pauseWithUpdate(500)) {
        return false;
    }

    turnMotor.setTarget(0);
    if (!waitForTurnTarget(kMotionTimeoutMs) || !pauseWithUpdate(500)) {
        return false;
    }

    armMotor.setTarget(200);
    if (!waitForArmTarget(kMotionTimeoutMs)) {
        return false;
    }

    allPumpsOff();
    if (!pauseWithUpdate(500)) {
        return false;
    }

    armMotor.setTarget(0);
    if (!waitForArmTarget(kMotionTimeoutMs)) {
        return false;
    }

    armMotor.disable();
    turnMotor.disable();
    return true;
}

bool HardwareManager::runFlipBlue() {
    setStopperPosition(true);
    pauseWithUpdate(1000);
    setStopperPosition(false);
    pauseWithUpdate(1000);
    flipperServo.write(FLIP_COLOR_START);
    pauseWithUpdate(1000);
    flipperServo.write(FLIP_B_END_COLOR);
    pauseWithUpdate(1000);
    setFlipperStop();
    pauseWithUpdate(1000);
    return true;
}

bool HardwareManager::runFlipYellow() {
    setStopperPosition(true);
    pauseWithUpdate(1000);
    setStopperPosition(false);
    pauseWithUpdate(1000);
    flipperServo.write(FLIP_COLOR_START);
    pauseWithUpdate(1000);
    flipperServo.write(FLIP_Y_END_COLOR);
    pauseWithUpdate(1000);
    setFlipperStop();
    pauseWithUpdate(1000);
    return true;
}

bool HardwareManager::pickUp(const int* cups) {
    bool requested_pumps[4] = {false, false, false, false};
    bool any = false;
    for (int i = 0; i < 4; ++i) {
        requested_pumps[i] = cups[i] != 0;
        any = any || requested_pumps[i];
    }
    if (!any) {
        return false;
    }
    return runPickupSequence(requested_pumps);
}

bool HardwareManager::flip(const int* colors) {
    bool any = false;
    for (int i = 0; i < 4; ++i) {
        if (colors[i] == 1) {
            any = true;
            if (!runFlipBlue()) {
                return false;
            }
        } else if (colors[i] == 2) {
            any = true;
            if (!runFlipYellow()) {
                return false;
            }
        }
    }
    return any;
}

bool HardwareManager::dropOff() {
    allPumpsOff();
    return true;
}