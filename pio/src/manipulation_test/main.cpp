/*
 * Manipulation Test Serial Commands
 * -----------------------------------------------
 * Commands:
 *   P<nums>                      -> Full pick+home sequence with pumps
 *                                   Examples: P1, P23, P1234, P14
 *   THERMO ON                    -> Start thermo pick sequence (pumps hold until THERMO OFF)
 *   THERMO OFF                   -> Release pumps and start thermo home sequence
 *   X                            -> Emergency stop, all motors off, pumps off
 *   E                            -> Print both encoder counts
 *
 * Servo Control:
 *   S 0                          -> Close stopper
 *   S 1                          -> Open stopper
 *   F 0                          -> Flipper stop
 *   F <colors>                   -> Flipper sequence, e.g. F B, F Y, F BBY, F BYBY (max 4)
 */

#include <Arduino.h>
#include <stdint.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include "ArmMotor.h"
#include "TurnMotor.h"

// ============================================================
// SECTION: Hardware pins
// ============================================================

constexpr int Pump1 = 25;
constexpr int Pump2 = 26;
constexpr int Pump3 = 27;
constexpr int Pump4 = 14;
constexpr int STOP  = 13;
constexpr int FLIP  = 12;

constexpr int MOTOR_TURN1 = 2;
constexpr int MOTOR_TURN2 = 15;
constexpr int MOTOR_LIFT1 = 5;
constexpr int MOTOR_LIFT2 = 4;

constexpr int ENCODER_LIFT_A = 19;
constexpr int ENCODER_LIFT_B = 21;
constexpr int ENCODER_TURN_A = 23;
constexpr int ENCODER_TURN_B = 22;

constexpr int PWM_CHANNEL_LIFT1 = 4;
constexpr int PWM_CHANNEL_LIFT2 = 5;
constexpr int PWM_CHANNEL_TURN1 = 6;
constexpr int PWM_CHANNEL_TURN2 = 7;

const int PUMP_PINS[4] = {Pump1, Pump2, Pump3, Pump4};

// ============================================================
// SECTION: Motor objects
// ============================================================

ArmMotor armMotor(
    /*kp*/ 1.0f, /*ki*/ 0.0f, /*kd*/ 0.0f,
    /*in1*/ MOTOR_LIFT1, /*in2*/ MOTOR_LIFT2,
    /*enc_a*/ ENCODER_LIFT_A, /*enc_b*/ ENCODER_LIFT_B,
    /*pwm_ch_fwd*/ PWM_CHANNEL_LIFT1, /*pwm_ch_rev*/ PWM_CHANNEL_LIFT2);

TurnMotor turnMotor(
    /*kp*/ 0.8f, /*ki*/ 0.05f, /*kd*/ 0.01f,
    /*in1*/ MOTOR_TURN1, /*in2*/ MOTOR_TURN2,
    /*enc_a*/ ENCODER_TURN_A, /*enc_b*/ ENCODER_TURN_B,
    /*pwm_ch_fwd*/ PWM_CHANNEL_TURN1, /*pwm_ch_rev*/ PWM_CHANNEL_TURN2);

// ============================================================
// SECTION: Pump state
// ============================================================

bool pump_states[4]    = {false, false, false, false};
bool requested_pumps[4] = {false, false, false, false};

void setPumpState(uint8_t pump_number, bool on) {
    if (pump_number < 1 || pump_number > 4) return;
    pump_states[pump_number - 1] = on;
    digitalWrite(PUMP_PINS[pump_number - 1], on ? HIGH : LOW);
}

void allPumpsOff() {
    for (int i = 0; i < 4; i++) {
        setPumpState(i + 1, false);
        requested_pumps[i] = false;
    }
}

// ============================================================
// SECTION: Servo tuning values
// ============================================================

// Stopper
constexpr int STOP_CLOSED_ANGLE = 180;
constexpr int STOP_OPEN_ANGLE   = 120;

// Flipper
constexpr int      FLIP_STOP         = 60;
constexpr int      FLIP_COLOR_START  = 50;
constexpr int      FLIP_B_END_COLOR  = 140;
constexpr int      FLIP_Y_END_COLOR  = 40;
constexpr uint32_t FLIPPER_SEQ_DELAY = 500;

// ============================================================
// SECTION: Servo objects and state
// ============================================================

Servo stopper_servo;
Servo flipper_servo;
bool  stopper_open = false;

void setStopperPosition(bool open) {
    stopper_open = open;
    stopper_servo.write(open ? STOP_OPEN_ANGLE : STOP_CLOSED_ANGLE);
    Serial.println(open ? "Stopper OPEN" : "Stopper CLOSED");
}

void setFlipperStop() {
    flipper_servo.write(FLIP_STOP);
    Serial.println("Flipper STOP");
}

// ============================================================
// SECTION: Flipper queue and state machine
// ============================================================

constexpr int FLIPPER_QUEUE_MAX = 4;

enum FlipperColor { FLIP_NONE, FLIP_BLUE, FLIP_YELLOW };

FlipperColor flipper_queue[FLIPPER_QUEUE_MAX];
int          flipper_queue_len = 0;
int          flipper_queue_idx = 0;

enum FlipperState {
    FLIPPER_IDLE,
    FLIPPER_STOPPER_OPEN,
    FLIPPER_STOPPER_OPEN_WAIT,
    FLIPPER_STOPPER_CLOSE,
    FLIPPER_STOPPER_CLOSE_WAIT,
    FLIPPER_MOVE_TO_START,
    FLIPPER_MOVE_TO_START_WAIT,
    FLIPPER_MOVE_TO_END_WAIT,
    FLIPPER_RETURN,
    FLIPPER_RETURN_WAIT,
    FLIPPER_NEXT,
};

FlipperState flipper_state       = FLIPPER_IDLE;
uint32_t     flipper_pause_timer = 0;

void flipperQueueClear() {
    flipper_queue_len = 0;
    flipper_queue_idx = 0;
    for (int i = 0; i < FLIPPER_QUEUE_MAX; i++) flipper_queue[i] = FLIP_NONE;
}

void startFlipperQueue(FlipperColor* colors, int count) {
    if (flipper_state != FLIPPER_IDLE) {
        Serial.println("Flipper sequence already running");
        return;
    }
    if (count <= 0 || count > FLIPPER_QUEUE_MAX) {
        Serial.println("Error: queue must be 1-4 commands");
        return;
    }
    flipperQueueClear();
    flipper_queue_len = count;
    for (int i = 0; i < count; i++) flipper_queue[i] = colors[i];
    flipper_queue_idx = 0;
    flipper_state = FLIPPER_STOPPER_OPEN;
    Serial.println("Flipper queue started");
}

void updateFlipperSequence() {
    auto currentEndAngle = [&]() -> int {
        if (flipper_queue_idx < flipper_queue_len) {
            return (flipper_queue[flipper_queue_idx] == FLIP_BLUE)
                ? FLIP_B_END_COLOR
                : FLIP_Y_END_COLOR;
        }
        return FLIP_STOP;
    };

    switch (flipper_state) {
        case FLIPPER_IDLE:
            break;

        case FLIPPER_STOPPER_OPEN:
            setStopperPosition(true);
            flipper_pause_timer = millis();
            flipper_state = FLIPPER_STOPPER_OPEN_WAIT;
            break;

        case FLIPPER_STOPPER_OPEN_WAIT:
            if (millis() - flipper_pause_timer >= FLIPPER_SEQ_DELAY) {
                flipper_state = FLIPPER_STOPPER_CLOSE;
            }
            break;

        case FLIPPER_STOPPER_CLOSE:
            setStopperPosition(false);
            flipper_pause_timer = millis();
            flipper_state = FLIPPER_STOPPER_CLOSE_WAIT;
            break;

        case FLIPPER_STOPPER_CLOSE_WAIT:
            if (millis() - flipper_pause_timer >= FLIPPER_SEQ_DELAY) {
                flipper_state = FLIPPER_MOVE_TO_START;
            }
            break;

        case FLIPPER_MOVE_TO_START:
            flipper_servo.write(FLIP_COLOR_START);
            flipper_pause_timer = millis();
            flipper_state = FLIPPER_MOVE_TO_START_WAIT;
            Serial.print("Flipper: move to start, color=");
            Serial.println(flipper_queue[flipper_queue_idx] == FLIP_BLUE ? "B" : "Y");
            break;

        case FLIPPER_MOVE_TO_START_WAIT:
            if (millis() - flipper_pause_timer >= FLIPPER_SEQ_DELAY) {
                flipper_servo.write(currentEndAngle());
                flipper_pause_timer = millis();
                flipper_state = FLIPPER_MOVE_TO_END_WAIT;
                Serial.println("Flipper: move to end");
            }
            break;

        case FLIPPER_MOVE_TO_END_WAIT:
            if (millis() - flipper_pause_timer >= FLIPPER_SEQ_DELAY) {
                flipper_state = FLIPPER_RETURN;
            }
            break;

        case FLIPPER_RETURN:
            setFlipperStop();
            flipper_pause_timer = millis();
            flipper_state = FLIPPER_RETURN_WAIT;
            break;

        case FLIPPER_RETURN_WAIT:
            if (millis() - flipper_pause_timer >= FLIPPER_SEQ_DELAY) {
                flipper_state = FLIPPER_NEXT;
            }
            break;

        case FLIPPER_NEXT:
            flipper_queue_idx++;
            if (flipper_queue_idx < flipper_queue_len) {
                Serial.println("Flipper: next in queue");
                flipper_state = FLIPPER_STOPPER_OPEN;
            } else {
                Serial.println("Flipper queue complete");
                flipperQueueClear();
                flipper_state = FLIPPER_IDLE;
            }
            break;
    }
}

// ============================================================
// SECTION: Pick/Home tuning values
// ============================================================

constexpr uint32_t LIFT_SEQ_DELAY       = 500;
constexpr uint32_t LIFT_PICK_HOLD_MS    = 1000;

constexpr int PICK_POS_LIFT_CLEAR  = 500;
constexpr int PICK_POS_TURN_MID    = 450;
constexpr int PICK_POS_LIFT_MID    = 1100;
constexpr int PICK_POS_TURN_PICK   = 1050;
constexpr int PICK_POS_LIFT_PICK   = 1350;
constexpr int PICK_POS_LIFT_WIGGLE = 1450;

constexpr int HOME_POS_LIFT_CLEAR   = 1000;
constexpr int HOME_POS_TURN_MID     = 500;
constexpr int HOME_POS_LIFT_MID     = 500;
constexpr int HOME_POS_TURN_HOME    = 0;
constexpr int HOME_POS_LIFT_DEPOSIT = 200;

// ============================================================
// SECTION: Thermo tuning values
// ============================================================

constexpr int THERMO_POS_LIFT_CLEAR   = 500;
constexpr int THERMO_POS_TURN_MID     = 450;
constexpr int THERMO_POS_LIFT_MID     = 1100;
constexpr int THERMO_POS_TURN_PICK    = 1050;
constexpr int THERMO_POS_LIFT_PICK    = 1350;
constexpr int THERMO_POS_LIFT_WIGGLE  = 1450;

constexpr int THERMO_HOME_POS_LIFT_CLEAR   = 1000;
constexpr int THERMO_HOME_POS_TURN_MID     = 500;
constexpr int THERMO_HOME_POS_LIFT_MID     = 500;
constexpr int THERMO_HOME_POS_TURN_HOME    = 0;
constexpr int THERMO_HOME_POS_LIFT_DEPOSIT = 200;

// ============================================================
// SECTION: Lift/Thermo state machine
// ============================================================

enum LiftState {
    LIFT_IDLE,

    // Pick sequence
    PICK_LIFT_TO_CLEAR,
    PICK_WAIT_LIFT_TO_CLEAR,
    PICK_PAUSE_AFTER_LIFT_CLEAR,
    PICK_TURN_TO_MID,
    PICK_WAIT_TURN_TO_MID,
    PICK_PAUSE_AFTER_TURN_MID,
    PICK_LIFT_TO_MID,
    PICK_WAIT_LIFT_TO_MID,
    PICK_PAUSE_AFTER_LIFT_MID,
    PICK_TURN_TO_PICK,
    PICK_WAIT_TURN_TO_PICK,
    PICK_PAUSE_AFTER_TURN_PICK,
    PICK_LIFT_TO_PICK,
    PICK_WAIT_LIFT_TO_PICK,
    PICK_PAUSE_AFTER_LIFT_PICK,
    PICK_PUMPS_ON,
    PICK_LIFT_WIGGLE_UP,
    PICK_WAIT_LIFT_WIGGLE_UP,
    PICK_HOLD_AT_PICK,
    PICK_DONE,

    // Home sequence
    HOME_LIFT_TO_CLEAR,
    HOME_WAIT_LIFT_TO_CLEAR,
    HOME_PAUSE_AFTER_LIFT_CLEAR,
    HOME_TURN_TO_MID,
    HOME_WAIT_TURN_TO_MID,
    HOME_PAUSE_AFTER_TURN_MID,
    HOME_LIFT_TO_MID,
    HOME_WAIT_LIFT_TO_MID,
    HOME_PAUSE_AFTER_LIFT_MID,
    HOME_TURN_TO_HOME,
    HOME_WAIT_TURN_TO_HOME,
    HOME_PAUSE_AFTER_TURN_HOME,
    HOME_LIFT_TO_DEPOSIT,
    HOME_WAIT_LIFT_TO_DEPOSIT,
    HOME_PUMPS_OFF,
    HOME_PAUSE_AFTER_PUMPS_OFF,
    HOME_LIFT_TO_REST,
    HOME_WAIT_LIFT_TO_REST,
    HOME_DONE,

    // Thermo pick sequence
    THERMO_LIFT_TO_CLEAR,
    THERMO_WAIT_LIFT_TO_CLEAR,
    THERMO_PAUSE_AFTER_LIFT_CLEAR,
    THERMO_TURN_TO_MID,
    THERMO_WAIT_TURN_TO_MID,
    THERMO_PAUSE_AFTER_TURN_MID,
    THERMO_LIFT_TO_MID,
    THERMO_WAIT_LIFT_TO_MID,
    THERMO_PAUSE_AFTER_LIFT_MID,
    THERMO_TURN_TO_PICK,
    THERMO_WAIT_TURN_TO_PICK,
    THERMO_PAUSE_AFTER_TURN_PICK,
    THERMO_LIFT_TO_PICK,
    THERMO_WAIT_LIFT_TO_PICK,
    THERMO_PAUSE_AFTER_LIFT_PICK,
    THERMO_PUMPS_ON,
    THERMO_LIFT_WIGGLE_UP,
    THERMO_WAIT_LIFT_WIGGLE_UP,
    THERMO_HOLDING,

    // Thermo home sequence
    THERMO_HOME_LIFT_TO_CLEAR,
    THERMO_HOME_WAIT_LIFT_TO_CLEAR,
    THERMO_HOME_PAUSE_AFTER_LIFT_CLEAR,
    THERMO_HOME_TURN_TO_MID,
    THERMO_HOME_WAIT_TURN_TO_MID,
    THERMO_HOME_PAUSE_AFTER_TURN_MID,
    THERMO_HOME_LIFT_TO_MID,
    THERMO_HOME_WAIT_LIFT_TO_MID,
    THERMO_HOME_PAUSE_AFTER_LIFT_MID,
    THERMO_HOME_TURN_TO_HOME,
    THERMO_HOME_WAIT_TURN_TO_HOME,
    THERMO_HOME_PAUSE_AFTER_TURN_HOME,
    THERMO_HOME_LIFT_TO_DEPOSIT,
    THERMO_HOME_WAIT_LIFT_TO_DEPOSIT,
    THERMO_HOME_PUMPS_OFF,
    THERMO_HOME_PAUSE_AFTER_PUMPS_OFF,
    THERMO_HOME_LIFT_TO_REST,
    THERMO_HOME_WAIT_LIFT_TO_REST,
    THERMO_HOME_DONE,
};

LiftState lift_state       = LIFT_IDLE;
uint32_t  lift_pause_timer = 0;

void updateLiftSequence() {
    switch (lift_state) {
    case LIFT_IDLE:
        break;

    // ----------------------------------------
    // Pick sequence
    // ----------------------------------------

    case PICK_LIFT_TO_CLEAR:
        armMotor.resetEncoder();
        turnMotor.resetEncoder();
        armMotor.enable();
        turnMotor.enable();
        armMotor.setTarget(PICK_POS_LIFT_CLEAR);
        Serial.println("Pick: lift -> clear");
        lift_state = PICK_WAIT_LIFT_TO_CLEAR;
        break;

    case PICK_WAIT_LIFT_TO_CLEAR:
        if (armMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = PICK_PAUSE_AFTER_LIFT_CLEAR;
        }
        break;

    case PICK_PAUSE_AFTER_LIFT_CLEAR:
        if (millis() - lift_pause_timer >= LIFT_SEQ_DELAY) {
            lift_state = PICK_TURN_TO_MID;
        }
        break;

    case PICK_TURN_TO_MID:
        turnMotor.setTarget(PICK_POS_TURN_MID);
        Serial.println("Pick: turn -> mid");
        lift_state = PICK_WAIT_TURN_TO_MID;
        break;

    case PICK_WAIT_TURN_TO_MID:
        if (turnMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = PICK_PAUSE_AFTER_TURN_MID;
        }
        break;

    case PICK_PAUSE_AFTER_TURN_MID:
        if (millis() - lift_pause_timer >= LIFT_SEQ_DELAY) {
            lift_state = PICK_LIFT_TO_MID;
        }
        break;

    case PICK_LIFT_TO_MID:
        armMotor.setTarget(PICK_POS_LIFT_MID);
        Serial.println("Pick: lift -> mid");
        lift_state = PICK_WAIT_LIFT_TO_MID;
        break;

    case PICK_WAIT_LIFT_TO_MID:
        if (armMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = PICK_PAUSE_AFTER_LIFT_MID;
        }
        break;

    case PICK_PAUSE_AFTER_LIFT_MID:
        if (millis() - lift_pause_timer >= LIFT_SEQ_DELAY) {
            lift_state = PICK_TURN_TO_PICK;
        }
        break;

    case PICK_TURN_TO_PICK:
        turnMotor.setTarget(PICK_POS_TURN_PICK);
        Serial.println("Pick: turn -> pick");
        lift_state = PICK_WAIT_TURN_TO_PICK;
        break;

    case PICK_WAIT_TURN_TO_PICK:
        if (turnMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = PICK_PAUSE_AFTER_TURN_PICK;
        }
        break;

    case PICK_PAUSE_AFTER_TURN_PICK:
        if (millis() - lift_pause_timer >= LIFT_SEQ_DELAY) {
            lift_state = PICK_LIFT_TO_PICK;
        }
        break;

    case PICK_LIFT_TO_PICK:
        armMotor.setTarget(PICK_POS_LIFT_PICK);
        Serial.println("Pick: lift -> pick");
        lift_state = PICK_WAIT_LIFT_TO_PICK;
        break;

    case PICK_WAIT_LIFT_TO_PICK:
        if (armMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = PICK_PAUSE_AFTER_LIFT_PICK;
        }
        break;

    case PICK_PAUSE_AFTER_LIFT_PICK:
        if (millis() - lift_pause_timer >= LIFT_SEQ_DELAY) {
            lift_state = PICK_PUMPS_ON;
        }
        break;

    case PICK_PUMPS_ON:
        for (int i = 0; i < 4; i++) {
            if (requested_pumps[i]) setPumpState(i + 1, true);
        }
        Serial.println("Pick: pumps ON");
        lift_state = PICK_LIFT_WIGGLE_UP;
        break;

    case PICK_LIFT_WIGGLE_UP:
        armMotor.setTarget(PICK_POS_LIFT_WIGGLE);
        Serial.println("Pick: lift -> wiggle");
        lift_state = PICK_WAIT_LIFT_WIGGLE_UP;
        break;

    case PICK_WAIT_LIFT_WIGGLE_UP:
        if (armMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = PICK_HOLD_AT_PICK;
        }
        break;

    case PICK_HOLD_AT_PICK:
        if (millis() - lift_pause_timer >= LIFT_PICK_HOLD_MS) {
            lift_state = PICK_DONE;
        }
        break;

    case PICK_DONE:
        Serial.println("Pick done, starting home sequence");
        lift_state = HOME_LIFT_TO_CLEAR;
        break;

    // ----------------------------------------
    // Home sequence
    // ----------------------------------------

    case HOME_LIFT_TO_CLEAR:
        armMotor.setTarget(HOME_POS_LIFT_CLEAR);
        Serial.println("Home: lift -> clear");
        lift_state = HOME_WAIT_LIFT_TO_CLEAR;
        break;

    case HOME_WAIT_LIFT_TO_CLEAR:
        if (armMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = HOME_PAUSE_AFTER_LIFT_CLEAR;
        }
        break;

    case HOME_PAUSE_AFTER_LIFT_CLEAR:
        if (millis() - lift_pause_timer >= LIFT_SEQ_DELAY) {
            lift_state = HOME_TURN_TO_MID;
        }
        break;

    case HOME_TURN_TO_MID:
        turnMotor.setTarget(HOME_POS_TURN_MID);
        Serial.println("Home: turn -> mid");
        lift_state = HOME_WAIT_TURN_TO_MID;
        break;

    case HOME_WAIT_TURN_TO_MID:
        if (turnMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = HOME_PAUSE_AFTER_TURN_MID;
        }
        break;

    case HOME_PAUSE_AFTER_TURN_MID:
        if (millis() - lift_pause_timer >= LIFT_SEQ_DELAY) {
            lift_state = HOME_LIFT_TO_MID;
        }
        break;

    case HOME_LIFT_TO_MID:
        armMotor.setTarget(HOME_POS_LIFT_MID);
        Serial.println("Home: lift -> mid");
        lift_state = HOME_WAIT_LIFT_TO_MID;
        break;

    case HOME_WAIT_LIFT_TO_MID:
        if (armMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = HOME_PAUSE_AFTER_LIFT_MID;
        }
        break;

    case HOME_PAUSE_AFTER_LIFT_MID:
        if (millis() - lift_pause_timer >= LIFT_SEQ_DELAY) {
            lift_state = HOME_TURN_TO_HOME;
        }
        break;

    case HOME_TURN_TO_HOME:
        turnMotor.setTarget(HOME_POS_TURN_HOME);
        Serial.println("Home: turn -> home");
        lift_state = HOME_WAIT_TURN_TO_HOME;
        break;

    case HOME_WAIT_TURN_TO_HOME:
        if (turnMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = HOME_PAUSE_AFTER_TURN_HOME;
        }
        break;

    case HOME_PAUSE_AFTER_TURN_HOME:
        if (millis() - lift_pause_timer >= LIFT_SEQ_DELAY) {
            lift_state = HOME_LIFT_TO_DEPOSIT;
        }
        break;

    case HOME_LIFT_TO_DEPOSIT:
        armMotor.setTarget(HOME_POS_LIFT_DEPOSIT);
        Serial.println("Home: lift -> deposit");
        lift_state = HOME_WAIT_LIFT_TO_DEPOSIT;
        break;

    case HOME_WAIT_LIFT_TO_DEPOSIT:
        if (armMotor.isAtTarget()) {
            lift_state = HOME_PUMPS_OFF;
        }
        break;

    case HOME_PUMPS_OFF:
        allPumpsOff();
        Serial.println("Home: pumps OFF");
        lift_pause_timer = millis();
        lift_state = HOME_PAUSE_AFTER_PUMPS_OFF;
        break;

    case HOME_PAUSE_AFTER_PUMPS_OFF:
        if (millis() - lift_pause_timer >= LIFT_SEQ_DELAY) {
            lift_state = HOME_LIFT_TO_REST;
        }
        break;

    case HOME_LIFT_TO_REST:
        armMotor.setTarget(0);
        Serial.println("Home: lift -> rest");
        lift_state = HOME_WAIT_LIFT_TO_REST;
        break;

    case HOME_WAIT_LIFT_TO_REST:
        if (armMotor.isAtTarget()) {
            lift_state = HOME_DONE;
        }
        break;

    case HOME_DONE:
        Serial.println("Home sequence complete");
        armMotor.disable();
        turnMotor.disable();
        lift_state = LIFT_IDLE;
        break;

    // ----------------------------------------
    // Thermo pick sequence
    // ----------------------------------------

    case THERMO_LIFT_TO_CLEAR:
        armMotor.resetEncoder();
        turnMotor.resetEncoder();
        armMotor.enable();
        turnMotor.enable();
        armMotor.setTarget(THERMO_POS_LIFT_CLEAR);
        Serial.println("Thermo: lift -> clear");
        lift_state = THERMO_WAIT_LIFT_TO_CLEAR;
        break;

    case THERMO_WAIT_LIFT_TO_CLEAR:
        if (armMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = THERMO_PAUSE_AFTER_LIFT_CLEAR;
        }
        break;

    case THERMO_PAUSE_AFTER_LIFT_CLEAR:
        if (millis() - lift_pause_timer >= LIFT_SEQ_DELAY) {
            lift_state = THERMO_TURN_TO_MID;
        }
        break;

    case THERMO_TURN_TO_MID:
        turnMotor.setTarget(THERMO_POS_TURN_MID);
        Serial.println("Thermo: turn -> mid");
        lift_state = THERMO_WAIT_TURN_TO_MID;
        break;

    case THERMO_WAIT_TURN_TO_MID:
        if (turnMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = THERMO_PAUSE_AFTER_TURN_MID;
        }
        break;

    case THERMO_PAUSE_AFTER_TURN_MID:
        if (millis() - lift_pause_timer >= LIFT_SEQ_DELAY) {
            lift_state = THERMO_LIFT_TO_MID;
        }
        break;

    case THERMO_LIFT_TO_MID:
        armMotor.setTarget(THERMO_POS_LIFT_MID);
        Serial.println("Thermo: lift -> mid");
        lift_state = THERMO_WAIT_LIFT_TO_MID;
        break;

    case THERMO_WAIT_LIFT_TO_MID:
        if (armMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = THERMO_PAUSE_AFTER_LIFT_MID;
        }
        break;

    case THERMO_PAUSE_AFTER_LIFT_MID:
        if (millis() - lift_pause_timer >= LIFT_SEQ_DELAY) {
            lift_state = THERMO_TURN_TO_PICK;
        }
        break;

    case THERMO_TURN_TO_PICK:
        turnMotor.setTarget(THERMO_POS_TURN_PICK);
        Serial.println("Thermo: turn -> pick");
        lift_state = THERMO_WAIT_TURN_TO_PICK;
        break;

    case THERMO_WAIT_TURN_TO_PICK:
        if (turnMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = THERMO_PAUSE_AFTER_TURN_PICK;
        }
        break;

    case THERMO_PAUSE_AFTER_TURN_PICK:
        if (millis() - lift_pause_timer >= LIFT_SEQ_DELAY) {
            lift_state = THERMO_LIFT_TO_PICK;
        }
        break;

    case THERMO_LIFT_TO_PICK:
        armMotor.setTarget(THERMO_POS_LIFT_PICK);
        Serial.println("Thermo: lift -> pick");
        lift_state = THERMO_WAIT_LIFT_TO_PICK;
        break;

    case THERMO_WAIT_LIFT_TO_PICK:
        if (armMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = THERMO_PAUSE_AFTER_LIFT_PICK;
        }
        break;

    case THERMO_PAUSE_AFTER_LIFT_PICK:
        if (millis() - lift_pause_timer >= LIFT_SEQ_DELAY) {
            lift_state = THERMO_PUMPS_ON;
        }
        break;

    case THERMO_PUMPS_ON:
        for (int i = 0; i < 4; i++) setPumpState(i + 1, true);
        Serial.println("Thermo: all pumps ON");
        lift_state = THERMO_LIFT_WIGGLE_UP;
        break;

    case THERMO_LIFT_WIGGLE_UP:
        armMotor.setTarget(THERMO_POS_LIFT_WIGGLE);
        Serial.println("Thermo: lift -> wiggle");
        lift_state = THERMO_WAIT_LIFT_WIGGLE_UP;
        break;

    case THERMO_WAIT_LIFT_WIGGLE_UP:
        if (armMotor.isAtTarget()) {
            lift_state = THERMO_HOLDING;
            Serial.println("Thermo: holding — send 'THERMO OFF' to release");
        }
        break;

    case THERMO_HOLDING:
        break;

    // ----------------------------------------
    // Thermo home sequence
    // ----------------------------------------

    case THERMO_HOME_LIFT_TO_CLEAR:
        armMotor.setTarget(THERMO_HOME_POS_LIFT_CLEAR);
        Serial.println("Thermo home: lift -> clear");
        lift_state = THERMO_HOME_WAIT_LIFT_TO_CLEAR;
        break;

    case THERMO_HOME_WAIT_LIFT_TO_CLEAR:
        if (armMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = THERMO_HOME_PAUSE_AFTER_LIFT_CLEAR;
        }
        break;

    case THERMO_HOME_PAUSE_AFTER_LIFT_CLEAR:
        if (millis() - lift_pause_timer >= LIFT_SEQ_DELAY) {
            lift_state = THERMO_HOME_TURN_TO_MID;
        }
        break;

    case THERMO_HOME_TURN_TO_MID:
        turnMotor.setTarget(THERMO_HOME_POS_TURN_MID);
        Serial.println("Thermo home: turn -> mid");
        lift_state = THERMO_HOME_WAIT_TURN_TO_MID;
        break;

    case THERMO_HOME_WAIT_TURN_TO_MID:
        if (turnMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = THERMO_HOME_PAUSE_AFTER_TURN_MID;
        }
        break;

    case THERMO_HOME_PAUSE_AFTER_TURN_MID:
        if (millis() - lift_pause_timer >= LIFT_SEQ_DELAY) {
            lift_state = THERMO_HOME_LIFT_TO_MID;
        }
        break;

    case THERMO_HOME_LIFT_TO_MID:
        armMotor.setTarget(THERMO_HOME_POS_LIFT_MID);
        Serial.println("Thermo home: lift -> mid");
        lift_state = THERMO_HOME_WAIT_LIFT_TO_MID;
        break;

    case THERMO_HOME_WAIT_LIFT_TO_MID:
        if (armMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = THERMO_HOME_PAUSE_AFTER_LIFT_MID;
        }
        break;

    case THERMO_HOME_PAUSE_AFTER_LIFT_MID:
        if (millis() - lift_pause_timer >= LIFT_SEQ_DELAY) {
            lift_state = THERMO_HOME_TURN_TO_HOME;
        }
        break;

    case THERMO_HOME_TURN_TO_HOME:
        turnMotor.setTarget(THERMO_HOME_POS_TURN_HOME);
        Serial.println("Thermo home: turn -> home");
        lift_state = THERMO_HOME_WAIT_TURN_TO_HOME;
        break;

    case THERMO_HOME_WAIT_TURN_TO_HOME:
        if (turnMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = THERMO_HOME_PAUSE_AFTER_TURN_HOME;
        }
        break;

    case THERMO_HOME_PAUSE_AFTER_TURN_HOME:
        if (millis() - lift_pause_timer >= LIFT_SEQ_DELAY) {
            lift_state = THERMO_HOME_LIFT_TO_DEPOSIT;
        }
        break;

    case THERMO_HOME_LIFT_TO_DEPOSIT:
        armMotor.setTarget(THERMO_HOME_POS_LIFT_DEPOSIT);
        Serial.println("Thermo home: lift -> deposit");
        lift_state = THERMO_HOME_WAIT_LIFT_TO_DEPOSIT;
        break;

    case THERMO_HOME_WAIT_LIFT_TO_DEPOSIT:
        if (armMotor.isAtTarget()) {
            lift_state = THERMO_HOME_PUMPS_OFF;
        }
        break;

    case THERMO_HOME_PUMPS_OFF:
        allPumpsOff();
        Serial.println("Thermo home: pumps OFF");
        lift_pause_timer = millis();
        lift_state = THERMO_HOME_PAUSE_AFTER_PUMPS_OFF;
        break;

    case THERMO_HOME_PAUSE_AFTER_PUMPS_OFF:
        if (millis() - lift_pause_timer >= LIFT_SEQ_DELAY) {
            lift_state = THERMO_HOME_LIFT_TO_REST;
        }
        break;

    case THERMO_HOME_LIFT_TO_REST:
        armMotor.setTarget(0);
        Serial.println("Thermo home: lift -> rest");
        lift_state = THERMO_HOME_WAIT_LIFT_TO_REST;
        break;

    case THERMO_HOME_WAIT_LIFT_TO_REST:
        if (armMotor.isAtTarget()) {
            lift_state = THERMO_HOME_DONE;
        }
        break;

    case THERMO_HOME_DONE:
        Serial.println("Thermo sequence complete");
        armMotor.disable();
        turnMotor.disable();
        lift_state = LIFT_IDLE;
        break;
    }
}

// ============================================================
// SECTION: Setup
// ============================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\nStarting Manipulation Test...");

    for (int i = 0; i < 4; i++) {
        pinMode(PUMP_PINS[i], OUTPUT);
        digitalWrite(PUMP_PINS[i], LOW);
    }

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    stopper_servo.setPeriodHertz(50);
    flipper_servo.setPeriodHertz(50);
    stopper_servo.attach(STOP, 500, 2400);
    flipper_servo.attach(FLIP, 500, 2400);
    setStopperPosition(false);
    setFlipperStop();

    armMotor.setup();
    turnMotor.setup();
    armMotor.enable();
    turnMotor.enable();
}

// ============================================================
// SECTION: Main loop
// ============================================================

void loop() {
    static uint32_t last = micros();
    uint32_t now = micros();
    const uint32_t interval = 2000;

    if (now - last >= interval) {
        last += interval;
        armMotor.update();
        turnMotor.update();
    }

    updateFlipperSequence();
    updateLiftSequence();

    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command.length() == 0) return;

        char operation = tolower(command.charAt(0));

        bool is_compact = (command.length() == 1) &&
                          (operation == 'x' || operation == 'e');

        if (is_compact) {
            switch (operation) {
                case 'x':
                    lift_state = LIFT_IDLE;
                    allPumpsOff();
                    setFlipperStop();
                    setStopperPosition(false);
                    ledcWrite(PWM_CHANNEL_LIFT1, 0);
                    ledcWrite(PWM_CHANNEL_LIFT2, 0);
                    ledcWrite(PWM_CHANNEL_TURN1, 0);
                    ledcWrite(PWM_CHANNEL_TURN2, 0);
                    Serial.println("STOP: sequence aborted, motors off, pumps off");
                    break;

                case 'e':
                    Serial.print("Arm encoder:  ");
                    Serial.println(armMotor.getEncoderCount());
                    Serial.print("Turn encoder: ");
                    Serial.println(turnMotor.getEncoderCount());
                    break;
            }
            return;
        }

        int space_idx = command.indexOf(' ');
        String cmd_type  = (space_idx > 0) ? command.substring(0, space_idx) : command;
        String cmd_value = (space_idx > 0) ? command.substring(space_idx + 1) : "";
        String cmd_type_upper  = cmd_type;
        String cmd_value_upper = cmd_value;
        cmd_type_upper.toUpperCase();
        cmd_value_upper.toUpperCase();

        if (cmd_type_upper.length() > 1 && cmd_type_upper.charAt(0) == 'P' && isDigit(cmd_type_upper.charAt(1))) {
            if (lift_state != LIFT_IDLE) {
                Serial.println("Sequence already running");
                return;
            }
            String pump_nums = cmd_type_upper.substring(1);
            bool valid = true;
            memset(requested_pumps, false, sizeof(requested_pumps));
            for (int i = 0; i < pump_nums.length(); i++) {
                char digit = pump_nums[i];
                if (digit >= '1' && digit <= '4') {
                    requested_pumps[digit - '1'] = true;
                } else {
                    valid = false;
                    break;
                }
            }
            if (!valid) {
                Serial.println("Error: Use P1, P23, P1234, etc.");
                return;
            }
            Serial.print("Starting pick+home sequence with pumps: ");
            Serial.println(pump_nums);
            lift_state = PICK_LIFT_TO_CLEAR;
        }
        else if (cmd_type_upper == "PUMP") {
            if (cmd_value_upper.length() == 1 && cmd_value_upper[0] >= '1' && cmd_value_upper[0] <= '4') {
                int pump = cmd_value_upper[0] - '0';
                bool new_state = !pump_states[pump - 1];
                setPumpState(pump, new_state);
                Serial.print("Pump ");
                Serial.print(pump);
                Serial.println(new_state ? " ON" : " OFF");
            } else {
                Serial.println("Error: Use PUMP 1, PUMP 2, PUMP 3, or PUMP 4");
            }
        }
        else if (cmd_type_upper == "THERMO") {
            if (cmd_value_upper == "ON") {
                if (lift_state != LIFT_IDLE) {
                    Serial.println("Sequence already running");
                    return;
                }
                Serial.println("Starting thermo pick sequence");
                lift_state = THERMO_LIFT_TO_CLEAR;
            }
            else if (cmd_value_upper == "OFF") {
                if (lift_state == THERMO_HOLDING) {
                    Serial.println("Thermo: releasing, starting home sequence");
                    lift_state = THERMO_HOME_LIFT_TO_CLEAR;
                } else {
                    Serial.println("Thermo not currently holding");
                }
            }
            else {
                Serial.println("Error: Use THERMO ON or THERMO OFF");
            }
        }
        else if (cmd_type_upper == "S") {
            if      (cmd_value_upper == "0") setStopperPosition(false);
            else if (cmd_value_upper == "1") setStopperPosition(true);
            else Serial.println("Error: Use S 0 or S 1");
        }
        else if (cmd_type_upper == "F") {
            if (cmd_value_upper == "0") {
                flipper_state = FLIPPER_IDLE;
                flipperQueueClear();
                setFlipperStop();
            }
            else {
                FlipperColor colors[FLIPPER_QUEUE_MAX];
                int count = 0;
                bool valid = true;
                if (cmd_value_upper.length() == 0 || cmd_value_upper.length() > FLIPPER_QUEUE_MAX) {
                    valid = false;
                } else {
                    for (int i = 0; i < (int)cmd_value_upper.length(); i++) {
                        if      (cmd_value_upper[i] == 'B') colors[count++] = FLIP_BLUE;
                        else if (cmd_value_upper[i] == 'Y') colors[count++] = FLIP_YELLOW;
                        else { valid = false; break; }
                    }
                }
                if (!valid || count == 0) {
                    Serial.println("Error: Use F 0, F B, F Y, F BBY, F BYBY, etc. (max 4)");
                } else {
                    startFlipperQueue(colors, count);
                }
            }
        }
        else {
            Serial.print("Unknown command: ");
            Serial.println(cmd_type);
        }
    }
}