/*
 * Manipulation Test Serial Commands
 * -----------------------------------------------
 * Commands:
 *   P<nums>                      -> Full pick+home sequence with pumps
 *                                   Examples: P1, P23, P1234, P14
 *   X                            -> Emergency stop, all motors off, pumps off
 *   E                            -> Print both encoder counts
 *
 * Servo Control:
 *   S 0                          -> Close stopper
 *   S 1                          -> Open stopper
 *   F 0                          -> Flipper stop (90)
 *   F B                          -> Flipper B sequence (65 then 140 after 1s)
 *   F Y                          -> Flipper Y position (110)
 */

#include <Arduino.h>
#include <stdint.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include "ArmMotor.h"
#include "TurnMotor.h"

// ----------------------------------------
// SECTION: Hardware pins
// ----------------------------------------

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
constexpr int ENCODER_LIFT_X = 18;
constexpr int ENCODER_TURN_A = 23;
constexpr int ENCODER_TURN_B = 22;
constexpr int ENCODER_TURN_X = 32;

constexpr int PWM_CHANNEL_LIFT1 = 4;
constexpr int PWM_CHANNEL_LIFT2 = 5;
constexpr int PWM_CHANNEL_TURN1 = 6;
constexpr int PWM_CHANNEL_TURN2 = 7;

const int PUMP_PINS[4] = {Pump1, Pump2, Pump3, Pump4};

// ----------------------------------------
// SECTION: Motor objects
// ----------------------------------------

ArmMotor armMotor(
    /*kp*/ 1.0f, /*ki*/ 0.0f, /*kd*/ 0.0f,
    /*in1*/ MOTOR_LIFT1, /*in2*/ MOTOR_LIFT2,
    /*enc_a*/ ENCODER_LIFT_A, /*enc_b*/ ENCODER_LIFT_B, /*enc_x*/ ENCODER_LIFT_X,
    /*pwm_ch_fwd*/ PWM_CHANNEL_LIFT1, /*pwm_ch_rev*/ PWM_CHANNEL_LIFT2);

TurnMotor turnMotor(
    /*kp*/ 2.0f, /*ki*/ 0.0f, /*kd*/ 0.0f,
    /*in1*/ MOTOR_TURN1, /*in2*/ MOTOR_TURN2,
    /*enc_a*/ ENCODER_TURN_A, /*enc_b*/ ENCODER_TURN_B, /*enc_x*/ ENCODER_TURN_X,
    /*pwm_ch_fwd*/ PWM_CHANNEL_TURN1, /*pwm_ch_rev*/ PWM_CHANNEL_TURN2);

// ----------------------------------------
// SECTION: Sequence state machine
// ----------------------------------------

enum LiftState {
    LIFT_IDLE,

    // Pick sequence
    PICK_ARM_TO_300,
    PICK_WAIT_ARM_300,
    PICK_PAUSE_1,
    PICK_SIMULTANEOUS,          // turn -> 1000, arm -> 950
    PICK_WAIT_SIMULTANEOUS,
    PICK_PAUSE_2,
    PICK_ARM_TO_1480,
    PICK_WAIT_ARM_1480,
    PICK_PUMPS_ON,
    PICK_PAUSE_HOLD,            // 2s hold after pumps on
    PICK_DONE,

    // Home sequence
    HOME_ARM_TO_800,
    HOME_WAIT_ARM_800,
    HOME_PAUSE_1,
    HOME_SIMULTANEOUS,          // turn -> 500, arm -> 325
    HOME_WAIT_SIMULTANEOUS,
    HOME_PAUSE_2,
    HOME_TURN_TO_0,
    HOME_WAIT_TURN_0,
    HOME_PAUSE_3,
    HOME_ARM_TO_0,
    HOME_WAIT_ARM_0,
    HOME_PUMPS_OFF,
    HOME_PUMPS_WAIT,
    HOME_FINAL_RETURN,
    HOME_DONE
};

LiftState lift_state         = LIFT_IDLE;
uint32_t  lift_pause_timer   = 0;
bool      requested_pumps[4] = {false, false, false, false};

// ----------------------------------------
// SECTION: Servo tuning values
// ----------------------------------------

constexpr int STOP_CLOSED_ANGLE  = 180;
constexpr int STOP_OPEN_ANGLE    = 130;

constexpr int      FLIP_STOP        = 90;
constexpr int      FLIP_B_START     = 65;
constexpr int      FLIP_B_END       = 140;
constexpr int      FLIP_Y           = 110;
constexpr uint32_t FLIP_B_DELAY_MS  = 1000;

// Pump shutoff delay between each pump (ms)
constexpr uint32_t PUMP_SHUTOFF_DELAY = 500;

// ----------------------------------------
// SECTION: Servo objects and state
// ----------------------------------------

Servo stopper_servo;
Servo flipper_servo;

bool     flipper_b_pending = false;
uint32_t flipper_b_timer   = 0;
bool     pump_states[4]    = {false, false, false, false};
bool     stopper_open      = false;

// ----------------------------------------
// SECTION: Helper functions
// ----------------------------------------

bool isNumeric(const String& value) {
    if (value.length() == 0) return false;
    for (size_t i = 0; i < value.length(); i++) {
        if (!isDigit(value[i])) return false;
    }
    return true;
}

// ----------------------------------------
// SECTION: Pump functions
// ----------------------------------------

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

// ----------------------------------------
// SECTION: Servo functions
// ----------------------------------------

void setStopperPosition(bool open) {
    stopper_open = open;
    stopper_servo.write(open ? STOP_OPEN_ANGLE : STOP_CLOSED_ANGLE);
    Serial.println(open ? "Stopper OPEN" : "Stopper CLOSED");
}

void setFlipperStop() { flipper_servo.write(FLIP_STOP); Serial.println("Flipper STOP"); }
void setFlipperY()    { flipper_servo.write(FLIP_Y);    Serial.println("Flipper Y"); }

void setFlipperB() {
    flipper_servo.write(FLIP_B_START);
    flipper_b_pending = true;
    flipper_b_timer   = millis();
    Serial.println("Flipper B: going to 65, then 140 in 1s");
}

// ----------------------------------------
// SECTION: Sequence
// ----------------------------------------

void updateLiftSequence() {
    switch (lift_state) {
    case LIFT_IDLE:
        break;

    // ----------------------------------------
    // Pick sequence
    // ----------------------------------------

    case PICK_ARM_TO_300:
        armMotor.setTarget(300);
        Serial.println("Pick seq: arm -> 300");
        lift_state = PICK_WAIT_ARM_300;
        break;

    case PICK_WAIT_ARM_300:
        if (armMotor.isAtTarget()) {
            Serial.println("Pick seq: arm at 300, waiting 0.5s");
            lift_pause_timer = millis();
            lift_state = PICK_PAUSE_1;
        }
        break;

    case PICK_PAUSE_1:
        if (millis() - lift_pause_timer >= 500) {
            lift_state = PICK_SIMULTANEOUS;
        }
        break;

    case PICK_SIMULTANEOUS:
        turnMotor.setTarget(1025);
        armMotor.setTarget(950);
        Serial.println("Pick seq: turn -> 1025, arm -> 950 simultaneously");
        lift_state = PICK_WAIT_SIMULTANEOUS;
        break;

    case PICK_WAIT_SIMULTANEOUS:
        if (armMotor.isAtTarget() && turnMotor.isAtTarget()) {
            Serial.println("Pick seq: both at position, waiting 0.5s");
            lift_pause_timer = millis();
            lift_state = PICK_PAUSE_2;
        }
        break;

    case PICK_PAUSE_2:
        if (millis() - lift_pause_timer >= 500) {
            lift_state = PICK_ARM_TO_1480;
        }
        break;

    case PICK_ARM_TO_1480:
        armMotor.setTarget(1480);
        Serial.println("Pick seq: arm -> 1480");
        lift_state = PICK_WAIT_ARM_1480;
        break;

    case PICK_WAIT_ARM_1480:
        if (armMotor.isAtTarget()) {
            Serial.println("Pick seq: arm at 1480, activating pumps");
            lift_state = PICK_PUMPS_ON;
        }
        break;

    case PICK_PUMPS_ON:
        for (int i = 0; i < 4; i++) {
            if (requested_pumps[i]) setPumpState(i + 1, true);
        }
        Serial.println("Pick seq: pumps ON, holding 2s");
        lift_pause_timer = millis();
        lift_state = PICK_PAUSE_HOLD;
        break;

    case PICK_PAUSE_HOLD:
        if (millis() - lift_pause_timer >= 2000) {
            lift_state = PICK_DONE;
        }
        break;

    case PICK_DONE:
        Serial.println("Pick done, starting home sequence");
        lift_state = HOME_ARM_TO_800;
        break;

    // ----------------------------------------
    // Home sequence
    // ----------------------------------------

    case HOME_ARM_TO_800:
        armMotor.setTarget(800);
        Serial.println("Home seq: arm -> 800");
        lift_state = HOME_WAIT_ARM_800;
        break;

    case HOME_WAIT_ARM_800:
        if (armMotor.isAtTarget()) {
            Serial.println("Home seq: arm at 800, waiting 0.5s");
            lift_pause_timer = millis();
            lift_state = HOME_PAUSE_1;
        }
        break;

    case HOME_PAUSE_1:
        if (millis() - lift_pause_timer >= 500) {
            lift_state = HOME_SIMULTANEOUS;
        }
        break;

    case HOME_SIMULTANEOUS:
        turnMotor.setTarget(500);
        armMotor.setTarget(325);
        Serial.println("Home seq: turn -> 500, arm -> 325 simultaneously");
        lift_state = HOME_WAIT_SIMULTANEOUS;
        break;

    case HOME_WAIT_SIMULTANEOUS:
        if (armMotor.isAtTarget() && turnMotor.isAtTarget()) {
            Serial.println("Home seq: both at position, waiting 0.5s");
            lift_pause_timer = millis();
            lift_state = HOME_PAUSE_2;
        }
        break;

    case HOME_PAUSE_2:
        if (millis() - lift_pause_timer >= 500) {
            lift_state = HOME_TURN_TO_0;
        }
        break;

    case HOME_TURN_TO_0:
        turnMotor.setTarget(0);
        Serial.println("Home seq: turn -> 0");
        lift_state = HOME_WAIT_TURN_0;
        break;

    case HOME_WAIT_TURN_0:
        if (turnMotor.isAtTarget()) {
            Serial.println("Home seq: turn at 0, waiting 0.5s");
            lift_pause_timer = millis();
            lift_state = HOME_PAUSE_3;
        }
        break;

    case HOME_PAUSE_3:
        if (millis() - lift_pause_timer >= 500) {
            lift_state = HOME_ARM_TO_0;
        }
        break;

    case HOME_ARM_TO_0:
        armMotor.setTarget(-20);
        turnMotor.setTarget(-20);
        Serial.println("Home seq: arm -> 0");
        lift_state = HOME_WAIT_ARM_0;
        break;

    case HOME_WAIT_ARM_0:
        if (armMotor.isAtTarget()) {
            Serial.println("Home seq: at home, turning pumps off");
            lift_state = HOME_PUMPS_OFF;
        }
        break;

case HOME_PUMPS_OFF:
        allPumpsOff();
        Serial.println("Home seq: all pumps OFF, waiting");
        lift_pause_timer = millis();
        lift_state = HOME_PUMPS_WAIT;
        break;

    case HOME_PUMPS_WAIT:
        if (millis() - lift_pause_timer >= 2000) {  // tune this
            lift_state = HOME_FINAL_RETURN;
        }
        break;

    case HOME_FINAL_RETURN:
        turnMotor.setTarget(0);
        armMotor.setTarget(0);
        Serial.println("Home seq: complete!");
        lift_state = HOME_DONE;
        break;

    case HOME_DONE:
        break;
    }
}

// ----------------------------------------
// SECTION: Setup
// ----------------------------------------

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
}

// ----------------------------------------
// SECTION: Main loop
// ----------------------------------------

void loop() {
    static uint32_t last = micros();
    uint32_t now = micros();
    const uint32_t interval = 2000;

    if (now - last >= interval) {
        last += interval;
        armMotor.update();
        turnMotor.update();
    }

    if (flipper_b_pending && (millis() - flipper_b_timer >= FLIP_B_DELAY_MS)) {
        flipper_b_pending = false;
        flipper_servo.write(FLIP_B_END);
        Serial.println("Flipper B: now at 140");
    }

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

        if (cmd_type_upper.startsWith("P") && cmd_type_upper.length() > 1) {
            if (lift_state != LIFT_IDLE &&
                lift_state != PICK_DONE &&
                lift_state != HOME_DONE) {
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
            lift_state = PICK_ARM_TO_300;
        }
        else if (cmd_type_upper == "S") {
            if      (cmd_value_upper == "0") setStopperPosition(false);
            else if (cmd_value_upper == "1") setStopperPosition(true);
            else Serial.println("Error: Use S 0 or S 1");
        }
        else if (cmd_type_upper == "F") {
            if      (cmd_value_upper == "0") setFlipperStop();
            else if (cmd_value_upper == "B") setFlipperB();
            else if (cmd_value_upper == "Y") setFlipperY();
            else Serial.println("Error: Use F 0, F B, or F Y");
        }
        else {
            Serial.print("Unknown command: ");
            Serial.println(cmd_type);
        }
    }
}