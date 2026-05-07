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
    /*kp*/ 0.8f, /*ki*/ 0.05f, /*kd*/ 0.01f,
    /*in1*/ MOTOR_TURN1, /*in2*/ MOTOR_TURN2,
    /*enc_a*/ ENCODER_TURN_A, /*enc_b*/ ENCODER_TURN_B, /*enc_x*/ ENCODER_TURN_X,
    /*pwm_ch_fwd*/ PWM_CHANNEL_TURN1, /*pwm_ch_rev*/ PWM_CHANNEL_TURN2);

// ----------------------------------------
// SECTION: Sequence state machine
// ----------------------------------------

enum LiftState {
    LIFT_IDLE,

    // Pick sequence
    PICK_ARM_500,
    PICK_WAIT_ARM_500,
    PICK_PAUSE_1,
    PICK_TURN_400,
    PICK_WAIT_TURN_400,
    PICK_PAUSE_2,
    PICK_ARM_900,
    PICK_WAIT_ARM_900,
    PICK_PAUSE_3,
    PICK_TURN_1050,
    PICK_WAIT_TURN_1050,
    PICK_PAUSE_4,
    PICK_ARM_1475,
    PICK_WAIT_ARM_1475,
    PICK_PAUSE_5,
    PICK_PUMPS_ON,
    WIGGLE,
    WIGGLE_WAIT,
    WIGGLE_START,
    WIGGLE_END,
    START_WIGGLE,
    WIGGLE_WAIT2,
    WIGGLE_WAIT3,
    PICK_PAUSE_6,
    PICK_PAUSE_7,
    PICK_PAUSE_HOLD,
    PICK_DONE,
    PICK_TURN,
    PICK_WAIT_TURN,
    PICK_ARM,
    PICK_WAIT_ARM,
    PICK_PAUSE_8,
    PICK_PAUSE_9,
    WIGGLE_END3,
    WIGGLE_WAIT4,
    PICK_PAUSE_HOLD2,


    // Home sequence
    HOME_ARM_1200,
    HOME_WAIT_ARM_1200,
    HOME_PAUSE_1,
    HOME_TURN_450,
    HOME_WAIT_TURN_450,
    HOME_PAUSE_2,
    HOME_ARM_550,
    HOME_WAIT_ARM_550,
    HOME_PAUSE_3,
    HOME_TURN_0,
    HOME_WAIT_TURN_0,
    HOME_PAUSE_4,
    HOME_ARM_200,
    HOME_WAIT_ARM_200,
    HOME_PUMPS_OFF,
    HOME_PAUSE_5,
    HOME_ARM_0,
    HOME_WAIT_ARM_0,
    HOME_DONE,
    HOME_PAUSE_14
};

LiftState lift_state         = LIFT_IDLE;
uint32_t  lift_pause_timer   = 0;
bool      requested_pumps[4] = {false, false, false, false};

// ----------------------------------------
// SECTION: Flipper sequence state machine
// ----------------------------------------

enum FlipperState {
    FLIPPER_IDLE,

    // B sequence (blue)
    FLIPPER_B_STOPPER_OPEN,
    FLIPPER_B_STOPPER_WAIT,
    FLIPPER_B_STOPPER_CLOSE,
    FLIPPER_B_STOPPER_CLOSE_WAIT,
    FLIPPER_B_FLIP,
    FLIPPER_B_FLIP_WAIT,
    FLIPPER_B_RETURN,
    FLIPPER_B_RETURN_WAIT,
    FLIPPER_B_DONE,

    // Y sequence (yellow)
    FLIPPER_Y_STOPPER_OPEN,
    FLIPPER_Y_STOPPER_WAIT,
    FLIPPER_Y_STOPPER_CLOSE,
    FLIPPER_Y_STOPPER_CLOSE_WAIT,
    FLIPPER_Y_FLIP,
    FLIPPER_Y_FLIP_WAIT,
    FLIPPER_Y_RETURN,
    FLIPPER_Y_RETURN_WAIT,
    FLIPPER_Y_DONE,
};

FlipperState flipper_state       = FLIPPER_IDLE;
uint32_t     flipper_pause_timer = 0;

// ----------------------------------------
// SECTION: Servo tuning values
// ----------------------------------------

// Stopper
constexpr int STOP_CLOSED_ANGLE = 180;   // tune this
constexpr int STOP_OPEN_ANGLE   = 120;   // tune this

// Flipper
constexpr int      FLIP_STOP          = 60;    // resting position
constexpr int      FLIP_COLOR_START   = 50;    // both colors go here first
constexpr int      FLIP_B_END_COLOR   = 120;   // blue end position
constexpr int      FLIP_Y_END_COLOR   = 40;    // yellow end position
constexpr uint32_t FLIPPER_SEQ_DELAY  = 1000;   // delay between each action

// ----------------------------------------
// SECTION: Servo objects and state
// ----------------------------------------

Servo stopper_servo;
Servo flipper_servo;

bool pump_states[4] = {false, false, false, false};
bool stopper_open   = false;


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

void setFlipperStop() {
    flipper_servo.write(FLIP_STOP);
    Serial.println("Flipper STOP");
}

void startFlipperB() {
    if (flipper_state != FLIPPER_IDLE) {
        Serial.println("Flipper sequence already running");
        return;
    }
    flipper_state = FLIPPER_B_STOPPER_OPEN;
    Serial.println("Flipper B sequence started");
}

void startFlipperY() {
    if (flipper_state != FLIPPER_IDLE) {
        Serial.println("Flipper sequence already running");
        return;
    }
    flipper_state = FLIPPER_Y_STOPPER_OPEN;
    Serial.println("Flipper Y sequence started");
}

void updateFlipperSequence() {
    switch (flipper_state) {
        case FLIPPER_IDLE:
            break;

        // ----------------------------------------
        // B sequence (blue)
        // ----------------------------------------

        case FLIPPER_B_STOPPER_OPEN:
            setStopperPosition(true);
            flipper_pause_timer = millis();
            flipper_state = FLIPPER_B_STOPPER_WAIT;
            break;

        case FLIPPER_B_STOPPER_WAIT:
            if (millis() - flipper_pause_timer >= FLIPPER_SEQ_DELAY) {
                flipper_state = FLIPPER_B_STOPPER_CLOSE;
            }
            break;

        case FLIPPER_B_STOPPER_CLOSE:
            setStopperPosition(false);
            flipper_pause_timer = millis();
            flipper_state = FLIPPER_B_STOPPER_CLOSE_WAIT;
            break;

        case FLIPPER_B_STOPPER_CLOSE_WAIT:
            if (millis() - flipper_pause_timer >= FLIPPER_SEQ_DELAY) {
                flipper_state = FLIPPER_B_FLIP;
            }
            break;

        case FLIPPER_B_FLIP:
            flipper_servo.write(FLIP_COLOR_START);
            flipper_pause_timer = millis();
            flipper_state = FLIPPER_B_FLIP_WAIT;
            Serial.println("Flipper B: at start, going to 120");
            break;

        case FLIPPER_B_FLIP_WAIT:
            if (millis() - flipper_pause_timer >= FLIPPER_SEQ_DELAY) {
                flipper_servo.write(FLIP_B_END_COLOR);
                flipper_pause_timer = millis();
                flipper_state = FLIPPER_B_RETURN;
                Serial.println("Flipper B: at end position");
            }
            break;

        case FLIPPER_B_RETURN:
            if (millis() - flipper_pause_timer >= FLIPPER_SEQ_DELAY) {
                setFlipperStop();
                flipper_pause_timer = millis();
                flipper_state = FLIPPER_B_RETURN_WAIT;
            }
            break;

        case FLIPPER_B_RETURN_WAIT:
            if (millis() - flipper_pause_timer >= FLIPPER_SEQ_DELAY) {
                flipper_state = FLIPPER_B_DONE;
            }
            break;

        case FLIPPER_B_DONE:
            Serial.println("Flipper B sequence complete");
            flipper_state = FLIPPER_IDLE;
            break;

        // ----------------------------------------
        // Y sequence (yellow)
        // ----------------------------------------

        case FLIPPER_Y_STOPPER_OPEN:
            setStopperPosition(true);
            flipper_pause_timer = millis();
            flipper_state = FLIPPER_Y_STOPPER_WAIT;
            break;

        case FLIPPER_Y_STOPPER_WAIT:
            if (millis() - flipper_pause_timer >= FLIPPER_SEQ_DELAY) {
                flipper_state = FLIPPER_Y_STOPPER_CLOSE;
            }
            break;

        case FLIPPER_Y_STOPPER_CLOSE:
            setStopperPosition(false);
            flipper_pause_timer = millis();
            flipper_state = FLIPPER_Y_STOPPER_CLOSE_WAIT;
            break;

        case FLIPPER_Y_STOPPER_CLOSE_WAIT:
            if (millis() - flipper_pause_timer >= FLIPPER_SEQ_DELAY) {
                flipper_state = FLIPPER_Y_FLIP;
            }
            break;

        case FLIPPER_Y_FLIP:
            flipper_servo.write(FLIP_COLOR_START);
            flipper_pause_timer = millis();
            flipper_state = FLIPPER_Y_FLIP_WAIT;
            Serial.println("Flipper Y: at start, going to 40");
            break;

        case FLIPPER_Y_FLIP_WAIT:
            if (millis() - flipper_pause_timer >= FLIPPER_SEQ_DELAY) {
                flipper_servo.write(FLIP_Y_END_COLOR);
                flipper_pause_timer = millis();
                flipper_state = FLIPPER_Y_RETURN;
                Serial.println("Flipper Y: at end position");
            }
            break;

        case FLIPPER_Y_RETURN:
            if (millis() - flipper_pause_timer >= FLIPPER_SEQ_DELAY) {
                setFlipperStop();
                flipper_pause_timer = millis();
                flipper_state = FLIPPER_Y_RETURN_WAIT;
            }
            break;

        case FLIPPER_Y_RETURN_WAIT:
            if (millis() - flipper_pause_timer >= FLIPPER_SEQ_DELAY) {
                flipper_state = FLIPPER_Y_DONE;
            }
            break;

        case FLIPPER_Y_DONE:
            Serial.println("Flipper Y sequence complete");
            flipper_state = FLIPPER_IDLE;
            break;
    }
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

    case PICK_ARM_500:
        armMotor.resetEncoder();
        turnMotor.resetEncoder();
        armMotor.enable();
        turnMotor.enable();
        armMotor.setTarget(500);
        Serial.println("Pick: arm -> 500");
        lift_state = PICK_WAIT_ARM_500;
        break;

    case PICK_WAIT_ARM_500:
        if (armMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = PICK_PAUSE_1;
        }
        break;

    case PICK_PAUSE_1:
        if (millis() - lift_pause_timer >= 500) {
            lift_state = PICK_TURN_400;
        }
        break;

    case PICK_TURN_400:
        turnMotor.setTarget(450);
        Serial.println("Pick: turn -> 400");
        lift_state = PICK_WAIT_TURN_400;
        break;

    case PICK_WAIT_TURN_400:
        if (turnMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = PICK_PAUSE_2;
        }
        break;

    case PICK_PAUSE_2:
        if (millis() - lift_pause_timer >= 500) {
            lift_state = PICK_ARM_900;
        }
        break;

    case PICK_ARM_900:
        armMotor.setTarget(1100);
        Serial.println("Pick: arm -> 900");
        lift_state = PICK_WAIT_ARM_900;
        break;

    case PICK_WAIT_ARM_900:
        if (armMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = PICK_PAUSE_3;
        }
        break;

    case PICK_PAUSE_3:
        if (millis() - lift_pause_timer >= 500) {
            lift_state = PICK_TURN_1050;
        }
        break;

    case PICK_TURN_1050:
        turnMotor.setTarget(1050);
        Serial.println("Pick: turn -> 1050");
        lift_state = PICK_WAIT_TURN_1050;
        break;

    case PICK_WAIT_TURN_1050:
        if (turnMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = PICK_PAUSE_4;
        }
        break;

    case PICK_PAUSE_4:
        if (millis() - lift_pause_timer >= 500) {
            lift_state = PICK_ARM;
        }
        break;

    // case PICK_ARM_1475:
    //     armMotor.setTarget(1100);
    //     Serial.println("Pick: arm -> 1475");
    //     lift_state = PICK_WAIT_ARM_1475;
    //     break;

    // case PICK_WAIT_ARM_1475:
    //     if (armMotor.isAtTarget()) {
    //         lift_pause_timer = millis();
    //         lift_state = PICK_PAUSE_5;
    //     }
    //     break;

    // case PICK_PAUSE_5:
    //     if (millis() - lift_pause_timer >= 500) {
    //         lift_state = PICK_TURN;
    //     }
    //     break;

    //     case PICK_TURN:
    //     turnMotor.setTarget(1050);
    //     Serial.println("Pick: turn -> 1050");
    //     lift_state = PICK_WAIT_TURN;
    //     break;

    // case PICK_WAIT_TURN:
    //     if (turnMotor.isAtTarget()) {
    //         lift_pause_timer = millis();
    //         lift_state = PICK_PAUSE_9;
    //     }
    //     break;

    // case PICK_PAUSE_9:
    //     if (millis() - lift_pause_timer >= 500) {
    //         lift_state = PICK_ARM;
    //     }
    //     break;

        case PICK_ARM:
        armMotor.setTarget(1350);
        Serial.println("Pick: arm -> 1350");
        lift_state = PICK_WAIT_ARM;
        break;

    case PICK_WAIT_ARM:
        if (armMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = PICK_PAUSE_8;
        }
        break;

    case PICK_PAUSE_8:
        if (millis() - lift_pause_timer >= 500) {
            lift_state = PICK_PUMPS_ON;
        }
        break;

    case PICK_PUMPS_ON:
        for (int i = 0; i < 4; i++) {
            if (requested_pumps[i]) setPumpState(i + 1, true);
        }
        Serial.println("Pick: pumps ON, holding 1s");
        lift_pause_timer = millis();
        lift_state = START_WIGGLE;
        break;

    case START_WIGGLE:
        armMotor.setTarget(1450);
        Serial.println("Pick: arm -> 1450");
        lift_state = WIGGLE_WAIT;
        break;

    case WIGGLE_WAIT:
        if (armMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = PICK_PAUSE_HOLD;
        }
        break;

    // case PICK_PAUSE_6:
    //     if (millis() - lift_pause_timer >= 500) {
    //         lift_state = WIGGLE;
    //     }
    //     break;

    // case WIGGLE:
    //     turnMotor.setTarget(1075);
    //     Serial.println("Pick: turn -> 1075");
    //     lift_state = WIGGLE_WAIT2;
    //     break;

    // case WIGGLE_WAIT2:
    //     if (turnMotor.isAtTarget()) {
    //         lift_pause_timer = millis();
    //         lift_state = PICK_PAUSE_7;
    //     }
    //     break;

    // case PICK_PAUSE_7:
    //     if (millis() - lift_pause_timer >= 500) {
    //         lift_state = WIGGLE_END;
    //     }
    //     break;

    // case WIGGLE_END:
    //     turnMotor.setTarget(1050);
    //     Serial.println("Pick: turn -> 1050");
    //     lift_state = WIGGLE_WAIT3;
    //     break;

    // case WIGGLE_WAIT3:
    //     if (turnMotor.isAtTarget()) {
    //         lift_pause_timer = millis();
    //         lift_state = PICK_PAUSE_HOLD;
    //     }
    //     break;

    case PICK_PAUSE_HOLD:
        if (millis() - lift_pause_timer >= 1000) {
            lift_state = PICK_DONE;
        }
        break;

    case PICK_DONE:
        Serial.println("Pick done, starting home sequence");
        lift_state = HOME_ARM_1200;
        break;

    // ----------------------------------------
    // Home sequence
    // ----------------------------------------

    case HOME_ARM_1200:
        armMotor.setTarget(1000);
        Serial.println("Home: arm -> 1200");
        lift_state = HOME_WAIT_ARM_1200;
        break;

    case HOME_WAIT_ARM_1200:
        if (armMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = HOME_PAUSE_1;
        }
        break;

    case HOME_PAUSE_1:
        if (millis() - lift_pause_timer >= 500) {
            lift_state = HOME_TURN_450;
        }
        break;

    case HOME_TURN_450:
        turnMotor.setTarget(800);
        Serial.println("Home: turn -> 450");
        lift_state = HOME_WAIT_TURN_450;
        break;

    case HOME_WAIT_TURN_450:
        if (turnMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = HOME_PAUSE_2;
        }
        break;

    case HOME_PAUSE_2:
        if (millis() - lift_pause_timer >= 500) {
            lift_state = HOME_ARM_550;
        }
        break;

    case HOME_ARM_550:
        armMotor.setTarget(800);
        Serial.println("Home: arm -> 550");
        lift_state = HOME_WAIT_ARM_550;
        break;

    case HOME_WAIT_ARM_550:
        if (armMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = HOME_PAUSE_3;
        }
        break;

    case HOME_PAUSE_3:
        if (millis() - lift_pause_timer >= 500) {
            lift_state = HOME_TURN_0;
        }
        break;

    case HOME_TURN_0:
        turnMotor.setTarget(500);
        Serial.println("Home: turn -> 0");
        lift_state = HOME_WAIT_TURN_0;
        break;

    case HOME_WAIT_TURN_0:
        if (turnMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = HOME_PAUSE_4;
        }
        break;

    case HOME_PAUSE_4:
        if (millis() - lift_pause_timer >= 500) {
            lift_state = HOME_ARM_200;
        }
        break;

    case HOME_ARM_200:
        armMotor.setTarget(500);
        Serial.println("Home: arm -> 200");
        lift_state = PICK_PAUSE_7;
        break;

    case PICK_PAUSE_7:
        if (armMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = HOME_PAUSE_14;
        }
        break;

    case HOME_PAUSE_14:
        if (millis() - lift_pause_timer >= 500) {
            lift_state = WIGGLE_END;
        }
        break;

    case WIGGLE_END:
        turnMotor.setTarget(0);
        Serial.println("Pick: turn -> 1050");
        lift_state = WIGGLE_WAIT3;
        break;

    case WIGGLE_WAIT3:
        if (turnMotor.isAtTarget()) {
            lift_pause_timer = millis();
            lift_state = PICK_PAUSE_HOLD2;
        }
        break;

    case PICK_PAUSE_HOLD2:
        if (millis() - lift_pause_timer >= 500) {
            lift_state = WIGGLE_END3;
        }
        break;

    case WIGGLE_END3:
        armMotor.setTarget(200);
        Serial.println("Pick: arm -> 200");
        lift_state = WIGGLE_WAIT4;
        break;

    case WIGGLE_WAIT4:
        if (armMotor.isAtTarget()) {
            lift_state = HOME_PUMPS_OFF;
        }
        break;

    case HOME_PUMPS_OFF:
        allPumpsOff();
        Serial.println("Home: pumps OFF");
        lift_pause_timer = millis();
        lift_state = HOME_PAUSE_5;
        break;

    case HOME_PAUSE_5:
        if (millis() - lift_pause_timer >= 500) {
            lift_state = HOME_ARM_0;
        }
        break;

    case HOME_ARM_0:
        armMotor.setTarget(0);
        Serial.println("Home: arm -> 0");
        lift_state = HOME_WAIT_ARM_0;
        break;

    case HOME_WAIT_ARM_0:
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
    armMotor.enable();
    turnMotor.enable();
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

        if (cmd_type_upper.startsWith("P") && cmd_type_upper.length() > 1) {
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
            lift_state = PICK_ARM_500;
        }
        else if (cmd_type_upper == "S") {
            if      (cmd_value_upper == "0") setStopperPosition(false);
            else if (cmd_value_upper == "1") setStopperPosition(true);
            else Serial.println("Error: Use S 0 or S 1");
        }
        else if (cmd_type_upper == "F") {
            if (cmd_value_upper == "0") {
                flipper_state = FLIPPER_IDLE;
                setFlipperStop();
            }
            else if (cmd_value_upper == "B") startFlipperB();
            else if (cmd_value_upper == "Y") startFlipperY();
            else Serial.println("Error: Use F 0, F B, or F Y");
        }
        else {
            Serial.print("Unknown command: ");
            Serial.println(cmd_type);
        }
    }
}