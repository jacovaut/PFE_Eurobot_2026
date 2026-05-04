/*
 * Manipulation Test Serial Commands
 * -----------------------------------------------
 * Pumps:
 *   P1 1, P2 1, P3 1, P4 1       -> turn an individual pump ON
 *   P1 0, P13 0, P1234 1         -> turn listed pumps OFF/ON
 *
 * Motors:
 *   T                            -> start lift sequence
 *   H                            -> arm to 50 then home after 1s
 *   U                            -> turn motor to 100 ticks
 *   Y                            -> arm motor to 500 ticks
 *   W                            -> free-wheel both motors
 *   E                            -> print both encoder counts
 *
 * Servo Control:
 *   S 0, S 1                     -> Close or Open stopper
 *   S <angle>                    -> Set stopper to specific angle (0-180)
 *   F CW, F CCW, F C             -> Set flipper position
 *   F <angle>                    -> Set flipper to specific angle (0-180)
 *   C YELLOW, C BLUE             -> Flip based on detected color
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
constexpr int Thermo = 33;
constexpr int STOP = 13;
constexpr int FLIP = 12;

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

// PWM channels:
// 0, 1 → free
// 2, 3 → servo timers (allocateTimer 0 and 1)
// 4, 5 → arm motor
// 6, 7 → turn motor
constexpr int PWM_CHANNEL_LIFT1 = 4;
constexpr int PWM_CHANNEL_LIFT2 = 5;
constexpr int PWM_CHANNEL_TURN1 = 6;
constexpr int PWM_CHANNEL_TURN2 = 7;

const int PUMP_PINS[4] = {Pump1, Pump2, Pump3, Pump4};

// ----------------------------------------
// SECTION: Motor objects
// ----------------------------------------

ArmMotor armMotor(
    /*kp*/ 2.0f, /*ki*/ 0.0f, /*kd*/ 0.0f,
    /*in1*/ MOTOR_LIFT1, /*in2*/ MOTOR_LIFT2,
    /*enc_a*/ ENCODER_LIFT_A, /*enc_b*/ ENCODER_LIFT_B, /*enc_x*/ ENCODER_LIFT_X,
    /*pwm_ch_fwd*/ PWM_CHANNEL_LIFT1, /*pwm_ch_rev*/ PWM_CHANNEL_LIFT2
);

TurnMotor turnMotor(
    /*kp*/ 1.0f, /*ki*/ 0.0f, /*kd*/ 0.0f,
    /*in1*/ MOTOR_TURN1, /*in2*/ MOTOR_TURN2,
    /*enc_a*/ ENCODER_TURN_A, /*enc_b*/ ENCODER_TURN_B, /*enc_x*/ ENCODER_TURN_X,
    /*pwm_ch_fwd*/ PWM_CHANNEL_TURN1, /*pwm_ch_rev*/ PWM_CHANNEL_TURN2
);

// ----------------------------------------
// SECTION: Lift sequence state machine
// ----------------------------------------

enum LiftState {
    LIFT_IDLE,

    // Pick sequence (down, negative ticks)
    PICK_ARM_TO_350,
    PICK_WAIT_ARM_350,
    PICK_PAUSE_1,
    PICK_SIMULTANEOUS,
    PICK_WAIT_SIMULTANEOUS,
    PICK_PAUSE_2,
    PICK_TURN_TO_1000,
    PICK_WAIT_TURN_1000,
    PICK_PAUSE_3,
    PICK_ARM_TO_1325,
    PICK_WAIT_ARM_1325,
    PICK_DONE,

    // Home sequence (up, back to zero)
    HOME_ARM_TO_1000,
    HOME_WAIT_ARM_1000,
    HOME_PAUSE_1,
    HOME_SIMULTANEOUS,
    HOME_WAIT_SIMULTANEOUS,
    HOME_PAUSE_2,
    HOME_ARM_TO_100,
    HOME_WAIT_ARM_100,
    HOME_PAUSE_3,
    HOME_ARM_TO_0,
    HOME_WAIT_ARM_0,
    HOME_DONE
};

LiftState lift_state       = LIFT_IDLE;
uint32_t  lift_pause_timer = 0;

// ----------------------------------------
// SECTION: Servo objects and state
// ----------------------------------------

Servo stopper_servo;
Servo flipper_servo;

bool   pump_states[4]    = {false, false, false, false};
bool   stopper_open      = false;
int    stopper_angle     = 15;
int    flipper_angle     = 90;
String flipper_state     = "CENTER";
String last_block_color  = "UNKNOWN";

constexpr int STOP_CLOSED_ANGLE = 15;
constexpr int STOP_OPEN_ANGLE   = 95;
constexpr int FLIP_CENTER_ANGLE = 90;
constexpr int FLIP_CW_ANGLE     = 35;
constexpr int FLIP_CCW_ANGLE    = 145;

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

bool applyPumpCommand(String pump_nums, bool state) {
    bool requested[4] = {false, false, false, false};
    if (pump_nums.length() == 0) pump_nums = "1234";

    for (int i = 0; i < pump_nums.length(); i++) {
        char digit = pump_nums[i];
        if (digit >= '1' && digit <= '4') {
            requested[digit - '1'] = true;
        } else {
            return false;
        }
    }
    for (int i = 0; i < 4; i++) {
        if (requested[i]) setPumpState(i + 1, state);
    }
    return true;
}

// ----------------------------------------
// SECTION: Servo functions
// ----------------------------------------

void setStopperAngle(int angle) {
    stopper_angle = constrain(angle, 0, 180);
    stopper_servo.write(stopper_angle);
    stopper_open = stopper_angle > ((STOP_CLOSED_ANGLE + STOP_OPEN_ANGLE) / 2);
}

void setStopperPosition(bool open) {
    stopper_open = open;
    setStopperAngle(open ? STOP_OPEN_ANGLE : STOP_CLOSED_ANGLE);
}

void setFlipperAngle(int angle) {
    flipper_angle = constrain(angle, 0, 180);
    flipper_servo.write(flipper_angle);
    if      (flipper_angle == FLIP_CW_ANGLE)     flipper_state = "CW";
    else if (flipper_angle == FLIP_CCW_ANGLE)    flipper_state = "CCW";
    else if (flipper_angle == FLIP_CENTER_ANGLE) flipper_state = "CENTER";
    else                                          flipper_state = "CUSTOM";
}

void setFlipperClockwise()        { setFlipperAngle(FLIP_CW_ANGLE); }
void setFlipperCounterClockwise() { setFlipperAngle(FLIP_CCW_ANGLE); }
void centerFlipper()              { setFlipperAngle(FLIP_CENTER_ANGLE); }

void flipForColor(const String& color) {
    last_block_color = color;
    if      (color == "YELLOW") setFlipperClockwise();
    else if (color == "BLUE")   setFlipperCounterClockwise();
}

// ----------------------------------------
// SECTION: Lift sequence
// ----------------------------------------

void updateLiftSequence() {
    switch (lift_state) {
        case LIFT_IDLE:
            break;

        // ----------------------------------------
        // P command — Pick sequence (PID controlled)
        // ----------------------------------------

        case PICK_ARM_TO_350:
            armMotor.setTarget(500);
            Serial.println("Pick seq: arm → 500");
            lift_state = PICK_WAIT_ARM_350;
            break;

        case PICK_WAIT_ARM_350:
            if (armMotor.isAtTarget()) {
                Serial.println("Pick seq: arm at 500, waiting 0.5s");
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
            turnMotor.setTarget(1000);
            armMotor.setTarget(800);
            Serial.println("Pick seq: turn → 1000, arm → 800 simultaneously");
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
                lift_state = PICK_TURN_TO_1000;
            }
            break;

        case PICK_TURN_TO_1000:
            armMotor.setTarget(1200);
            Serial.println("Pick seq: arm → 1200");
            lift_state = PICK_WAIT_TURN_1000;
            break;

        case PICK_WAIT_TURN_1000:
            if (armMotor.isAtTarget()) {
                Serial.println("Pick seq: arm at 1200, waiting 0.5s");
                lift_pause_timer = millis();
                lift_state = PICK_PAUSE_3;
            }
            break;

        case PICK_PAUSE_3:
            if (millis() - lift_pause_timer >= 500) {
                lift_state = PICK_ARM_TO_1325;
            }
            break;

        case PICK_ARM_TO_1325:
            armMotor.setTarget(1325);
            Serial.println("Pick seq: arm → 1325");
            lift_state = PICK_WAIT_ARM_1325;
            break;

        case PICK_WAIT_ARM_1325:
            if (armMotor.isAtTarget()) {
                Serial.println("Pick seq: complete!");
                lift_state = PICK_DONE;
            }
            break;

        case PICK_DONE:
            break;

        // ----------------------------------------
        // H command — Home sequence (255 full power up)
        // ----------------------------------------

        case HOME_ARM_TO_1000:
            armMotor.setTarget(800);
            Serial.println("Home seq: arm → 800");
            lift_state = HOME_WAIT_ARM_1000;
            break;

        case HOME_WAIT_ARM_1000:
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
            armMotor.setTarget(350);
            Serial.println("Home seq: turn → 500, arm → 325 simultaneously");
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
                lift_state = HOME_ARM_TO_100;
            }
            break;

        case HOME_ARM_TO_100:
            turnMotor.setTarget(0);
            Serial.println("Home seq: turn → 0");
            lift_state = HOME_WAIT_ARM_100;
            break;

        case HOME_WAIT_ARM_100:
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
            armMotor.setTarget(0);
            Serial.println("Home seq: arm → 0");
            lift_state = HOME_WAIT_ARM_0;
            break;

        case HOME_WAIT_ARM_0:
            if (armMotor.isAtTarget()) {
                Serial.println("Home seq: complete!");
                lift_state = HOME_DONE;
            }
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

    // Use timers 0 and 1 for servos — leaves timers 2 and 3 free for motor PWM
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    stopper_servo.setPeriodHertz(50);
    flipper_servo.setPeriodHertz(50);
    stopper_servo.attach(STOP, 500, 2400);
    flipper_servo.attach(FLIP, 500, 2400);
    setStopperPosition(false);
    centerFlipper();

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

    // Motor update at 500 Hz
    if (now - last >= interval) {
        last += interval;
        armMotor.update();
        turnMotor.update();
    }

    updateLiftSequence();

    // ----------------------------------------
    // Serial command handling
    // ----------------------------------------
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command.length() == 0) return;

        char operation = tolower(command.charAt(0));
        bool is_compact = (command.length() == 1) &&
                          (operation == 'p' || operation == 'h' ||
                           operation == 'x' || operation == 'e');

        if (is_compact) {
            switch (operation) {
                case 'p':
                if (lift_state != LIFT_IDLE &&
                        lift_state != PICK_DONE &&
                        lift_state != HOME_DONE) {
                        Serial.println("Sequence already running");
                } else {
                        lift_state = PICK_ARM_TO_350;
                        Serial.println("Pick sequence started");
                }
                break;

            case 'h':
                if (lift_state != LIFT_IDLE &&
                        lift_state != PICK_DONE &&
                        lift_state != HOME_DONE) {
                        Serial.println("Sequence already running");
                } else {
                        lift_state = HOME_ARM_TO_1000;
                        Serial.println("Home sequence started");
                }
                break;

                case 'x':
                        lift_state = LIFT_IDLE;
                        ledcWrite(PWM_CHANNEL_LIFT1, 0);
                        ledcWrite(PWM_CHANNEL_LIFT2, 0);
                        ledcWrite(PWM_CHANNEL_TURN1, 0);
                        ledcWrite(PWM_CHANNEL_TURN2, 0);
                        Serial.println("Sequence stopped, both motors off");
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

        if (cmd_type_upper.startsWith("P")) {
            if (cmd_value == "0" || cmd_value == "1") {
                bool state = (cmd_value == "1");
                String pump_nums = cmd_type_upper.substring(1);
                if (applyPumpCommand(pump_nums, state)) {
                    Serial.print("Pumps ");
                    Serial.print(pump_nums.length() == 0 ? "1234" : pump_nums);
                    Serial.println(state ? " ON" : " OFF");
                } else {
                    Serial.println("Error: Invalid pump number");
                }
            } else {
                Serial.println("Error: Use P<nums> 0 or P<nums> 1");
            }
        }
        else if (cmd_type_upper == "S") {
            if (cmd_value_upper == "0") {
                setStopperPosition(false);
                Serial.println("Stopper CLOSED");
            } else if (cmd_value_upper == "1") {
                setStopperPosition(true);
                Serial.println("Stopper OPEN");
            } else if (isNumeric(cmd_value)) {
                int angle = cmd_value.toInt();
                if (angle >= 0 && angle <= 180) {
                    setStopperAngle(angle);
                    Serial.print("Stopper → ");
                    Serial.print(stopper_angle);
                    Serial.println(" deg");
                } else {
                    Serial.println("Error: angle must be 0-180");
                }
            } else {
                Serial.println("Error: Use S 0, S 1, or S <angle>");
            }
        }
        else if (cmd_type_upper == "F") {
            if      (cmd_value_upper == "CW")  { setFlipperClockwise();        Serial.println("Flipper CW"); }
            else if (cmd_value_upper == "CCW") { setFlipperCounterClockwise(); Serial.println("Flipper CCW"); }
            else if (cmd_value_upper == "C")   { centerFlipper();              Serial.println("Flipper CENTER"); }
            else if (isNumeric(cmd_value)) {
                int angle = cmd_value.toInt();
                if (angle >= 0 && angle <= 180) {
                    setFlipperAngle(angle);
                    Serial.print("Flipper → ");
                    Serial.print(flipper_angle);
                    Serial.println(" deg");
                } else {
                    Serial.println("Error: angle must be 0-180");
                }
            } else {
                Serial.println("Error: Use F CW, F CCW, F C, or F <angle>");
            }
        }
        else if (cmd_type_upper == "C") {
            if (cmd_value_upper == "YELLOW" || cmd_value_upper == "BLUE") {
                flipForColor(cmd_value_upper);
                Serial.print("Flipper → color: ");
                Serial.println(cmd_value_upper);
            } else {
                Serial.println("Error: Use C YELLOW or C BLUE");
            }
        }
        else {
            Serial.print("Unknown command: ");
            Serial.println(cmd_type);
        }
    }
}