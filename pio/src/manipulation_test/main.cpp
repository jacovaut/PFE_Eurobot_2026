/*
 * Manipulation Test Serial Commands
 * ---------------------------------
 * Pumps:
 *   P1 1, P2 1, P3 1, P4 1       -> turn an individual pump ON
 *   P1 0, P13 0, P1234 1         -> turn listed pumps OFF/ON
 *
 * Lift motor:
 *   f###                         -> lift forward at speed 0..255
 *   b###                         -> lift backward at speed 0..255
 *   x                            -> lift brake
 *   c / r                        -> read / reset lift encoder
 *
 * Turn motor:
 *   a###                         -> turn motor one way at speed 0..255
 *   d###                         -> turn motor the other way at speed 0..255
 *   s                            -> turn brake
 *   t / p                        -> read / reset turn encoder
 *   TURN CW / TURN CCW           -> one full turn test (1050 ticks)
 *   TCW / TCCW                   -> short aliases for one full turn
 *   TURN <ticks>                 -> exact tick move; + = CW, - = CCW
 *
 * Servos / sorting:
 *   S 0, S 1, S <angle>          -> stopper closed, open, or custom angle
 *   F CW, F CCW, F CENTER        -> flipper positions
 *   F <angle>                    -> flipper custom angle
 *   COLOR YELLOW / COLOR BLUE    -> color-based flipper move
 *
 * Other:
 *   STATUS                       -> show all current states
 *   HELP                         -> show help in serial monitor
 */
#include <Arduino.h>
#include <stdint.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>

// ---- FROM PCB MANIPULATION ---- //

constexpr int Pump1 = 25; // Pump for suction cup 1
constexpr int Pump2 = 26; // Pump for suction cup 2
constexpr int Pump3 = 27; // Pump for suction cup 3
constexpr int Pump4 = 14; // Pump for suction cup 4
constexpr int Thermo = 33; // Control Thermo?
constexpr int STOP = 13; // Servo stopper for the block chute
constexpr int FLIP = 18; // Servo flipper for sorting blocks

constexpr int MOTOR_LIFT1 = 15;
constexpr int MOTOR_LIFT2 = 2;
constexpr int MOTOR_TURN1 = 4;
constexpr int MOTOR_TURN2 = 5;

constexpr int ENCODER_LIFT_A = 23;
constexpr int ENCODER_LIFT_B = 22;
constexpr int ENCODER_TURN_A = 21;
constexpr int ENCODER_TURN_B = 19;

constexpr int PWM_CHANNEL_LIFT1 = 0;
constexpr int PWM_CHANNEL_LIFT2 = 1;
constexpr int PWM_CHANNEL_TURN1 = 2;
constexpr int PWM_CHANNEL_TURN2 = 3;
constexpr int PWM_FREQ = 5000;
constexpr int PWM_RESOLUTION = 8;
constexpr int FULL_TURN_TICKS = 1050;
constexpr int TURN_TEST_SPEED = 180;

const int PUMP_PINS[4] = {Pump1, Pump2, Pump3, Pump4};

// constexpr int STOP = 23; // Coder for M1
// constexpr int FLIP = 22; // Coder for M1

// constexpr int STOP = 21; // Coder for M2
// constexpr int FLIP = 19; // Coder for M2



Servo stopper_servo;
Servo flipper_servo;
ESP32Encoder lift_encoder;
ESP32Encoder turn_encoder;

// Current state tracking
bool pump_states[4] = {false, false, false, false};
bool stopper_open = false;
int stopper_angle = 15;
int flipper_angle = 90;
int lift_speed = 0;
int turn_speed = 0;
bool turn_test_active = false;
int64_t turn_test_start_count = 0;
int64_t turn_test_target_ticks = 0;
String turn_test_direction = "IDLE";
String lift_state = "BRAKE";
String turn_state = "BRAKE";
String flipper_state = "CENTER";
String last_block_color = "UNKNOWN";

constexpr int STOP_CLOSED_ANGLE = 15;
constexpr int STOP_OPEN_ANGLE = 95;
constexpr int FLIP_CENTER_ANGLE = 90;
constexpr int FLIP_CW_ANGLE = 35;
constexpr int FLIP_CCW_ANGLE = 145;

bool isNumeric(const String& value) {
    if (value.length() == 0) {
        return false;
    }

    for (size_t i = 0; i < value.length(); i++) {
        if (!isDigit(value[i])) {
            return false;
        }
    }

    return true;
}

void setPumpState(uint8_t pump_number, bool on) {
    if (pump_number < 1 || pump_number > 4) {
        return;
    }

    pump_states[pump_number - 1] = on;
    digitalWrite(PUMP_PINS[pump_number - 1], on ? HIGH : LOW);
}

bool applyPumpCommand(String pump_nums, bool state) {
    bool requested[4] = {false, false, false, false};

    if (pump_nums.length() == 0) {
        pump_nums = "1234";
    }

    for (int i = 0; i < pump_nums.length(); i++) {
        char digit = pump_nums[i];
        if (digit >= '1' && digit <= '4') {
            requested[digit - '1'] = true;
        } else {
            return false;
        }
    }

    for (int i = 0; i < 4; i++) {
        if (requested[i]) {
            setPumpState(i + 1, state);
        }
    }

    return true;
}

void LmotorForward(int speed) {
    lift_speed = constrain(speed, 0, 255);
    lift_state = "FORWARD";
    ledcWrite(PWM_CHANNEL_LIFT2, 0);
    ledcWrite(PWM_CHANNEL_LIFT1, lift_speed);
    Serial.printf("Lift forward at speed %d\n", lift_speed);
}

void LmotorBackward(int speed) {
    lift_speed = constrain(speed, 0, 255);
    lift_state = "BACKWARD";
    ledcWrite(PWM_CHANNEL_LIFT2, lift_speed);
    ledcWrite(PWM_CHANNEL_LIFT1, 0);
    Serial.printf("Lift backward at speed %d\n", lift_speed);
}

void LmotorBrake() {
    lift_speed = 0;
    lift_state = "BRAKE";
    ledcWrite(PWM_CHANNEL_LIFT1, 255);
    ledcWrite(PWM_CHANNEL_LIFT2, 255);
    Serial.println("Lift braking fast");
}

void TmotorForward(int speed) {
    turn_speed = constrain(speed, 0, 255);
    turn_state = "FORWARD";
    ledcWrite(PWM_CHANNEL_TURN2, 0);
    ledcWrite(PWM_CHANNEL_TURN1, turn_speed);
    Serial.printf("Turn forward at speed %d\n", turn_speed);
}

void TmotorBackward(int speed) {
    turn_speed = constrain(speed, 0, 255);
    turn_state = "BACKWARD";
    ledcWrite(PWM_CHANNEL_TURN2, turn_speed);
    ledcWrite(PWM_CHANNEL_TURN1, 0);
    Serial.printf("Turn backward at speed %d\n", turn_speed);
}

void TmotorBrake() {
    turn_speed = 0;
    turn_state = "BRAKE";
    ledcWrite(PWM_CHANNEL_TURN1, 255);
    ledcWrite(PWM_CHANNEL_TURN2, 255);
    Serial.println("Turn braking fast");
}

void startTurnTickMove(int64_t ticks) {
    if (ticks == 0) {
        turn_test_active = false;
        turn_test_target_ticks = 0;
        turn_test_direction = "IDLE";
        TmotorBrake();
        Serial.println("Turn tick move cancelled");
        return;
    }

    turn_test_active = true;
    turn_test_start_count = turn_encoder.getCount();
    turn_test_target_ticks = (ticks > 0) ? ticks : -ticks;
    turn_test_direction = (ticks > 0) ? "CW" : "CCW";

    if (ticks > 0) {
        TmotorForward(TURN_TEST_SPEED);
    } else {
        TmotorBackward(TURN_TEST_SPEED);
    }

    Serial.printf("Starting turn move %s for %lld ticks at speed %d\n", turn_test_direction.c_str(), turn_test_target_ticks, TURN_TEST_SPEED);
}

void startFullTurnTest(bool clockwise) {
    startTurnTickMove(clockwise ? FULL_TURN_TICKS : -FULL_TURN_TICKS);
}

void updateFullTurnTest() {
    if (!turn_test_active) {
        return;
    }

    int64_t delta = turn_encoder.getCount() - turn_test_start_count;
    if (delta < 0) {
        delta = -delta;
    }

    if (delta >= turn_test_target_ticks) {
        turn_test_active = false;
        TmotorBrake();
        Serial.printf("Turn move %s complete: %lld / %lld ticks\n", turn_test_direction.c_str(), delta, turn_test_target_ticks);
        turn_test_direction = "IDLE";
        turn_test_target_ticks = 0;
    }
}

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

    if (flipper_angle == FLIP_CW_ANGLE) {
        flipper_state = "CW";
    } else if (flipper_angle == FLIP_CCW_ANGLE) {
        flipper_state = "CCW";
    } else if (flipper_angle == FLIP_CENTER_ANGLE) {
        flipper_state = "CENTER";
    } else {
        flipper_state = "CUSTOM";
    }
}

void setFlipperClockwise() {
    setFlipperAngle(FLIP_CW_ANGLE);
}

void setFlipperCounterClockwise() {
    setFlipperAngle(FLIP_CCW_ANGLE);
}

void centerFlipper() {
    setFlipperAngle(FLIP_CENTER_ANGLE);
}

void flipForColor(const String& color) {
    last_block_color = color;

    // Swap these two mappings if the hardware orientation is reversed.
    if (color == "YELLOW") {
        setFlipperClockwise();
    } else if (color == "BLUE") {
        setFlipperCounterClockwise();
    }
}

void printStatus() {
    Serial.print("\n=== Current Status ===\n");
    for (int i = 0; i < 4; i++) {
        Serial.print("Pump");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(pump_states[i] ? "ON" : "OFF");
    }
    Serial.print("Lift motor: ");
    Serial.print(lift_state);
    Serial.print(" (");
    Serial.print(lift_speed);
    Serial.println(")");
    Serial.print("Turn motor: ");
    Serial.print(turn_state);
    Serial.print(" (");
    Serial.print(turn_speed);
    Serial.println(")");
    Serial.printf("Lift encoder: %lld\n", lift_encoder.getCount());
    Serial.printf("Turn encoder: %lld\n", turn_encoder.getCount());
    Serial.print("Turn test: ");
    if (turn_test_active) {
        Serial.print(turn_test_direction);
        Serial.print(" RUNNING (");
        Serial.print(turn_test_target_ticks);
        Serial.println(" ticks)");
    } else {
        Serial.println("IDLE");
    }
    Serial.print("Stopper: ");
    Serial.print(stopper_open ? "OPEN" : "CLOSED");
    Serial.print(" (");
    Serial.print(stopper_angle);
    Serial.println(" deg)");
    Serial.print("Flipper: ");
    Serial.print(flipper_state);
    Serial.print(" (");
    Serial.print(flipper_angle);
    Serial.println(" deg)");
    Serial.print("Last color: ");
    Serial.println(last_block_color);
    Serial.print("=====================\n\n");
}

void printHelp() {
    Serial.print("\n=== Commands ===\n");
    Serial.print("P<nums> <0|1>    - Control pumps independently\n");
    Serial.print("                   Examples: P1 1, P2 1, P13 0, P1234 1\n");
    Serial.print("                   Pumps not named in the command keep their current state\n");
    Serial.print("f### / b### / x  - Lift motor forward, backward, brake\n");
    Serial.print("a### / d### / s  - Turn motor forward, backward, brake\n");
    Serial.print("TURN CW          - One full clockwise turn test, 1050 ticks\n");
    Serial.print("TURN CCW         - One full counter-clockwise turn test, 1050 ticks\n");
    Serial.print("TURN <ticks>     - Turn exact ticks: positive = CW, negative = CCW\n");
    Serial.print("                   Examples: TURN 250, TURN -400, TURN 1050\n");
    Serial.print("TCW / TCCW       - Short aliases for the same turn tests\n");
    Serial.print("c / r            - Read or reset lift encoder\n");
    Serial.print("t / p            - Read or reset turn encoder\n");
    Serial.print("S <0|1|angle>    - Stopper CLOSED (0), OPEN (1), or set angle\n");
    Serial.print("STOP <0|1>       - Same as S for the stopper servo\n");
    Serial.print("F <CW|CCW|CENTER|angle> - Control the flipper servo\n");
    Serial.print("COLOR <YELLOW|BLUE>     - Flip based on block color\n");
    Serial.print("STATUS           - Show current status\n");
    Serial.print("HELP             - Show this help message\n");
    Serial.print("=================\n\n");
}

void setup(){
    Serial.begin(115200);
    delay(1000); // Give serial time to initialize
    
    Serial.print("\n\nStarting Manipulation Test...\n");

    for (int i = 0; i < 4; i++) {
        pinMode(PUMP_PINS[i], OUTPUT);
        digitalWrite(PUMP_PINS[i], LOW);
    }

    lift_encoder.attachHalfQuad(ENCODER_LIFT_A, ENCODER_LIFT_B);
    lift_encoder.setCount(0);
    turn_encoder.attachHalfQuad(ENCODER_TURN_A, ENCODER_TURN_B);
    turn_encoder.setCount(0);

    ledcSetup(PWM_CHANNEL_LIFT1, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_LIFT1, PWM_CHANNEL_LIFT1);
    ledcSetup(PWM_CHANNEL_LIFT2, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_LIFT2, PWM_CHANNEL_LIFT2);
    ledcSetup(PWM_CHANNEL_TURN1, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_TURN1, PWM_CHANNEL_TURN1);
    ledcSetup(PWM_CHANNEL_TURN2, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_TURN2, PWM_CHANNEL_TURN2);

    LmotorBrake();
    TmotorBrake();

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    stopper_servo.setPeriodHertz(50);
    flipper_servo.setPeriodHertz(50);
    stopper_servo.attach(STOP, 500, 2400);
    flipper_servo.attach(FLIP, 500, 2400);
    setStopperPosition(false);
    centerFlipper();

    printHelp();
    printStatus();
}

void loop(){
    updateFullTurnTest();

    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command.length() == 0) {
            return;
        }

        char operation = tolower(command.charAt(0));
        bool is_compact_motor_cmd = false;

        if (command.length() == 1) {
            is_compact_motor_cmd = (operation == 'x' || operation == 's' || operation == 'c' || operation == 'r' || operation == 't' || operation == 'p');
        } else {
            is_compact_motor_cmd = ((operation == 'f' || operation == 'b' || operation == 'a' || operation == 'd') && isDigit(command.charAt(1)));
        }

        if (is_compact_motor_cmd) {
            int speed = 0;
            if (command.length() > 1) {
                speed = constrain(command.substring(1).toInt(), 0, 255);
            }

            switch (operation) {
                case 'f':
                    LmotorForward(speed);
                    break;
                case 'b':
                    LmotorBackward(speed);
                    break;
                case 'x':
                    LmotorBrake();
                    break;
                case 'a':
                    turn_test_active = false;
                    turn_test_direction = "IDLE";
                    TmotorForward(speed);
                    break;
                case 'd':
                    turn_test_active = false;
                    turn_test_direction = "IDLE";
                    TmotorBackward(speed);
                    break;
                case 's':
                    turn_test_active = false;
                    turn_test_direction = "IDLE";
                    TmotorBrake();
                    break;
                case 'c':
                    Serial.printf("Lift encoder count: %lld\n", lift_encoder.getCount());
                    break;
                case 'r':
                    lift_encoder.setCount(0);
                    Serial.println("Lift encoder count reset to 0");
                    break;
                case 't':
                    Serial.printf("Turn encoder count: %lld\n", turn_encoder.getCount());
                    break;
                case 'p':
                    turn_encoder.setCount(0);
                    Serial.println("Turn encoder count reset to 0");
                    break;
            }
            return;
        }

        int space_idx = command.indexOf(' ');
        String cmd_type = (space_idx > 0) ? command.substring(0, space_idx) : command;
        String cmd_value = (space_idx > 0) ? command.substring(space_idx + 1) : "";
        String cmd_type_upper = cmd_type;
        String cmd_value_upper = cmd_value;
        cmd_type_upper.toUpperCase();
        cmd_value_upper.toUpperCase();

        if (cmd_type_upper.startsWith("P")) {
            if (cmd_value == "0" || cmd_value == "1") {
                bool state = (cmd_value == "1");
                String pump_nums = cmd_type_upper.substring(1);

                if (applyPumpCommand(pump_nums, state)) {
                    Serial.print("Pump");
                    if (pump_nums.length() == 0) {
                        Serial.print("s 1234");
                    } else {
                        Serial.print("s ");
                        Serial.print(pump_nums);
                    }
                    Serial.print(" set to ");
                    Serial.println(state ? "ON" : "OFF");
                } else {
                    Serial.println("Error: Invalid pump number. Use P1, P2, P3, P4, or combos like P13");
                }
            } else {
                Serial.println("Error: Use P<nums> 0 or P<nums> 1");
            }
        }
        else if (cmd_type_upper == "TURN" || cmd_type_upper == "TCW" || cmd_type_upper == "TCCW") {
            if (cmd_type_upper == "TCW" || cmd_value_upper == "CW") {
                startFullTurnTest(true);
            }
            else if (cmd_type_upper == "TCCW" || cmd_value_upper == "CCW") {
                startFullTurnTest(false);
            }
            else if (cmd_value.length() > 0) {
                int64_t ticks = cmd_value.toInt();
                if (ticks != 0 || cmd_value == "0" || cmd_value == "+0" || cmd_value == "-0") {
                    startTurnTickMove(ticks);
                } else {
                    Serial.println("Error: Use TURN CW, TURN CCW, or TURN <ticks>");
                }
            }
            else {
                Serial.println("Error: Use TURN CW, TURN CCW, or TURN <ticks>");
            }
        }
        else if (cmd_type_upper == "S" || cmd_type_upper == "STOP") {
            if (cmd_value_upper == "0" || cmd_value_upper == "CLOSE" || cmd_value_upper == "CLOSED") {
                setStopperPosition(false);
                Serial.println("Stopper CLOSED");
            }
            else if (cmd_value_upper == "1" || cmd_value_upper == "OPEN") {
                setStopperPosition(true);
                Serial.println("Stopper OPEN");
            }
            else if (isNumeric(cmd_value)) {
                int angle = cmd_value.toInt();
                if (angle >= 0 && angle <= 180) {
                    setStopperAngle(angle);
                    Serial.print("Stopper angle set to ");
                    Serial.print(stopper_angle);
                    Serial.println(" degrees");
                } else {
                    Serial.println("Error: Stopper angle must be between 0 and 180");
                }
            }
            else {
                Serial.println("Error: Use S 0, S 1, or S <angle>");
            }
        }
        else if (cmd_type_upper == "F" || cmd_type_upper == "FLIP") {
            if (cmd_value_upper == "CW" || cmd_value_upper == "RIGHT" || cmd_value_upper == "R") {
                setFlipperClockwise();
                Serial.println("Flipper set to CLOCKWISE side");
            }
            else if (cmd_value_upper == "CCW" || cmd_value_upper == "LEFT" || cmd_value_upper == "L") {
                setFlipperCounterClockwise();
                Serial.println("Flipper set to COUNTER-CLOCKWISE side");
            }
            else if (cmd_value_upper == "CENTER" || cmd_value_upper == "C") {
                centerFlipper();
                Serial.println("Flipper CENTERED");
            }
            else if (isNumeric(cmd_value)) {
                int angle = cmd_value.toInt();
                if (angle >= 0 && angle <= 180) {
                    setFlipperAngle(angle);
                    Serial.print("Flipper angle set to ");
                    Serial.print(flipper_angle);
                    Serial.println(" degrees");
                } else {
                    Serial.println("Error: Flipper angle must be between 0 and 180");
                }
            }
            else {
                Serial.println("Error: Use F CW, F CCW, F CENTER, or F <angle>");
            }
        }
        else if (cmd_type_upper == "COLOR") {
            if (cmd_value_upper == "YELLOW" || cmd_value_upper == "BLUE") {
                flipForColor(cmd_value_upper);
                Serial.print("Flipper moved for color: ");
                Serial.println(cmd_value_upper);
            } else {
                Serial.println("Error: Use COLOR YELLOW or COLOR BLUE");
            }
        }
        else if (cmd_type_upper == "STATUS") {
            printStatus();
        }
        else if (cmd_type_upper == "HELP") {
            printHelp();
        }
        else {
            Serial.print("Unknown command: ");
            Serial.println(cmd_type);
            Serial.print("Type HELP for available commands\n");
        }
    }
}