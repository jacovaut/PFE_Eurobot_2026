/*
 * Manipulation Test Serial Commands - SIMPLIFIED
 * -----------------------------------------------
 * Pumps:
 *   P1 1, P2 1, P3 1, P4 1       -> turn an individual pump ON
 *   P1 0, P13 0, P1234 1         -> turn listed pumps OFF/ON
 *
 * Lift Motor (direct ticks, auto-stops):
 *   L <ticks>                    -> lift forward/backward (positive=up, negative=down, PWM 255)
 *                                   ex: L 450, L -900
 *   s                            -> lift brake
 *
 * Turn Motor (direct ticks, auto-stops):
 *   T <ticks>                    -> turn CW/CCW (positive=CW, negative=CCW, PWM 255)
 *                                   ex: T 450, T -450
 *   x                            -> turn brake
 *

 *
 * Encoders (grouped):
 *   c                            -> read both encoder counts
 *   r                            -> reset both encoders to 0
 *
 * Test / Homing:
 *   m                            -> coordinated move: lift to 880 ticks (PWM 255),
 *                                   turn starts at 150 ticks (target 450 ticks, PWM 255)
 *   n                            -> return both motors to 0 (homing)
 *
 * Other:
 *   STATUS                       -> show all current states
 *   HELP                         -> show help in serial monitor
 */
#include <Arduino.h>
#include <stdint.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include "ArmMotor.h"

// ----------------------------------------
// SECTION: Hardware pins and configuration
// ----------------------------------------

// ---- FROM PCB MANIPULATION ---- //

constexpr int Pump1 = 25; // Pump for suction cup 1
constexpr int Pump2 = 26; // Pump for suction cup 2
constexpr int Pump3 = 27; // Pump for suction cup 3
constexpr int Pump4 = 14; // Pump for suction cup 4
constexpr int Thermo = 33; // Control Thermo?
constexpr int STOP = 13; // Servo stopper for the block chute
constexpr int FLIP = 18; // Servo flipper for sorting blocks

// MOTORS SWAPPED: Turn motor now uses pins 15,2  Lift motor now uses pins 4,5
constexpr int MOTOR_TURN1 = 15;
constexpr int MOTOR_TURN2 = 2;
constexpr int MOTOR_LIFT1 = 4;
constexpr int MOTOR_LIFT2 = 5;

constexpr int ENCODER_LIFT_A = 21;
constexpr int ENCODER_LIFT_B = 19;
constexpr int ENCODER_TURN_A = 22;  // Swapped from 23
constexpr int ENCODER_TURN_B = 23;  // Swapped from 22

constexpr int PWM_CHANNEL_LIFT1 = 0;
constexpr int PWM_CHANNEL_LIFT2 = 1;
constexpr int PWM_CHANNEL_TURN1 = 2;
constexpr int PWM_CHANNEL_TURN2 = 3;
constexpr int PWM_FREQ = 5000;
constexpr int PWM_RESOLUTION = 8;
// Note: TICKS_PER_TURN is defined in motor control parameters section (around line 200)

const int PUMP_PINS[4] = {Pump1, Pump2, Pump3, Pump4};

// ArmMotor armMotor(1.0f, 0.01f, 0.0f, MOTOR_LIFT1, MOTOR_LIFT2, ENCODER_LIFT_A, ENCODER_LIFT_B, 18);
ArmMotor armMotor(1.0f, 0.01f, 0.0f, 5, 4, 21, 19, 18);


Servo stopper_servo;
Servo flipper_servo;

// ----------------------------------------
// SECTION: Runtime state variables
// ----------------------------------------

// Current state tracking
bool pump_states[4] = {false, false, false, false};
bool stopper_open = false;
int stopper_angle = 15;
int flipper_angle = 90;
bool lift_test_active = false;
int64_t lift_test_start_count = 0;
int64_t lift_test_target_ticks = 0;
String lift_test_direction = "IDLE";
bool turn_test_active = false;
int64_t turn_test_start_count = 0;
int64_t turn_test_target_ticks = 0;
String turn_test_direction = "IDLE";
String lift_state = "BRAKE";
String turn_state = "BRAKE";
String flipper_state = "CENTER";
String last_block_color = "UNKNOWN";

// Debug timers
unsigned long lift_debug_timer = 0;
unsigned long turn_debug_timer = 0;

// Homing timeout support
bool return_to_zero_active = false;
unsigned long return_to_zero_start_time = 0;
const unsigned long RETURN_TO_ZERO_TIMEOUT_MS = 2000;

// Test mode flags
bool m_command_active = false;
bool m_command_turn_triggered = false;
bool m_turn_retract_pending = false;
bool lift_reset_encoders_on_complete = false;
bool pump_encoder_reset_pending = false;
unsigned long pump_encoder_reset_timer = 0;

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

// ----------------------------------------
// SECTION: Pump state functions
// ----------------------------------------

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

    // When any pump is turned ON, retract lift by 5 ticks then reset encoders
    if (state) {
        bool any_on = false;
        for (int i = 0; i < 4; i++) { if (requested[i]) { any_on = true; break; } }
        if (any_on) {
            Serial.println("Pump ON: retracting lift -5 ticks then resetting encoders");
            lift_reset_encoders_on_complete = true;
            // armMotor.setTarget(-5); // Need relative target?
        }
    }

    return true;
}


// ========================================

// ----------------------------------------
// SECTION: Servo Control
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
    Serial.println(lift_state);
    Serial.print("Turn motor: ");
    Serial.println(turn_state);
    Serial.print("Turn test: ");
    if (turn_test_active) {
        Serial.print(turn_test_direction);
        Serial.print(" RUNNING (");
        Serial.print(turn_test_target_ticks);
        Serial.println(" ticks)");
    } else {
        Serial.println("IDLE");
    }
    
    /* SERVO STATUS - COMMENTED OUT */
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
    Serial.print("\n=== Commands (SIMPLIFIED) ===\n");
    Serial.print("Pumps:\n");
    Serial.print("  P<nums> <0|1>   - Control pumps (Examples: P1 1, P2 1, P13 0, P1234 1)\n");
    Serial.print("\nLift Motor (direct ticks, auto-stops at target):\n");
    Serial.print("  L <ticks>       - Move lift N ticks (positive = up, negative = down, PWM 255)\n");
    Serial.print("  s               - Lift brake\n");
    Serial.print("\nTurn Motor (direct ticks, auto-stops at target):\n");
    Serial.print("  T <ticks>       - Move turn N ticks (positive = CW, negative = CCW, PWM 255)\n");
    Serial.print("  x               - Turn brake\n");
    Serial.print("\nEncoders (grouped):\n");
    Serial.print("  c               - Read both encoder counts\n");
    Serial.print("  r               - Reset both encoders to 0\n");
    Serial.print("\nTest:\n");
    Serial.print("  m               - Lift to 880 ticks at 255; turn starts at 150 ticks (target 450 ticks) at 255\n");
    Serial.print("  n               - Return both motors to zero position\n");
    Serial.print("\nOther:\n");
    Serial.print("  STATUS          - Show current status\n");
    Serial.print("  HELP            - Show this help message\n");
    Serial.print("[SERVOS ACTIVE]\n");
    Serial.print("=============================\n\n");
}

void setup(){
    Serial.begin(115200);
    delay(1000); // Give serial time to initialize
    
    Serial.print("\n\nStarting Manipulation Test...\n");
    
    for (int i = 0; i < 4; i++) {
        pinMode(PUMP_PINS[i], OUTPUT);
        digitalWrite(PUMP_PINS[i], LOW);
    }

    pinMode(MOTOR_TURN1, OUTPUT);
    pinMode(MOTOR_TURN2, OUTPUT);
    digitalWrite(MOTOR_TURN1, LOW);
    digitalWrite(MOTOR_TURN2, LOW);

    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    stopper_servo.setPeriodHertz(50);
    flipper_servo.setPeriodHertz(50);
    stopper_servo.attach(STOP, 500, 2400);
    flipper_servo.attach(FLIP, 500, 2400);
    setStopperPosition(false);
    centerFlipper();
    
    armMotor.setup();
    
    printHelp();
    // printStatus();
    
    // while (true)
    // {
    //     delay(1000);
    //     Serial.print(".");
    // }
    
}

// ----------------------------------------
// SECTION: Arduino main loop
// ----------------------------------------

void loop(){
     
    static uint32_t last = micros();
    uint32_t now = micros();
    const uint32_t interval = 2000;  // 2 ms for 500 Hz (1/500 * 1e6 microseconds)

    if (now - last >= interval) {
        last += interval;
        armMotor.update();
    }
    // Optional: Add a small yield or other non-blocking code here if needed

    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command.length() == 0) {
            return;
        }

        char operation = tolower(command.charAt(0));
        bool is_compact_motor_cmd = false;

        if (command.length() == 1) {
            is_compact_motor_cmd = (operation == 'x' || operation == 's' || operation == 'c' || operation == 'r' || operation == 'm' || operation == 'n' || operation == 'w');
        }

        if (is_compact_motor_cmd) {
            switch (operation) {
                case 'c':
                    // Serial.printf("Encoders - Lift: %lld, Turn: %lld\n", lift_encoder.getCount(), turn_encoder.getCount());
                    break;
                case 'm':
                    armMotor.setTarget(150.0);
                    break;
                case 'n':
                    Serial.println("Home not available - encoder-based homing in progress");
                    break;
                case 'w':
                    lift_test_active = false;
                    turn_test_active = false;
                    lift_state = "FREE";
                    turn_state = "FREE";
                    ledcWrite(PWM_CHANNEL_LIFT1, 0);
                    ledcWrite(PWM_CHANNEL_LIFT2, 0);
                    ledcWrite(PWM_CHANNEL_TURN1, 0);
                    ledcWrite(PWM_CHANNEL_TURN2, 0);
                    Serial.println("Both motors RELEASED (free-wheel)");
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

        // Pump commands first (must check before motor commands)
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
        // New unified commands: L <ticks> for lift, T <ticks> for turn (signed, positive or negative)
        else if ((cmd_type_upper == "L" || cmd_type_upper == "T") && cmd_value.length() > 0) {
            int64_t ticks = cmd_value.toInt();
            if (cmd_type_upper == "L") {
                armMotor.setTarget(ticks);
            } else {
                armMotor.setTarget(ticks);
            }
        }
        else if (cmd_type_upper == "STATUS") {
            printStatus();
        }
        else if (cmd_type_upper == "HELP") {
            printHelp();
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
        else {
            Serial.print("Unknown command: ");
            Serial.println(cmd_type);
            Serial.print("Type HELP for available commands\n");
        }
    }
}