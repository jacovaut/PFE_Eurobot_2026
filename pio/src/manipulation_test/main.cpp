/*
 * Manipulation Test Serial Commands - SIMPLIFIED
 * -----------------------------------------------
 * Pumps:
 *   P1 1, P2 1, P3 1, P4 1       -> turn an individual pump ON
 *   P1 0, P13 0, P1234 1         -> turn listed pumps OFF/ON
 *
 * Lift Motor (by number of turns, auto-stops):
 *   f <turns>                    -> lift forward N turns (1050 ticks each)
 *   b <turns>                    -> lift backward N turns
 *   s                            -> lift brake
 *
 * Turn Motor (by number of turns, auto-stops):
 *   t <turns>                    -> turn forward N turns (1050 ticks each)
 *   d <turns>                    -> turn backward N turns
 *   x                            -> turn brake
 *
 * Encoders (grouped):
 *   c                            -> read both encoders
 *   r                            -> reset both encoders
 *
 * [COMMENTED OUT] Servos / sorting:
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

Servo stopper_servo;
Servo flipper_servo;

ESP32Encoder lift_encoder;
ESP32Encoder turn_encoder;

// ----------------------------------------
// SECTION: Runtime state variables
// ----------------------------------------

// Current state tracking
bool pump_states[4] = {false, false, false, false};
bool stopper_open = false;
int stopper_angle = 15;
int flipper_angle = 90;
int lift_speed = 0;
int turn_speed = 0;
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

// Test mode flag
bool m_command_turn_triggered = false;

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

void startTurnTickMove(int64_t ticks);

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

    return true;
}

// ----------------------------------------
// SECTION: Motor brake functions
// ----------------------------------------

void LmotorBrake() {
    lift_speed = 0;
    lift_state = "BRAKE";
    ledcWrite(PWM_CHANNEL_LIFT1, 255);
    ledcWrite(PWM_CHANNEL_LIFT2, 255);
    Serial.println("Lift brake");
}

void TmotorBrake() {
    turn_speed = 0;
    turn_state = "BRAKE";
    ledcWrite(PWM_CHANNEL_TURN1, 255);
    ledcWrite(PWM_CHANNEL_TURN2, 255);
    Serial.println("Turn brake");
}

// ----------------------------------------
// SECTION: Motor control parameters & functions
// ----------------------------------------

// === PARAMETERS: Modify these values to tune motor behavior ===
constexpr int64_t TICKS_PER_TURN = 450;           // encoder ticks per full rotation
constexpr int LIFT_FORWARD_SPEED = 255;           // PWM value for lift forward (0-255)
constexpr int LIFT_BACKWARD_SPEED = 255;          // PWM value for lift backward
constexpr int TURN_FORWARD_SPEED = 255;           // PWM value for turn forward
constexpr int TURN_BACKWARD_SPEED = 255;          // PWM value for turn backward

constexpr int LIFT_HOME_SPEED = 180;              // PWM value for homing (3/4 speed)
constexpr int TURN_HOME_SPEED = 180;              // PWM value for homing (1/2 speed)

constexpr int LIFT_TEST_SPEED = 220;              // PWM value for m command test
constexpr int TURN_TEST_SPEED = 200;              // PWM value when triggered during m command
constexpr int TEST_LIFT_TURNS = 0;                // turns for m command (set 0 to disable)
constexpr int TEST_TURN_TRIGGER_TICKS = 200;     // when to trigger turn motor during m command

// ========================================
void startLiftTickMove(int64_t ticks) {
    if (ticks == 0) {
        lift_test_active = false;
        LmotorBrake();
        return;
    }

    lift_test_active = true;
    lift_test_start_count = lift_encoder.getCount();
    lift_test_target_ticks = abs(ticks);
    lift_test_direction = (ticks > 0) ? "UP" : "DOWN";

    int pwm_speed = (ticks > 0) ? LIFT_FORWARD_SPEED : LIFT_BACKWARD_SPEED;
    int pwm_pin1 = (ticks > 0) ? PWM_CHANNEL_LIFT1 : PWM_CHANNEL_LIFT2;
    int pwm_pin2 = (ticks > 0) ? PWM_CHANNEL_LIFT2 : PWM_CHANNEL_LIFT1;
    
    lift_speed = pwm_speed;
    lift_state = (ticks > 0) ? "FORWARD" : "BACKWARD";
    ledcWrite(pwm_pin1, pwm_speed);
    ledcWrite(pwm_pin2, 0);

    Serial.printf("Lift %s: %lld ticks (%lld turns) at PWM %d\n", 
                  lift_test_direction.c_str(), lift_test_target_ticks, 
                  lift_test_target_ticks / TICKS_PER_TURN, pwm_speed);
}

void updateLiftTickMove() {
    if (!lift_test_active) {
        return;
    }

    int64_t current_count = lift_encoder.getCount();
    int64_t delta = abs(current_count - lift_test_start_count);

    if (millis() - lift_debug_timer > 100) {
        Serial.printf("Lift: %lld/%lld ticks\n", delta, lift_test_target_ticks);
        lift_debug_timer = millis();
    }

    // m command: trigger turn motor at specified tick count
    if (TEST_LIFT_TURNS > 0 && !m_command_turn_triggered && delta >= TEST_TURN_TRIGGER_TICKS) {
        m_command_turn_triggered = true;
        startTurnTickMove(1);  // 1 turn forward
    }

    if (delta >= lift_test_target_ticks) {
        lift_test_active = false;
        LmotorBrake();
        Serial.printf("Lift complete: %lld ticks\n", delta);
        lift_test_target_ticks = 0;
    }
}

void startTurnTickMove(int64_t ticks) {
    if (ticks == 0) {
        turn_test_active = false;
        TmotorBrake();
        return;
    }

    turn_test_active = true;
    turn_test_start_count = turn_encoder.getCount();
    turn_test_target_ticks = abs(ticks);
    turn_test_direction = (ticks > 0) ? "CW" : "CCW";

    int pwm_speed = (ticks > 0) ? TURN_FORWARD_SPEED : TURN_BACKWARD_SPEED;
    int pwm_pin1 = (ticks > 0) ? PWM_CHANNEL_TURN1 : PWM_CHANNEL_TURN2;
    int pwm_pin2 = (ticks > 0) ? PWM_CHANNEL_TURN2 : PWM_CHANNEL_TURN1;
    
    turn_speed = pwm_speed;
    turn_state = (ticks > 0) ? "FORWARD" : "BACKWARD";
    ledcWrite(pwm_pin1, pwm_speed);
    ledcWrite(pwm_pin2, 0);

    Serial.printf("Turn %s: %lld ticks (%lld turns) at PWM %d\n", 
                  turn_test_direction.c_str(), turn_test_target_ticks, 
                  turn_test_target_ticks / TICKS_PER_TURN, pwm_speed);
}

void updateTurnTickMove() {
    if (!turn_test_active) {
        return;
    }

    int64_t current_count = turn_encoder.getCount();
    int64_t delta = abs(current_count - turn_test_start_count);

    if (millis() - turn_debug_timer > 100) {
        Serial.printf("Turn: %lld/%lld ticks\n", delta, turn_test_target_ticks);
        turn_debug_timer = millis();
    }

    if (delta >= turn_test_target_ticks) {
        turn_test_active = false;
        TmotorBrake();
        Serial.printf("Turn complete: %lld ticks\n", delta);
        turn_test_target_ticks = 0;
    }
}

// ----------------------------------------
// SECTION: Test and homing commands
// ----------------------------------------

void testLiftHalfTurn() {
    Serial.println("=== Test m: Lift half turn with delayed turn trigger ===");
    m_command_turn_triggered = false;
    
    lift_test_active = true;
    lift_test_start_count = lift_encoder.getCount();
    lift_test_target_ticks = (TICKS_PER_TURN / 2);
    lift_test_direction = "TEST";
    
    lift_speed = LIFT_TEST_SPEED;
    lift_state = "TEST_FORWARD";
    ledcWrite(PWM_CHANNEL_LIFT2, 0);
    ledcWrite(PWM_CHANNEL_LIFT1, LIFT_TEST_SPEED);
    
    Serial.printf("Lift test: 0.5 turns (%lld ticks) at PWM %d\n", 
                  lift_test_target_ticks, LIFT_TEST_SPEED);
}

void returnToZero() {
    Serial.println("=== Home command: Return both motors to 0 ===");
    int64_t lift_current = lift_encoder.getCount();
    int64_t turn_current = turn_encoder.getCount();
    
    lift_test_active = (lift_current != 0);
    turn_test_active = (turn_current != 0);
    
    if (lift_test_active) {
        lift_test_start_count = lift_current;
        lift_test_target_ticks = abs(lift_current);
        lift_test_direction = (lift_current > 0) ? "DOWN" : "UP";
        
        int pwm_dir1 = (lift_current > 0) ? PWM_CHANNEL_LIFT2 : PWM_CHANNEL_LIFT1;
        
        lift_speed = LIFT_HOME_SPEED;
        lift_state = "HOMING";
        ledcWrite(pwm_dir1, LIFT_HOME_SPEED);
        ledcWrite((pwm_dir1 == PWM_CHANNEL_LIFT1) ? PWM_CHANNEL_LIFT2 : PWM_CHANNEL_LIFT1, 0);
        
        Serial.printf("Lift homing: %lld ticks back to 0\n", lift_test_target_ticks);
    }
    
    if (turn_test_active) {
        turn_test_start_count = turn_current;
        turn_test_target_ticks = abs(turn_current);
        turn_test_direction = (turn_current > 0) ? "CCW" : "CW";
        
        int pwm_dir1 = (turn_current > 0) ? PWM_CHANNEL_TURN2 : PWM_CHANNEL_TURN1;
        
        turn_speed = TURN_HOME_SPEED;
        turn_state = "HOMING";
        ledcWrite(pwm_dir1, TURN_HOME_SPEED);
        ledcWrite((pwm_dir1 == PWM_CHANNEL_TURN1) ? PWM_CHANNEL_TURN2 : PWM_CHANNEL_TURN1, 0);
        
        Serial.printf("Turn homing: %lld ticks back to 0\n", turn_test_target_ticks);
    }
    
    return_to_zero_active = (lift_test_active || turn_test_active);
    if (return_to_zero_active) {
        return_to_zero_start_time = millis();
    } else {
        Serial.println("Already at home position\n");
    }
}


void checkReturnToZeroTimeout() {
    if (!return_to_zero_active) {
        return;
    }

    if (!lift_test_active && !turn_test_active) {
        return_to_zero_active = false;
        return;
    }

    if (millis() - return_to_zero_start_time <= RETURN_TO_ZERO_TIMEOUT_MS) {
        return;
    }

    Serial.println("Return-to-home timeout exceeded (>2s), resetting encoders to 0");
    lift_encoder.setCount(0);
    turn_encoder.setCount(0);
    lift_test_active = false;
    turn_test_active = false;
    lift_test_direction = "IDLE";
    turn_test_direction = "IDLE";
    lift_test_target_ticks = 0;
    turn_test_target_ticks = 0;
    return_to_zero_active = false;
    LmotorBrake();
    TmotorBrake();
}

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
    Serial.print("\n=== Commands (SIMPLIFIED) ===\n");
    Serial.print("Pumps:\n");
    Serial.print("  P<nums> <0|1>   - Control pumps (Examples: P1 1, P2 1, P13 0, P1234 1)\n");
    Serial.print("\nLift Motor (by turns, auto-stops at target):\n");
    Serial.print("  f <turns>       - Lift forward N turns (Examples: f 1, f 2, f 3)\n");
    Serial.print("  b <turns>       - Lift backward N turns\n");
    Serial.print("  s               - Lift brake\n");
    Serial.print("\nTurn Motor (by turns, auto-stops at target):\n");
    Serial.print("  t <turns>       - Turn forward N turns (Examples: t 1, t 2)\n");
    Serial.print("  d <turns>       - Turn backward N turns\n");
    Serial.print("  x               - Turn brake\n");
    Serial.print("\nEncoders (grouped):\n");
    Serial.print("  c               - Read both encoder counts\n");
    Serial.print("  r               - Reset both encoders to 0\n");
    Serial.print("\nTest:\n");
    Serial.print("  m               - Test lift: 0.5 turns forward at 3/4 speed\n");
    Serial.print("  n               - Return both motors to zero position\n");
    Serial.print("\nOther:\n");
    Serial.print("  STATUS          - Show current status\n");
    Serial.print("  HELP            - Show this help message\n");
    Serial.print("[SERVOS/SORTING COMMENTED OUT]\n");
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

// ----------------------------------------
// SECTION: Arduino main loop
// ----------------------------------------

void loop(){
    checkReturnToZeroTimeout();
    updateLiftTickMove();
    updateTurnTickMove();

    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command.length() == 0) {
            return;
        }

        char operation = tolower(command.charAt(0));
        bool is_compact_motor_cmd = false;

        if (command.length() == 1) {
            is_compact_motor_cmd = (operation == 'x' || operation == 's' || operation == 'c' || operation == 'r' || operation == 'm' || operation == 'n');
        }

        if (is_compact_motor_cmd) {
            switch (operation) {
                case 's':
                    LmotorBrake();
                    lift_test_active = false;
                    Serial.println("Lift BRAKE (manual stop)");
                    break;
                case 'x':
                    TmotorBrake();
                    turn_test_active = false;
                    Serial.println("Turn BRAKE (manual stop)");
                    break;
                case 'c':
                    Serial.printf("Encoders - Lift: %lld, Turn: %lld\n", lift_encoder.getCount(), turn_encoder.getCount());
                    break;
                case 'r':
                    lift_encoder.setCount(0);
                    turn_encoder.setCount(0);
                    Serial.println("Both encoders reset to 0");
                    break;
                case 'm':
                    testLiftHalfTurn();
                    break;
                case 'n':
                    returnToZero();
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
        // Handle motor commands with turns: f <turns>, b <turns>, t <turns>, d <turns>
        else if ((cmd_type_upper == "F" || cmd_type_upper == "B" || cmd_type_upper == "T" || cmd_type_upper == "D") && cmd_value.length() > 0) {
            if (!isNumeric(cmd_value)) {
                Serial.println("Error: Motor commands require a number of turns (f 1, b 2, t 1, d 1, etc)");
                return;
            }
            
            int turns = cmd_value.toInt();
            if (turns == 0) {
                Serial.println("Error: Number of turns must be positive");
                return;
            }
            
            int64_t ticks = (int64_t)turns * TICKS_PER_TURN;
            
            if (cmd_type_upper == "F") {
                startLiftTickMove(ticks);
            } else if (cmd_type_upper == "B") {
                startLiftTickMove(-ticks);
            } else if (cmd_type_upper == "T") {
                startTurnTickMove(ticks);
            } else if (cmd_type_upper == "D") {
                startTurnTickMove(-ticks);
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