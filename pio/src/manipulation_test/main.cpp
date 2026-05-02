/*
 * Manipulation Test Serial Commands
 * -----------------------------------------------
 * Pumps:
 *   P1 1, P2 1, P3 1, P4 1       -> turn an individual pump ON
 *   P1 0, P13 0, P1234 1         -> turn listed pumps OFF/ON
 *
 * Test / Homing:
 *   T                            -> coordinated move: set target to move,
 *   H                            -> return both motors to 0 (homing)
 *   w                            -> stop all motor movement (free-wheel)
 *   E                            -> print encoder count for debugging
 *
 * Servo Control:
 *  S 0, S 1                      -> Close or Open stopper
 *  S <angle>                     -> Set stopper to specific angle (0-180)
 *  F CW, F CCW, F C              -> Set flipper to clockwise, counter-clockwise, or center positions
 *  F <angle>                     -> Set flipper to specific angle (0-180)
 *  C YELLOW, C BLUE              -> Flip based on detected color
 * 
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
constexpr int FLIP = 12; // Servo flipper for sorting blocks

// MOTORS SWAPPED: Turn motor now uses pins 15,2  Lift motor now uses pins 4,5
constexpr int MOTOR_TURN1 = 15;
constexpr int MOTOR_TURN2 = 2;
constexpr int MOTOR_LIFT1 = 4;
constexpr int MOTOR_LIFT2 = 5;

constexpr int ENCODER_LIFT_A = 21;
constexpr int ENCODER_LIFT_B = 19;
constexpr int ENCODER_LIFT_X = 18; // Home index pulse for lift
constexpr int ENCODER_TURN_A = 22; 
constexpr int ENCODER_TURN_B = 23; 
constexpr int ENCODER_TURN_X = 32; // Home index pulse for turn

constexpr int PWM_CHANNEL_LIFT1 = 0;
constexpr int PWM_CHANNEL_LIFT2 = 1;
constexpr int PWM_CHANNEL_TURN1 = 2;
constexpr int PWM_CHANNEL_TURN2 = 3;
constexpr int PWM_FREQ = 5000;
constexpr int PWM_RESOLUTION = 8;
// Note: TICKS_PER_TURN is defined in motor control parameters section (around line 200)

const int PUMP_PINS[4] = {Pump1, Pump2, Pump3, Pump4};

// ArmMotor armMotor(1.0f, 0.01f, 0.0f, MOTOR_LIFT1, MOTOR_LIFT2, ENCODER_LIFT_A, ENCODER_LIFT_B, Home);
ArmMotor armMotor(
    /*kp*/ 1.5f,  /*ki*/ 0.0f,  /*kd*/ 0.0f,
    /*in1*/ 4,    /*in2*/ 5,
    /*enc_a*/ 21, /*enc_b*/ 19,  /*enc_x*/ 18,
    /*pwm_ch_fwd*/ 4, /*pwm_ch_rev*/ 5
);

bool     h_sequence_pending = false;
uint32_t h_sequence_timer   = 0;

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
String flipper_state = "CENTER";
String last_block_color = "UNKNOWN";

constexpr int STOP_CLOSED_ANGLE = 15;
constexpr int STOP_OPEN_ANGLE = 95;
constexpr int FLIP_CENTER_ANGLE = 90;
constexpr int FLIP_CW_ANGLE = 35;
constexpr int FLIP_CCW_ANGLE = 145;

// ----------------------------------------
// SECTION: Servo serial control
// ----------------------------------------

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

    return true;
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

    if (color == "YELLOW") {
        setFlipperClockwise();
    } else if (color == "BLUE") {
        setFlipperCounterClockwise();
    }
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
}

// ----------------------------------------
// SECTION: Main loop
// ----------------------------------------

void loop(){
     
    static uint32_t last = micros();
    uint32_t now = micros();
    const uint32_t interval = 2000;  // 2 ms for 500 Hz (1/500 * 1e6 microseconds)

    if (h_sequence_pending && (millis() - h_sequence_timer >= 1000)) {
        h_sequence_pending = false;
        armMotor.setTarget(0);
        Serial.println("Motor → home (0 ticks)");
    }

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
            is_compact_motor_cmd = (operation == 't' || operation == 'h' || operation == 'w' || operation == 'e');
        }

        if (is_compact_motor_cmd) {
            switch (operation) {
                case 't':                           // T → go to target
                    armMotor.setTarget(400);        // change 150 to whatever tick count you want
                    Serial.println("Motor → target (400 ticks)");
                    break;
                case 'h':
                    armMotor.setTarget(50);
                    h_sequence_pending = true;
                    h_sequence_timer   = millis();
                    Serial.println("Motor → 50 ticks, returning to 0 in 1 second");
                    break;
                case 'w':
                    ledcWrite(PWM_CHANNEL_LIFT1, 0);
                    ledcWrite(PWM_CHANNEL_LIFT2, 0);
                    ledcWrite(PWM_CHANNEL_TURN1, 0);
                    ledcWrite(PWM_CHANNEL_TURN2, 0);
                    Serial.println("Both motors RELEASED (free-wheel)");
                    break;
                case 'e': 
                    Serial.print("Encoder count: ");
                    Serial.println(armMotor.getEncoderCount());
                    ledcWrite(PWM_CHANNEL_LIFT1, 1);
                    ledcWrite(PWM_CHANNEL_LIFT2, 1);
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
        else if (cmd_type_upper == "S" ) {
            if (cmd_value_upper == "0" ) {
                setStopperPosition(false);
                Serial.println("Stopper CLOSED");
            }
            else if (cmd_value_upper == "1" ) {
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

        }
        else if (cmd_type_upper == "F") {
            if (cmd_value_upper == "CW") {
                setFlipperClockwise();
                Serial.println("Flipper set to CLOCKWISE side");
            }
            else if (cmd_value_upper == "CCW") {
                setFlipperCounterClockwise();
                Serial.println("Flipper set to COUNTER-CLOCKWISE side");
            }
            else if (cmd_value_upper == "C") {
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
        else if (cmd_type_upper == "C") {
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
        }
    }
}