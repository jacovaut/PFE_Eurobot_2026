#include <Arduino.h>
#include <stdint.h>

// ---- FROM PCB MANIPULATION ---- //

constexpr int pump = 32; // Control the Pump
constexpr int Valve1 = 25; // Control the Valve1
constexpr int Valve2 = 26; // Control the Valve2
constexpr int Valve3 = 27; // Control the Valve3
constexpr int Valve4 = 14; // Control the Valve4
constexpr int Valve_out = 12; // Control the Valve_out
constexpr int Thermo = 33; // Control Thermo?
constexpr int STOP = 13; // Connected to Stopper?

// Current state tracking
uint8_t pump_speed = 0;
bool pump_on = false;
uint32_t pump_start_time = 0;
bool valve1_state = false;
bool valve2_state = false;
bool valve3_state = false;
bool valve4_state = false;
bool valve_out_state = false;

// Soft start parameters
constexpr uint32_t SOFT_START_DURATION = 2000; // 2 seconds to reach full speed
constexpr uint8_t MAX_PUMP_SPEED = 255;

void printStatus() {
    Serial.print("\n=== Current Status ===\n");
    Serial.print("Pump: ");
    Serial.print(pump_speed);
    Serial.print("/255\n");
    Serial.print("Valve1: ");
    Serial.println(valve1_state ? "ON" : "OFF");
    Serial.print("Valve2: ");
    Serial.println(valve2_state ? "ON" : "OFF");
    Serial.print("Valve3: ");
    Serial.println(valve3_state ? "ON" : "OFF");
    Serial.print("Valve4: ");
    Serial.println(valve4_state ? "ON" : "OFF");
    Serial.print("Valve_out: ");
    Serial.println(valve_out_state ? "ON" : "OFF");
    Serial.print("=====================\n\n");
}

void printHelp() {
    Serial.print("\n=== Commands ===\n");
    Serial.print("P <0|1>          - Pump OFF (0) or ON with soft start (1)\n");
    Serial.print("V<nums> <0|1>    - Set valves (e.g., V12 1 = open V1+V2)\n");
    Serial.print("                   V1, V2, V3, V4, V123, V1234, etc.\n");
    Serial.print("STATUS           - Show current status\n");
    Serial.print("HELP             - Show this help message\n");
    Serial.print("=================\n\n");
}

void setup(){
    Serial.begin(115200);
    delay(1000); // Give serial time to initialize
    
    Serial.print("\n\nStarting Manipulation Test...\n");

    pinMode(Valve1, OUTPUT);
    pinMode(Valve2, OUTPUT);
    pinMode(Valve3, OUTPUT);
    pinMode(Valve4, OUTPUT);
    pinMode(Valve_out, OUTPUT);
    pinMode(pump, OUTPUT);
    pinMode(STOP, OUTPUT);

    // Initialize all outputs to LOW
    digitalWrite(pump, LOW);
    digitalWrite(Valve1, LOW);
    digitalWrite(Valve2, LOW);
    digitalWrite(Valve3, LOW);
    digitalWrite(Valve4, LOW);
    digitalWrite(Valve_out, LOW);
    digitalWrite(STOP, LOW);

    printHelp();
    printStatus();
}

void loop(){
    // Handle pump soft start
    if (pump_on) {
        uint32_t elapsed = millis() - pump_start_time;
        if (elapsed < SOFT_START_DURATION) {
            // Linear ramp from 0 to MAX_PUMP_SPEED
            pump_speed = (elapsed * MAX_PUMP_SPEED) / SOFT_START_DURATION;
        } else {
            pump_speed = MAX_PUMP_SPEED;
        }
        analogWrite(pump, pump_speed);
    }
    
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toUpperCase();

        // Parse command
        int space_idx = command.indexOf(' ');
        String cmd_type = (space_idx > 0) ? command.substring(0, space_idx) : command;
        String cmd_value = (space_idx > 0) ? command.substring(space_idx + 1) : "";

        if (cmd_type == "P") {
            // Set pump on/off
            if (cmd_value.length() > 0) {
                int state = cmd_value.toInt();
                if (state == 0) {
                    pump_on = false;
                    pump_speed = 0;
                    analogWrite(pump, 0);
                    Serial.print("Pump OFF\n");
                } else if (state == 1) {
                    pump_on = true;
                    pump_start_time = millis();
                    Serial.print("Pump ON - Soft starting...\n");
                } else {
                    Serial.print("Error: Pump only accepts 0 (OFF) or 1 (ON)\n");
                }
            } else {
                Serial.print("Error: P requires a value (0 or 1)\n");
            }
        }
        else if (cmd_type.startsWith("V")) {
            // Handle valve commands (V1, V12, V1234, etc.)
            if (cmd_value.length() > 0) {
                int state = cmd_value.toInt();
                String valve_nums = cmd_type.substring(1); // Remove 'V'
                bool valid = true;
                
                // Track which valves are requested
                bool requested[5] = {false}; // indices 1-4 used
                
                // Parse and validate valve numbers
                for (int i = 0; i < valve_nums.length(); i++) {
                    char digit = valve_nums[i];
                    if (digit >= '1' && digit <= '4') {
                        requested[digit - '0'] = true;
                    } else {
                        valid = false;
                        break;
                    }
                }
                
                if (valid) {
                    // Close valves not in this command
                    if (!requested[1]) {
                        valve1_state = false;
                        digitalWrite(Valve1, LOW);
                    }
                    if (!requested[2]) {
                        valve2_state = false;
                        digitalWrite(Valve2, LOW);
                    }
                    if (!requested[3]) {
                        valve3_state = false;
                        digitalWrite(Valve3, LOW);
                    }
                    if (!requested[4]) {
                        valve4_state = false;
                        digitalWrite(Valve4, LOW);
                    }
                    
                    // Set requested valves to the state
                    for (int i = 1; i <= 4; i++) {
                        if (requested[i]) {
                            if (i == 1) {
                                valve1_state = (state != 0);
                                digitalWrite(Valve1, valve1_state ? HIGH : LOW);
                            } else if (i == 2) {
                                valve2_state = (state != 0);
                                digitalWrite(Valve2, valve2_state ? HIGH : LOW);
                            } else if (i == 3) {
                                valve3_state = (state != 0);
                                digitalWrite(Valve3, valve3_state ? HIGH : LOW);
                            } else if (i == 4) {
                                valve4_state = (state != 0);
                                digitalWrite(Valve4, valve4_state ? HIGH : LOW);
                            }
                        }
                    }
                    
                    Serial.print("Valves ");
                    Serial.print(valve_nums);
                    Serial.print(" set to ");
                    Serial.println(state != 0 ? "ON" : "OFF");
                } else {
                    Serial.print("Error: Invalid valve number. Use 1, 2, 3, 4, or combinations like V12, V1234\n");
                }
            } else {
                Serial.print("Error: Valve command requires a value (0 or 1)\n");
            }
        }
        else if (cmd_type == "STATUS") {
            printStatus();
        }
        else if (cmd_type == "HELP") {
            printHelp();
        }
        else {
            Serial.print("Unknown command: ");
            Serial.println(cmd_type);
            Serial.print("Type HELP for available commands\n");
        }
    }
}