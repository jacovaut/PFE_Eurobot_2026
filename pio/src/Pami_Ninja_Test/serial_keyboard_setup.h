#ifndef PAMI_NINJA_SERIAL_SETUP_H
#define PAMI_NINJA_SERIAL_SETUP_H

#include <Arduino.h>

TaskHandle_t core1_handle = NULL;

bool valveState = false;
bool pumpState = false;
bool encoderReadingEnabled = false;

int currentServoAngle = 90;
bool servoSweepEnabled = false;
int servoSweepDirection = 1;
unsigned long lastServoSweepStepTime = 0;
const int SERVO_INCREMENT = 10;

int currentMaxSpeed = 100;
int currentOmegaSpeed = 90;
const int MIN_SPEED = 50;
const int MAX_SPEED_LIMIT = MAX_SPEED;
const int SPEED_INCREMENT = 5;
const unsigned long SPEED_CHANGE_DEBOUNCE = 200;

bool keyPressed[256] = {false};
unsigned long keyLastReceived[256] = {0};
const unsigned long KEY_TIMEOUT = 150;

void error_loop();
void core1(void* pvParameters);
void printKeyboardHelp();
void processSerialKeyboardInput();
bool serialMovementActive();
void applySerialMovementControl();
void applySerialActionControl();

void setMecanumSpeeds(float vx, float vy, float omega);
void setMotor(int motor, int speed);
void setServo(int servo, int angle);
void setPump(bool state);
void setValve(bool state);
void resetEncoders();
void pickupBlock();
void releaseBlock();
void toggleServoSweep();
void updateServoSweep();
void togglePump();
void toggleEncoderReading();
void setSpeed(int speed);
void setOmegaSpeed(int speed);

#endif
