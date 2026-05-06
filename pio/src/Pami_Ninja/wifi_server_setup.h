#ifndef PAMI_NINJA_MICROROS_SETUP_H
#define PAMI_NINJA_MICROROS_SETUP_H

#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>

// micro-ROS WiFi configuration copied from entrainement/main.cpp.
char MICROROS_WIFI_SSID[] = "GRUM";
char MICROROS_WIFI_PASSWORD[] = "GELE>GMEC";
IPAddress MICROROS_AGENT_IP(192, 168, 1, 131);
const uint16_t MICROROS_AGENT_PORT = 8888;

TaskHandle_t core1_handle = NULL;
TaskHandle_t core2_handle = NULL;

rcl_subscription_t cmdvel_sub;
geometry_msgs__msg__Twist cmdvel_msg;
rcl_subscription_t keyboard_sub;
std_msgs__msg__String keyboard_msg;
char keyboard_msg_buffer[8];

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

struct CmdVel {
  float vx;
  float vy;
  float w;
  uint32_t seq;
};

volatile CmdVel cmdvel = {0.0f, 0.0f, 0.0f, 0};
volatile bool cmdvelReceived = false;
volatile unsigned long lastCmdVelTime = 0;
const unsigned long CMDVEL_TIMEOUT_MS = 500;

bool valveState = false;
bool pumpState = false;
bool encoderReadingEnabled = false;

int currentServoAngle = 90;
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

#define RCCHECK(fn) { \
  rcl_ret_t rc = fn; \
  if (rc != RCL_RET_OK) { \
    error_loop(); \
  } \
}

#define RCSOFTCHECK(fn) { \
  rcl_ret_t rc = fn; \
  if (rc != RCL_RET_OK) { \
  } \
}

void error_loop();
void cmdvel_callback(const void* msgin);
void keyboard_callback(const void* msgin);
void timer_callback(rcl_timer_t* timer, int64_t last_call_time);
void setupMicroRos();
void core1(void* pvParameters);
void core2(void* pvParameters);
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
void togglePump();
void toggleEncoderReading();
void setSpeed(int speed);

#endif
