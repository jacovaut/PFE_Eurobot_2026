#ifndef PAMI_NINJA_MICROROS_SETUP_H
#define PAMI_NINJA_MICROROS_SETUP_H

#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <ctype.h>
#include <string.h>

extern bool hasRun;
extern volatile bool startRequested;
extern volatile bool stopRequested;
extern volatile uint8_t selectedAutonomousRun;

char MICROROS_WIFI_SSID[] = "GRUM";
char MICROROS_WIFI_PASSWORD[] = "GELE>GMEC";
IPAddress MICROROS_AGENT_IP(192, 168, 1, 131);
const uint16_t MICROROS_AGENT_PORT = 8895;

rcl_subscription_t runningSub;
rcl_subscription_t teamColorSub;
std_msgs__msg__Bool runningMsg;
std_msgs__msg__String teamColorMsg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
char teamColorBuffer[16];

void errorLoop() {
  pinMode(2, OUTPUT);
  while (true) {
    digitalWrite(2, !digitalRead(2));
    delay(100);
  }
}

#define RCCHECK(fn) { \
  rcl_ret_t rc = fn; \
  if (rc != RCL_RET_OK) { \
    errorLoop(); \
  } \
}

#define RCSOFTCHECK(fn) { \
  rcl_ret_t rc = fn; \
  if (rc != RCL_RET_OK) { \
  } \
}

void runningCallback(const void* msgin) {
  const std_msgs__msg__Bool* msg =
    static_cast<const std_msgs__msg__Bool*>(msgin);

  if (msg->data) {
    startRequested = true;
    stopRequested = false;
  } else {
    stopRequested = true;
    startRequested = false;
    hasRun = false;
  }
}

void teamColorCallback(const void* msgin) {
  const std_msgs__msg__String* msg =
    static_cast<const std_msgs__msg__String*>(msgin);

  char color[sizeof(teamColorBuffer)];
  const size_t length = min(msg->data.size, sizeof(color) - 1);
  memcpy(color, msg->data.data, length);
  color[length] = '\0';

  for (size_t i = 0; i < length; i++) {
    color[i] = static_cast<char>(tolower(color[i]));
  }

  if (strcmp(color, "yellow") == 0 || strcmp(color, "jaune") == 0 ||
      strcmp(color, "1") == 0 || strcmp(color, "run1") == 0) {
    selectedAutonomousRun = 1;
    hasRun = false;
    Serial.println("team_color selected yellow run");
  } else if (strcmp(color, "blue") == 0 || strcmp(color, "bleu") == 0 ||
             strcmp(color, "2") == 0 || strcmp(color, "run2") == 0) {
    selectedAutonomousRun = 2;
    hasRun = false;
    Serial.println("team_color selected blue run");
  } else {
    Serial.printf("Ignoring unknown team_color: %s\n", color);
  }
}

void setupMicroRosTrigger() {
  allocator = rcl_get_default_allocator();

  set_microros_wifi_transports(
    MICROROS_WIFI_SSID,
    MICROROS_WIFI_PASSWORD,
    MICROROS_AGENT_IP,
    MICROROS_AGENT_PORT
  );

  Serial.printf(
    "micro-ROS trigger mode: ssid=%s agent=%s:%u\n",
    MICROROS_WIFI_SSID,
    MICROROS_AGENT_IP.toString().c_str(),
    MICROROS_AGENT_PORT
  );
  Serial.println("Waiting for micro-ROS agent...");
  while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
    delay(1000);
  }
  Serial.println("micro-ROS agent connected");

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "pami_ninja_script_trigger", "", &support));
  RCCHECK(rclc_subscription_init_default(
    &runningSub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "match/running"
  ));
  RCCHECK(rclc_subscription_init_default(
    &teamColorSub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/team_color"
  ));

  teamColorMsg.data.data = teamColorBuffer;
  teamColorMsg.data.size = 0;
  teamColorMsg.data.capacity = sizeof(teamColorBuffer);

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &runningSub,
    &runningMsg,
    &runningCallback,
    ON_NEW_DATA
  ));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &teamColorSub,
    &teamColorMsg,
    &teamColorCallback,
    ON_NEW_DATA
  ));

  Serial.println("Listening to match/running (std_msgs/Bool).");
  Serial.println("Listening to /team_color (std_msgs/String): yellow=run 1, blue=run 2.");
  Serial.println("true starts the script. false resets the trigger.");
}

#endif
