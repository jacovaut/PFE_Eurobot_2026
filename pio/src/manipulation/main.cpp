#include <Arduino.h>
#include <stdint.h>

#include <micro_ros_platformio.h>
#include "custom_msgs/msg/blocks.h"
#include <std_msgs/msg/u_int8.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include "HardwareManager.h"

rclc_executor_t executor;  // executor to handle timers/callbacks
rclc_support_t support;    // support structure for rclc
rcl_allocator_t allocator; // allocator used by rcl
rcl_node_t node;           // ROS node handle

rcl_publisher_t status_pub;
rcl_subscription_t sub_legacy_blocks;
rcl_subscription_t sub_pickup;
rcl_subscription_t sub_flip;
rcl_subscription_t sub_dispense;

custom_msgs__msg__Blocks legacy_blocks_msg;
custom_msgs__msg__Blocks pickup_msg;
custom_msgs__msg__Blocks flip_msg;
std_msgs__msg__UInt8 dispense_msg;
std_msgs__msg__UInt8 status_msg;

enum ManipStatus : uint8_t {
  STATUS_IDLE = 0,
  STATUS_PICKUP_STARTED = 1,
  STATUS_PICKUP_DONE = 2,
  STATUS_PICKUP_FAILED = 3,
  STATUS_FLIP_STARTED = 4,
  STATUS_FLIP_DONE = 5,
  STATUS_FLIP_FAILED = 6,
  STATUS_DISPENSE_STARTED = 7,
  STATUS_DISPENSE_DONE = 8,
  STATUS_DISPENSE_FAILED = 9,
};

HardwareManager Hardware;

void publish_status(uint8_t code) {
  status_msg.data = code;
  rcl_publish(&status_pub, &status_msg, NULL);
}

// ---- Callbacks ---- //

void legacy_blocks_callback(const void * msgin) {
  const custom_msgs__msg__Blocks * msg =
      (const custom_msgs__msg__Blocks *)msgin;

  int cups[4];
  int flip_colors[4];
  // CHANGE: else condition
  for (int i = 0; i < 4; i++) {
    cups[i] = msg->colors[i] != 0 ? 1 : 0;
    flip_colors[i] = msg->colors[i] == 2 ? 2 : 0;
  }

  publish_status(STATUS_PICKUP_STARTED);
  if (!Hardware.pickUp(cups)) {
    publish_status(STATUS_PICKUP_FAILED);
    return;
  }

  for (int i = 0; i < 4; ++i) {
    if (flip_colors[i] != 0) {
      publish_status(STATUS_FLIP_STARTED);
      if (!Hardware.flip(flip_colors)) {
        publish_status(STATUS_FLIP_FAILED);
        return;
      }
      publish_status(STATUS_FLIP_DONE);
      break;
    }
  }

  publish_status(STATUS_PICKUP_DONE);
}

void pickup_callback(const void * msgin) {
  const custom_msgs__msg__Blocks * msg =
      (const custom_msgs__msg__Blocks *)msgin;
  // CHANGE: else condition | pump order
  int cups[4];
  for (int i = 0; i < 4; i++) {
    cups[i] = msg->colors[i] != 0 ? 1 : 0;
  }

  publish_status(STATUS_PICKUP_STARTED);
  if (Hardware.pickUp(cups)) {
    publish_status(STATUS_PICKUP_DONE);
  } else {
    publish_status(STATUS_PICKUP_FAILED);
  }
}

void flip_callback(const void * msgin) {
  const custom_msgs__msg__Blocks * msg =
      (const custom_msgs__msg__Blocks *)msgin;


  // CHANGE:  missing block / delete block manager
  int colors[4];
  for (int i = 0; i < 4; i++) {
    colors[i] = msg->colors[i];
  }

  publish_status(STATUS_FLIP_STARTED);
  if (Hardware.flip(colors)) {
    publish_status(STATUS_FLIP_DONE);
  } else {
    publish_status(STATUS_FLIP_FAILED);
  }
}

// Called when dispense_action.py publishes on /manip_node/dispense
// data: number of blocks to dispense
void dispense_callback(const void * msgin) {
  // const std_msgs__msg__UInt8 * msg = (const std_msgs__msg__UInt8 *)msgin;
  // uint8_t count = msg->data;
  // Change: what???
  publish_status(STATUS_DISPENSE_STARTED);
  if (Hardware.dropOff()) {
    publish_status(STATUS_DISPENSE_DONE);
  } else {
    publish_status(STATUS_DISPENSE_FAILED);
  }
}

void setup() {
  Serial.begin(115200);
  // set_microros_serial_transports(Serial);
  IPAddress agent_ip(192,168,1,131);

  set_microros_wifi_transports(
      "GRUM",
      "GELE>GMEC",
      agent_ip,
      7777
  );
  
  // Wait for agent
  while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
      delay(1000);
  }
  Hardware.init();

  allocator = rcl_get_default_allocator();

  rclc_support_init(
    &support,
    0,
    NULL,
    &allocator
  );

  rclc_node_init_default(
    &node,
    "manipulator_node",
    "",
    &support
  );

  rclc_publisher_init_default(
    &status_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "/manip_node/status"
  );

  // Legacy combined pick+flip command path.
  rclc_subscription_init_default(
    &sub_legacy_blocks,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(custom_msgs, msg, Blocks),
    "/manip_node/blocks"
  );

  // Subscribe to /manip_node/pickup  (pickup command from ROS2)
  rclc_subscription_init_default(
    &sub_pickup,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(custom_msgs, msg, Blocks),
    "/manip_node/pickup"
  );

  // Subscribe to /manip_node/flip  (flip command from ROS2)
  rclc_subscription_init_default(
    &sub_flip,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(custom_msgs, msg, Blocks),
    "/manip_node/flip"
  );

  // Subscribe to /manip_node/dispense  (dispense command from ROS2)
  rclc_subscription_init_default(
    &sub_dispense,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "/manip_node/dispense"
  );

  // Executor: 2 subscriptions
  rclc_executor_init(
    &executor,
    &support.context,
    4,
    &allocator
  );

  rclc_executor_add_subscription(
    &executor,
    &sub_legacy_blocks,
    &legacy_blocks_msg,
    &legacy_blocks_callback,
    ON_NEW_DATA
  );

  rclc_executor_add_subscription(
    &executor,
    &sub_pickup,
    &pickup_msg,
    &pickup_callback,
    ON_NEW_DATA
  );

  rclc_executor_add_subscription(
    &executor,
    &sub_flip,
    &flip_msg,
    &flip_callback,
    ON_NEW_DATA
  );

  rclc_executor_add_subscription(
    &executor,
    &sub_dispense,
    &dispense_msg,
    &dispense_callback,
    ON_NEW_DATA
  );

  // Change: blocking blocks (dual core????)
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
