#include <Arduino.h>
#include <stdint.h>

#include <micro_ros_platformio.h>
#include "custom_msgs/msg/blocks.h"
#include <std_msgs/msg/u_int8.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include "BlockManager.h"
#include "HardwareManager.h"

rclc_executor_t executor;  // executor to handle timers/callbacks
rclc_support_t support;    // support structure for rclc
rcl_allocator_t allocator; // allocator used by rcl
rcl_node_t node;           // ROS node handle

// Two subscriptions: blocks (pick) and dispense
rcl_subscription_t sub_blocks;
rcl_subscription_t sub_dispense;

custom_msgs__msg__Blocks   blocks_msg;
std_msgs__msg__UInt8       dispense_msg;

// ---- FROM PCB MANIPULATION ---- //
constexpr int M1_I1 = 4;
constexpr int M1_I2 = 5;
constexpr int M2_I1 = 15;
constexpr int M2_I2 = 2;
constexpr int MOS1 = 32; // Control the Pump
constexpr int MOS2 = 25; // Control the Valve1
constexpr int MOS3 = 26; // Control the Valve2
constexpr int MOS4 = 27; // Control the Valve3
constexpr int MOS5 = 14; // Control the Valve4
constexpr int MOS6 = 12; // Control the Valve_out
constexpr int MOS7 = 33; // Control Thermo?
constexpr int STOP = 13; // Connected to Stopper?

BlockManager BlockList;
HardwareManager Hardware(&BlockList);

// ---- Callbacks ---- //

// Called when pick_action.py publishes on /manip_node/blocks
// colors[0..3]: 0=not present, 1=pick as-is, 2=pick and flip
// count: number of blocks present
void blocks_callback(const void * msgin) {
  const custom_msgs__msg__Blocks * msg =
      (const custom_msgs__msg__Blocks *)msgin;

  int cups[4];
  for (int i = 0; i < 4; i++) {
    cups[i] = msg->colors[i];
  }
  Hardware.pickUp(cups);
}

// Called when dispense_action.py publishes on /manip_node/dispense
// data: number of blocks to dispense
void dispense_callback(const void * msgin) {
  // const std_msgs__msg__UInt8 * msg = (const std_msgs__msg__UInt8 *)msgin;
  // uint8_t count = msg->data;
  Hardware.dropOff();
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000); // give the agent time to connect

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

  // Subscribe to /manip_node/blocks  (pick command from ROS2)
  rclc_subscription_init_default(
    &sub_blocks,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(custom_msgs, msg, Blocks),
    "/manip_node/blocks"
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
    2,
    &allocator
  );

  rclc_executor_add_subscription(
    &executor,
    &sub_blocks,
    &blocks_msg,
    &blocks_callback,
    ON_NEW_DATA
  );

  rclc_executor_add_subscription(
    &executor,
    &sub_dispense,
    &dispense_msg,
    &dispense_callback,
    ON_NEW_DATA
  );
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
