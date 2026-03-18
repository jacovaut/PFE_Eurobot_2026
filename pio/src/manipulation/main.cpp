#include <Arduino.h>
#include <stdint.h>

#include <micro_ros_platformio.h>
#include "custom_msgs/msg/blocks.h"
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
rcl_timer_t timer;         // periodic timer

rcl_publisher_t pub;
rcl_subscription_t sub;
custom_msgs__msg__Blocks msg;

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

void subscription_callback(const void * msgin) {

}

void setup() {
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

  rclc_subscription_init_default(
    &sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(custom_msgs, msg, Blocks),
    "block_queue_cmd"
  );

  rclc_executor_init(
    &executor,
    &support.context,
    1,
    &allocator
  );

  rclc_executor_add_subscription(
    &executor,
    &sub,
    &msg,
    &subscription_callback,
    ON_NEW_DATA
  );
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
