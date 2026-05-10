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

rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;

rcl_publisher_t    status_pub;
rcl_subscription_t sub_pickup;
rcl_subscription_t sub_home;
rcl_subscription_t sub_thermo;
rcl_subscription_t sub_flip;

custom_msgs__msg__Blocks pickup_msg;
std_msgs__msg__UInt8     home_msg;
std_msgs__msg__UInt8     thermo_msg;
custom_msgs__msg__Blocks flip_msg;
std_msgs__msg__UInt8     status_msg;

enum ManipStatus : uint8_t {
    STATUS_IDLE             = 0,
    STATUS_PICKUP_STARTED   = 1,
    STATUS_PICKUP_DONE      = 2,
    STATUS_PICKUP_FAILED    = 3,
    STATUS_FLIP_STARTED     = 4,
    STATUS_FLIP_DONE        = 5,
    STATUS_FLIP_FAILED      = 6,
    STATUS_HOME_STARTED     = 7,
    STATUS_HOME_DONE        = 8,
    STATUS_HOME_FAILED      = 9,
    STATUS_THERMO_STARTED   = 10,
    STATUS_THERMO_DONE      = 11,
    STATUS_THERMO_FAILED    = 12,
};

HardwareManager Hardware;

void publish_status(uint8_t code) {
    status_msg.data = code;
    rcl_publish(&status_pub, &status_msg, NULL);
}

// ----------------------------------------
// Callbacks
// ----------------------------------------

void pickup_callback(const void* msgin) {
    const custom_msgs__msg__Blocks* msg =
        (const custom_msgs__msg__Blocks*)msgin;

    bool requested_pumps[4];
    for (int i = 0; i < 4; i++) {
        requested_pumps[i] = msg->colors[i] != 0;
    }

    publish_status(STATUS_PICKUP_STARTED);
    if (Hardware.pick(requested_pumps)) {
        publish_status(STATUS_PICKUP_DONE);
    } else {
        publish_status(STATUS_PICKUP_FAILED);
    }
}

void home_callback(const void* msgin) {
    publish_status(STATUS_HOME_STARTED);
    if (Hardware.home()) {
        publish_status(STATUS_HOME_DONE);
    } else {
        publish_status(STATUS_HOME_FAILED);
    }
}

void thermo_callback(const void* msgin) {
    publish_status(STATUS_THERMO_STARTED);
    if (Hardware.thermo()) {
        publish_status(STATUS_THERMO_DONE);
    } else {
        publish_status(STATUS_THERMO_FAILED);
    }
}

void flip_callback(const void* msgin) {
    const custom_msgs__msg__Blocks* msg =
        (const custom_msgs__msg__Blocks*)msgin;

    publish_status(STATUS_FLIP_STARTED);
    bool ok = true;
    for (int i = 0; i < 4; i++) {
        if (msg->colors[i] == 1) {
            if (!Hardware.dispense_flip()) { ok = false; break; }
        } else if (msg->colors[i] == 2) {
            if (!Hardware.dispense_keep()) { ok = false; break; }
        }
    }
    publish_status(ok ? STATUS_FLIP_DONE : STATUS_FLIP_FAILED);
}

// ----------------------------------------
// Setup
// ----------------------------------------

void setup() {
    Serial.begin(115200);

    IPAddress agent_ip(192, 168, 1, 131);
    set_microros_wifi_transports(
        "GRUM",
        "GELE>GMEC",
        agent_ip,
        7777
    );

    while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
        delay(1000);
    }

    Hardware.init();

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "manipulator_node", "", &support);

    rclc_publisher_init_default(
        &status_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        "/manip_node/status"
    );

    rclc_subscription_init_default(
        &sub_pickup, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(custom_msgs, msg, Blocks),
        "/manip_node/pickup"
    );

    rclc_subscription_init_default(
        &sub_home, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        "/manip_node/home"
    );

    rclc_subscription_init_default(
        &sub_thermo, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        "/manip_node/thermo"
    );

    rclc_subscription_init_default(
        &sub_flip, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(custom_msgs, msg, Blocks),
        "/manip_node/flip"
    );

    rclc_executor_init(&executor, &support.context, 4, &allocator);

    rclc_executor_add_subscription(
        &executor, &sub_pickup, &pickup_msg,
        &pickup_callback, ON_NEW_DATA
    );

    rclc_executor_add_subscription(
        &executor, &sub_home, &home_msg,
        &home_callback, ON_NEW_DATA
    );

    rclc_executor_add_subscription(
        &executor, &sub_thermo, &thermo_msg,
        &thermo_callback, ON_NEW_DATA
    );

    rclc_executor_add_subscription(
        &executor, &sub_flip, &flip_msg,
        &flip_callback, ON_NEW_DATA
    );
}

// ----------------------------------------
// Loop
// ----------------------------------------

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}