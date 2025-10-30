#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>

rcl_publisher_t publisher;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_init_options_t init_options;
std_msgs__msg__Int32 msg;

void setup() {
  Serial.begin(115200);

  // Initialize micro-ROS transport over serial
  set_microros_transports();

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "esp32_node", "", &support);
  rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "esp32_counter"
  );
}

void loop() {
  static int count = 0;
  msg.data = count++;
  rcl_publish(&publisher, &msg, NULL);
  delay(1000);
}