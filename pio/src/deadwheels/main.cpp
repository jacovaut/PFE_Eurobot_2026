// deadwheels/main.cpp
// micro-ROS example: publish an encoder count from an ESP32 to a ROS2 topic
// Hardware:
//  - ESP32
//  - Rotary encoder attached to GPIO pins 22 (A) and 23 (B)
//  - On-board LED used as a heartbeat on pin defined below
// ROS2/micro-ROS behavior:
//  - Publishes std_msgs/Int32 on topic "encoder_count" once per timer tick
//  - Uses a simple timer-based publisher via rclc executor


// TEST GIT

#include <Arduino.h>
#include <ESP32Encoder.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

// LED used as a simple heartbeat indicator (typically the on-board LED)
#define LED_PIN 2

// Encoder instance (ESP32-specific library)
ESP32Encoder encoder0;
ESP32Encoder encoder1;
ESP32Encoder encoder2;

// micro-ROS / rclc objects
rcl_publisher_t pub0;
rcl_publisher_t pub1;
rcl_publisher_t pub2;
std_msgs__msg__Int32 msg_0;
std_msgs__msg__Int32 msg_1;
std_msgs__msg__Int32 msg_2;

rclc_executor_t executor;  // executor to handle timers/callbacks
rclc_support_t support;    // support structure for rclc
rcl_allocator_t allocator; // allocator used by rcl
rcl_node_t node;           // ROS node handle
rcl_timer_t timer;         // periodic timer

// Helpers for concise error handling. RCCHECK aborts into an error loop
// on failure; RCSOFTCHECK ignores non-fatal errors.
#define RCCHECK(fn){     \
  rcl_ret_t rc = fn;     \
  if (rc != RCL_RET_OK)  \
  {                      \
    error_loop();        \
  }                      \
}

#define RCSOFTCHECK(fn){ \
  rcl_ret_t rc = fn;     \
  if (rc != RCL_RET_OK)  \
  {                      \
  }                      \
}

// Blink LED quickly to signal fatal setup error (never returns)
void error_loop(){
  while (true)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Timer callback invoked by the rclc executor on each timer tick.
// Reads the encoder count, publishes it, and toggles the heartbeat LED.
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    // get current encoder tick count
    msg_0.data = encoder0.getCount();
    msg_1.data = encoder1.getCount();
    msg_2.data = encoder2.getCount();

    // publish the current count (soft-check errors)
    RCSOFTCHECK(rcl_publish(&pub0, &msg_0, NULL));
    RCSOFTCHECK(rcl_publish(&pub1, &msg_1, NULL));
    RCSOFTCHECK(rcl_publish(&pub2, &msg_2, NULL));

    // toggle LED to show activity
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // heartbeat
  }
}

// Standard Arduino setup function: initializes hardware and micro-ROS
void setup()
{
  Serial.begin(115200);

  // configure heartbeat LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // start LED on

  // Configure encoder library: enable internal pull-ups and attach pins
  // ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder0.attachHalfQuad(23, 22);
  encoder1.attachHalfQuad(36, 39);
  encoder2.attachHalfQuad(34, 35);
  encoder0.setCount(0); // reset counter
  encoder1.setCount(0); // reset counter
  encoder2.setCount(0); // reset counter

  // Setup micro-ROS transport over the Arduino Serial instance
  set_microros_serial_transports(Serial);

  // Initialize rclc support and node/publisher structures
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "encoder_node", "", &support));
  RCCHECK(rclc_publisher_init_default(
    &pub0, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "encoder_0_count"));
  RCCHECK(rclc_publisher_init_default(
    &pub1, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "encoder_1_count"));
  RCCHECK(rclc_publisher_init_default(
    &pub2, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "encoder_2_count"));
  
  // Create a periodic timer (timeout in milliseconds)
  const unsigned int timer_timeout = 1000; // ms
  RCCHECK(rclc_timer_init_default(
      &timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  // Initialize executor with one handle and add the timer to it
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // initialize published message value
  msg_0.data = 0;
  msg_1.data = 0;
  msg_2.data = 0;
}

// Arduino main loop: let the rclc executor run callbacks periodically
void loop()
{
  // spin_some will run ready callbacks for up to the provided timeout
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
