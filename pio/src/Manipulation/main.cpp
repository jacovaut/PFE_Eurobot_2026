#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <std_msgs/msg/string.h>

// block color enum
enum BlockColor : uint8_t {
  COLOR_UNKNOWN = 0,
  COLOR_BLUE    = 1,
  COLOR_YELLOW  = 2,
};

struct BlockEntry {
  BlockColor color;
  uint8_t cup_id;  // 0..3
};

constexpr int QUEUE_SIZE = 12;
BlockEntry block_queue[QUEUE_SIZE];
int queue_head = 0;
int queue_tail = 0;

bool queue_empty() {
  return queue_head == queue_tail;
}

bool queue_full() {
  return ((queue_tail + 1) % QUEUE_SIZE) == queue_head;
}

bool queue_push(BlockEntry e) {
  if (queue_full()) return false;
  block_queue[queue_tail] = e;
  queue_tail = (queue_tail + 1) % QUEUE_SIZE;
  return true;
}

bool queue_pop(BlockEntry &out) {
  if (queue_empty()) return false;
  out = block_queue[queue_head];
  queue_head = (queue_head + 1) % QUEUE_SIZE;
  return true;
}

constexpr int LIMIT_SWITCH_PIN = 14; // change to your pin

rcl_publisher_t pub;
rcl_subscription_t sub;
std_msgs__msg__String msg;

void subscription_callback(const void * msgin) {
  const std_msgs__msg__String * m = (const std_msgs__msg__String *)msgin;
  String s = String(m->data.data, m->data.size);
  // Split by comma and push each color
  int start = 0;
  while (start < s.length()) {
    int comma = s.indexOf(',', start);
    String colorStr = (comma == -1) ? s.substring(start) : s.substring(start, comma);
    colorStr.trim();
    BlockEntry e;
    if (colorStr == "yellow") e.color = COLOR_YELLOW;
    else if (colorStr == "blue") e.color = COLOR_BLUE;
    else e.color = COLOR_UNKNOWN;
        if (!queue_push(e)) {
      Serial.println("queue full, dropping");
    }
    if (comma == -1) break;
    start = comma + 1;
  }
}

void setup() {
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  //GPIOs for motorBras, MotorFlip, MotorStop, MotorPump, Valve1, Valve2, Valve3, Valve4
  //Different functions I will be needing, Load(), 

  // ---- FROM PCB MANIPULATION ---- //
  //GPIO 5 (D4) = M1_I1
  //GPIO 8 (D5) = M1_I2
  //GPIO 3 (D15) = M2_I1
  //GPIO 4 (D2) = M2_I2
  //GPIO 21 (D32) = MOS1 , Control the Pump
  //GPIO 23 (D25) = MOS2 , Control the Valve1
  //GPIO 24 (D26) = MOS3 , Control the Valve2
  //GPIO 25 (D27) = MOS4 , Control the Valve3
  //GPIO 26 (D14) = MOS5 , Control the Valve4
  //GPIO 27 (D12) = MOS6 , Control the Valve_out
  //GPIO 22 (D33) = MOS7 , Control Thermo?
  //GPIO 28 (D13) = STOP , Connected to Stopper?
  //GPIO 12 (RXO) = TXMA , Connected to Rpi JST
  //GPIO 13 (TXO) = RXMA , Connected to Rpi JST

  rclc_support_t support;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  rcl_node_t node;
  rcl_node_init_default(&node, "manipulator_node", "", &support);

  rclc_subscription_init_default(
    &sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "block_queue_cmd"
  );

  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, 1, &allocator);
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
  static bool last_switch = HIGH;
  bool s = digitalRead(LIMIT_SWITCH_PIN);
  if (last_switch == HIGH && s == LOW) {
    // falling edge = block left the robot
    BlockEntry popped;
    if (queue_pop(popped)) {
      // do something with popped, e.g. log or publish
      Serial.printf("popped block cup=%u color=%u\n", popped.cup_id, (uint8_t)popped.color);
    }
  }
  last_switch = s;

  // ... rest of loop ...
}
