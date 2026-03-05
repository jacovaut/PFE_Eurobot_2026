// deadwheels/main.cpp
// micro-ROS example: publish an encoder count from an ESP32 to a ROS2 topic
// Hardware:
//  - ESP32
//  - Rotary encoder attached to GPIO pins 22 (A) and 23 (B)
//  - On-board LED used as a heartbeat on pin defined below

#include <Arduino.h>
#include <ESP32Encoder.h>
#include <micro_ros_platformio.h>
#include <cmath>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include "deadwheel_msgs/msg/deadwheel_ticks.h"

// LED used as a simple heartbeat indicator (typically the on-board LED)
#define LED_PIN 2

//Define variables for odometry
struct odo{
double x = 0;
double y = 0;
double angle = 0;
double vx = 0;
double vy = 0;
double vangle = 0;
}deadwheelodo;

//fonction pour normaliser les angles à l'intervalle [-pi, pi]
double normalizeAngleSigned(double angle) {
    return std::remainder(angle, 2.0 * M_PI);
}

//Ajoute les différences de position relatives à la position connue basée sur les encodeurs + trouve les vitesses 
class deadwheels{
  private : 
    //Constantes, à ajouter bonnes valeurs
    double ENCODER_TICKS_PER_REVOLUTION [3] = {4096, 4096, 4096};
    double DEADWHEEL_DIAMETER = 0.0373;
    double DEADWHEEL_CIRCUMFERENCE = (M_PI) * DEADWHEEL_DIAMETER;
    double DEADWHEEL_DISTANCE = 0.1446; //distance entre les deux deadwheel principaux
    double OFFSET = 0.063; //distance entre le side deadwheel et le centre de rotation du robot

    int64_t prevTicks[3] = {0, 0, 0};
    double prevTime = 0.0;
    bool initialized = false;

  public : 
    void deadwheel_odometry(int64_t ticks0, int64_t ticks1, int64_t ticks2, double time){
      //pour le premier appel
      if (!initialized) {
        prevTicks[0] = ticks0;
        prevTicks[1] = ticks1;
        prevTicks[2] = ticks2;
        prevTime = time;
        initialized = true;
        return;
      }

      //calculs des delta ticks
      int64_t dRTicks = ticks0 - prevTicks[0];
      int64_t dLTicks = ticks1 - prevTicks[1];
      int64_t dSTicks = ticks2 - prevTicks[2];
      prevTicks[0] = ticks0;
      prevTicks[1] = ticks1;
      prevTicks[2] = ticks2;

      //calculs des déplacements selon les axes du robot
      double rightDist = static_cast<double>(dRTicks) * DEADWHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION[0];
      double leftDist = static_cast<double>(dLTicks) * DEADWHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION[1];
      double dxr = 0.5 * (rightDist + leftDist);
      double dangle = (rightDist - leftDist) / DEADWHEEL_DISTANCE;
      double dyr = (static_cast<double>(dSTicks) * DEADWHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION[2]) - OFFSET * dangle;
      double avgangle = deadwheelodo.angle + dangle/2; 

      //calculs des déplacements selon le world
      double c = std::cos(avgangle);
      double s = std::sin(avgangle);
      double dx = dxr*c - dyr*s;
      double dy = dyr*c + dxr*s;
      
      //mise à jour de la position du robot
      deadwheelodo.x = deadwheelodo.x + dx;
      deadwheelodo.y = deadwheelodo.y + dy;
      deadwheelodo.angle = normalizeAngleSigned(deadwheelodo.angle + dangle);

      //calculs de vitesse
      double dt = time - prevTime;
      prevTime = time;
      if (dt <= 1e-6) return;
      deadwheelodo.vx = dx/dt;
      deadwheelodo.vy = dy/dt;
      deadwheelodo.vangle = dangle/dt;
      
    }
};

//Define odometry update callback
deadwheels callback;

// Encoder instance (ESP32-specific library)
ESP32Encoder encoder0;
ESP32Encoder encoder1;
ESP32Encoder encoder2;

// micro-ROS / rclc objects using custom message
rcl_publisher_t deadwheel_pub;
deadwheel_msgs__msg__DeadwheelTicks deadwheel_msg;

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
    // Read tick counts (absolute)
    int64_t ticks0 = encoder0.getCount();
    int64_t ticks1 = encoder1.getCount();
    int64_t ticks2 = encoder2.getCount();

    deadwheel_msg.t0 = ticks0;
    deadwheel_msg.t1 = ticks1;
    deadwheel_msg.t2 = ticks2;

    // Timestamp (ROS-synced epoch time)
    const uint64_t now_ns = rmw_uros_epoch_nanos();
    deadwheel_msg.header.stamp.sec = (int32_t)(now_ns / 1000000000ULL);
    deadwheel_msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000ULL);

    // Publish one message containing all 3 ticks + time (soft-check errors)
    RCSOFTCHECK(rcl_publish(&deadwheel_pub, &deadwheel_msg, NULL));

    //Update local odometry
    double time = esp_timer_get_time() * 1e-6;
    callback.deadwheel_odometry(ticks0, ticks1, ticks2, time);

    // toggle LED to show activity
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
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
  encoder0.attachHalfQuad(2, 19);
  encoder1.attachHalfQuad(36, 34);
  encoder2.attachHalfQuad(35, 32);
  encoder0.setCount(0); // reset counter
  encoder1.setCount(0); // reset counter
  encoder2.setCount(0); // reset counter

  // Setup micro-ROS transport over the Arduino Serial instance
  set_microros_serial_transports(Serial);

  rmw_uros_sync_session(1000);  // try to sync with agent for up to 1s

  // Initialize rclc support and node/publisher structures
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "encoder_node", "", &support));
  RCCHECK(rclc_publisher_init_default(
    &deadwheel_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(deadwheel_msgs, msg, DeadwheelTicks),
    "deadwheel_ticks"));
  
  // Create a periodic timer (timeout in milliseconds)
  const unsigned int timer_timeout = 10; // ms
  RCCHECK(rclc_timer_init_default(
      &timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  // Initialize executor with one handle and add the timer to it
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

}

// Arduino main loop: let the rclc executor run callbacks periodically
void loop()
{
  // spin_some will run ready callbacks for up to the provided timeout
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
