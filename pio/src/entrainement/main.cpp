// main motion control for a 4-wheeled mecanum robot
// subscriber :
//      - cmd_vel
//      - odometry (optional, not implemented yet)
// publisher :
//      - wheel speeds (optional, not implemented yet)
//
// wheel configuration:
//        Front        ⊤
//  [0] \        / [1] |
//                     |
//                     |   
//                     | lx
//                     |
//  [2] /        \ [3] |
//        Back         ⊥
//  ⊢-----------------⊣
//           ly
//
// to change acceleraiton, or current, edit MotorDriver.h
// to change microstepping or decay mode, edit MotorDriver.cpp
//
// not sure if needed and not implemented yet:
//      - PID control on wheel speeds based on odometry feedback
//      - acceleration/deceleration ramps / configurable acceleration
//      - max speed limits
//      - move calculation of wheel speeds to MotorDriver class
//      - Add wheel placement definition to MotorDriver class
//
// jacov 2025

#include <Arduino.h>
#include "MotorDriver.h"
#include "deadwheels.h"
#include <stdint.h>
#include <micro_ros_platformio.h>

/* ---------------------- ROS INCLUDES ---------------------- */
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include "deadwheel_msgs/msg/deadwheel_ticks.h"
#include <geometry_msgs/msg/twist.h>
/* ---------------------------------------------------------- */

/* ---------------------- ROS DEFENITIONS ---------------------- */
TaskHandle_t core1_handle, core2_handle;
rcl_subscription_t cmdvel_sub;
geometry_msgs__msg__Twist cmdvel_msg;

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
    pinMode(2, OUTPUT);
    while (true)
    {
        digitalWrite(2, !digitalRead(2));
        delay(100);
    }
}
/* ------------------------------------------------------------- */

deadwheels Deadwheel(2, 19, 36, 34, 35, 32); // A0, B0, A1, B1, A2, B2

struct CmdVel {
  float vx;
  float vy;
  float w;
  uint32_t seq;
};

volatile CmdVel cmdvel = {0.0f, 0.0f, 0.0f, 0};

void cmdvel_callback(const void* msgin)
{
  const geometry_msgs__msg__Twist* msg =
      (const geometry_msgs__msg__Twist*)msgin;

  // Sequence-lock update
  cmdvel.seq++;
  cmdvel.vx = msg->linear.x;
  cmdvel.vy = msg->linear.y;
  cmdvel.w  = msg->angular.z;
  cmdvel.seq++;
}


// Timer callback invoked by the rclc executor on each timer tick.
// Reads the encoder count, publishes it, and toggles the heartbeat LED.
void timercallback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    //       // Later, monitor both:
    // Serial.printf("Core1: %u, Core2: %u\n", 
    // uxTaskGetStackHighWaterMark(core1_handle),
    // uxTaskGetStackHighWaterMark(core2_handle));
    // Read tick counts
    int64_t ticks [3];
    Deadwheel.getCount(ticks);

    deadwheel_msg.t0 = ticks[0];
    deadwheel_msg.t1 = ticks[1];
    deadwheel_msg.t2 = ticks[2];

    // Timestamp (ROS-synced epoch time)
    const uint64_t now_ns = rmw_uros_epoch_nanos();
    deadwheel_msg.header.stamp.sec = (int32_t)(now_ns / 1000000000ULL);
    deadwheel_msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000ULL);

    // Publish one message containing all 3 ticks + time (soft-check errors)
    RCSOFTCHECK(rcl_publish(&deadwheel_pub, &deadwheel_msg, NULL));

    //Update local odometry
    double time = esp_timer_get_time() * 1e-6; // possible improvement : use rcl time instead of esp_timer
    // Deadwheel.deadwheel_odometry(ticks[0], ticks[1], ticks[2], time);
  }
}

FastAccelStepperEngine engine;

// Motor definitions (StepPin, DirPin, CSPin)
MotorDriver motor1(17, 16, 4);
MotorDriver motor2(15, 13, 12);
MotorDriver motor3(26, 25, 33);
MotorDriver motor4(23, 22, 21);
MotorDriver* motors[] = { &motor1, &motor2, &motor3, &motor4 };

// Robot motion and dimmention variables
float vx = 0; // m/s
float vy = 0; // m/s
float w  = 0; // rad/s

const float lxy = 0.125 + 0.2; // lx (m) + ly (m) = 240mm

const float r = 0.03; //m   (100mm mecanum)

void calculateWheelSpeeds(float vx, float vy, float w, float* wheelSpeeds);
void setSpeed(float new_vx, float new_vy, float new_w);
void core1 (void* pvParameters);
void core2 (void* pvParameters);


void setup() {
    
    allocator = rcl_get_default_allocator();
    
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    
    // Wait for agent
    while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
        delay(1000);
    }
    
    rmw_uros_sync_session(1000);  // try to sync with agent for up to 1s
    
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    
    // create node
    RCCHECK(rclc_node_init_default(
    &node,
    "esp32_motion_node",
    "",
    &support
    ));
    
    // create publisher for deadwheel ticks
    RCCHECK(rclc_publisher_init_default(
        &deadwheel_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(deadwheel_msgs, msg, DeadwheelTicks),
        "deadwheel_ticks"
    ));

    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(10),
        timercallback
    ));
    
    RCCHECK(rclc_executor_init(
        &executor,
        &support.context,
        2,
        &allocator
    ));

    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    
    RCCHECK(rclc_subscription_init_default(
        &cmdvel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    ));
    
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &cmdvel_sub,
        &cmdvel_msg,
        &cmdvel_callback,
        ON_NEW_DATA
    ));
    
    pinMode(27, OUTPUT);
    digitalWrite(27, HIGH);
    
    SPI.begin(18,14,5);
    engine.init();
    Deadwheel.begin();
    
    motor1.begin(engine);
    motor2.begin(engine);
    motor3.begin(engine);
    motor4.begin(engine);
    
    motor1.Enabledriver(true);
    motor2.Enabledriver(true);
    motor3.Enabledriver(true);
    motor4.Enabledriver(true);
    
    xTaskCreatePinnedToCore(
        core1,
        "Control",
        16000,
        NULL,
        3,
        &core1_handle,  
        1   // Core 1
    );

    xTaskCreatePinnedToCore(
        core2,
        "ROS",
        3000,
        NULL,
        1,
        &core2_handle,
        0   // Core 0
    );
}


// set robot speeds in m/s and wheel angular velocity in rad/s
void setSpeed(float new_vx, float new_vy, float new_w) {
    vx = new_vx;
    vy = new_vy;
    w  = new_w;
    
    // stop all motors if no movement
    if (vx == 0 && vy == 0 && w == 0) {
        motor1.stop();
        motor2.stop();
        motor3.stop();
        motor4.stop();
        return;
    }
    
    float wheelSpeeds [4] = {0, 0, 0, 0}; // FL(\), FR(/), RL(/), RR(\)
    
    // get the wheel speeds in rad/s
    calculateWheelSpeeds(vx, vy, w, wheelSpeeds);
    
    for (int i = 0; i < 4; i++) {
        float speed = abs(wheelSpeeds[i]);
        
        motors[i]->setSpeedRPM(speed * 60 / (2 * PI)); // set speed in RPM
        
        if (wheelSpeeds[i] < 0) {
            motors[i]->runBackward();
        }
        else {
            motors[i]->runForward();
        }
    }
}

void calculateWheelSpeeds(float vx, float vy, float w, float* wheelSpeeds) { // self explanatory
    wheelSpeeds[0] = (1/r) * ( vx + vy - w*lxy ); // FL
    wheelSpeeds[1] = (1/r) * ( vx - vy + w*lxy ); // FR
    wheelSpeeds[2] = (1/r) * ( vx - vy - w*lxy ); // RL
    wheelSpeeds[3] = (1/r) * ( vx + vy + w*lxy ); // RR
}


// Control tasks
//   - encoders
//   - odometry
//   - motion control
//   - motor commands
void core1 (void* pvParameters){
    const TickType_t period = pdMS_TO_TICKS(5); // 200 Hz
    TickType_t lastWake = xTaskGetTickCount();

    CmdVel local;
    uint32_t s1, s2;

    for (;;) {
        // sequence-lock read
        do {
            s1 = cmdvel.seq;
            local.vx = cmdvel.vx;
            local.vy = cmdvel.vy;
            local.w  = cmdvel.w;
            s2 = cmdvel.seq;
        } while (s1 != s2 || (s1 & 1));

        setSpeed(local.vx, local.vy, local.w);

        vTaskDelayUntil(&lastWake, period);
    }
}

// ROS2 communication tasks
//   - rclc executor
//   - publishers / subscribers
void core2(void* pvParameters){
    for (;;)
    {
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
        vTaskDelay(1);
    }
}

void loop(){vTaskDelay(portMAX_DELAY);}