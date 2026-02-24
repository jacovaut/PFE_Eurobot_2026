#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include "Alimentation_Pinout.h"

bool RobotActivated = false;

//Function Signatures

int ReadCurrent();
int ReadVoltage();


// micro-ROS / rclc objects using custom message
rcl_publisher_t alimentation_pub;
// Message structure for publishing current and voltage values
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


void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    return; // Timer callback is currently empty, but can be used for periodic tasks if needed
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  // Setup micro-ROS transport over the Arduino Serial instance
  set_microros_serial_transports(Serial);

  rmw_uros_sync_session(1000);  // try to sync with agent for up to 1s

  // Initialize rclc support and node/publisher structures
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "alimentation_node", "", &support));

  /*RCCHECK(rclc_publisher_init_default(
    &alimentation_pub, &node,
    //ROSIDL_GET_MSG_TYPE_SUPPORT(deadwheel_msgs, msg, DeadwheelTicks),
    "deadwheel_ticks")); */
  
  // Initialize a timer to trigger every 10ms (100Hz)
 const unsigned int timer_timeout = 10; // ms

 RCCHECK(rclc_timer_init_default(
      &timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  // Initialize executor with one handle and add the timer to it
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  //Setup pin modes
  pinMode(CDAT, INPUT);
  pinMode(VDAT, INPUT);
  pinMode(ActPlug, INPUT);

  //Setup Mosfets pins
  pinMode(SOFST, OUTPUT);
  pinMode(MOSF, OUTPUT);

    // Activate slow start mosfet
  digitalWrite(SOFST, HIGH); // Activate the MOSFET to allow current flow

}

void loop()
{
  // Spin the executor to handle timer callbacks and other events
   RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  // Robot Activation Logic
  int Startup = digitalRead(ActPlug);
  
  if (Startup == HIGH) {
    RobotActivated = true;
    Serial.println("Activate Main Robot");    //Going to be a micro Ros Topic Message in the future
  }

  //Current Measurement Value
   int sensorcurrentvalue = ReadCurrent(); // Read current value from the function
  int sensorvoltagevalue = ReadVoltage(); // Read voltage value from the function

  if (sensorcurrentvalue >= 30) 
  { // If current exceeds 30A (mapped value), send a warning message
  
    Serial.println("Warning: High Current Detected!"); // Going to be a micro Ros Topic Message in the future
    digitalWrite(MOSF, LOW); // Deactivate the MOSFET to cut off power

  };

    //Graphical Representation of Voltage and Current Values (for displaying purposes)


}

  int ReadCurrent() 
  {
    
    int sensorcurrentvalue = analogRead(CDAT);

    //Convert the raw ADC values to voltage and current using the map function
    
    int currentvalue = map(sensorcurrentvalue, 0, 4095, 0, 30); // Map the ADC value to a current range (0-30A)

    Serial.println("Current Value: " + String(currentvalue));
    return currentvalue;
  }

  int ReadVoltage() {
    int sensorvoltagevalue = analogRead(VDAT);
    int voltagevalue = map(sensorvoltagevalue, 0, 4095, 0.02445, 25); // Map the ADC value to a voltage range (0-25V)
    
    Serial.println("Voltage Value: " + String(voltagevalue));
    return voltagevalue;
  }




