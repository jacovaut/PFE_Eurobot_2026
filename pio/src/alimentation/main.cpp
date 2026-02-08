// alimentation/main.cpp
// micro-ROS example: publish an encoder count from an ESP32 to a ROS2 topic
// Hardware:
//  - ESP32

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <cmath>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

//Ajoute les différences de position relatives à la position connue basée sur les encodeurs + trouve les vitesses 
class alimentation{
  private:
    bool initialized = false;
    double prevTime = 0.0;

  public:
    void alimentation_odometry(double time){
      //pour le premier appel
      if (!initialized) {
        prevTime = time;
        initialized = true;
        return;
      }

      //calculs des delta ticks
      double dTime = time - prevTime;
      prevTime = time;

    }
};

// Timer callback invoked by the rclc executor on each timer tick.
// Reads the encoder count, publishes it, and toggles the heartbeat LED.
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
 
}

// Standard Arduino setup function: initializes hardware and micro-ROS
void setup()
{
  Serial.begin(115200);
  Serial.println("Starting micro-ROS alimentation example");
  pinMode(LED_PIN, OUTPUT);


}

// Arduino main loop: let the rclc executor run callbacks periodically
void loop()
{
  Serial.println("Running alimentation eps32");
  delay(1000);

}
