#pragma once
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <micro_ros_platformio.h>
#include <cmath>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include "custom_msgs/msg/deadwheel_ticks.h"

// for later use
// //Define odometry update callback
// deadwheels callback;

//Ajoute les différences de position relatives à la position connue basée sur les encodeurs + trouve les vitesses 
class deadwheels{
public :

  deadwheels(int A_0, int B_0, int A_1, int B_1,  int A_2, int B_2);

  void begin();
  void deadwheel_odometry(double ticks0, double ticks1, double ticks2, double time);
  double normalizeAngleSigned(double angle);
  void getCount(int64_t *ticks);
  
  //Define variables for odometry
  struct odo{
  double x = 0;
  double y = 0;
  double angle = 0;
  double vx = 0;
  double vy = 0;
  double vangle = 0;
  }deadwheelodo;

private :

  int A_0, B_0, A_1, B_1, A_2, B_2; // pin numbers for encoders

  //Constantes, à ajouter bonnes valeurs
  double ENCODER_TICKS_PER_REVOLUTION [3] = {1000, 1000, 1000};
  double DEADWHEEL_DIAMETER = 2;
  double DEADWHEEL_CIRCUMFERENCE = (M_PI) * DEADWHEEL_DIAMETER;
  double DEADWHEEL_DISTANCE = 10; //distance entre les deux deadwheel principaux
  double OFFSET = 2; //distance entre le side deadwheel et le centre de rotation du robot

  double prevTicks[3] = {0, 0, 0};
  double prevTime = 0.0;
  bool initialized = false;

  // Encoder instance (ESP32-specific library)
  ESP32Encoder encoder0;
  ESP32Encoder encoder1;
  ESP32Encoder encoder2;
};
