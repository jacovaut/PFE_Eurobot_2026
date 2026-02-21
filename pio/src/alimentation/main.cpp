#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

// Pin definitions
#define MOSF 18
#define CDAT 13
#define VDAT 14
#define ActPlug 22
#define Ex2 23
#define Ex3 25
#define Ex4 26
#define Ex5 27
#define Ex6 32
#define Ex7 35
#define Ex8 34
#define TX  1
#define RX 2
#define SOFST 33

HardwareSerial Serial2(2);

bool RobotActivated = false;


int ReadCurrent();
int ReadVoltage();

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(2000); // Wait for the serial connection to be established
  Serial.println("Serial Monitor Started");

  //Initialize UART Communication with Rapsberry PI

  Serial2.begin(115200, SERIAL_8N1, RX, TX);
  Serial.println("UART Communication Initialized with Raspberry Pi");

  // Initialize micro-ROS
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create a node
  rcl_node_t node;
  rclc_node_init_default(&node, "alimentation_node", "", &support);

  //Setup pin modes
  pinMode(CDAT, INPUT);
  pinMode(VDAT, INPUT);
  pinMode(ActPlug, INPUT);
    //Mosfets
  pinMode(SOFST, OUTPUT);
  pinMode(MOSF, OUTPUT);

    // Activate slow start mosfet
  digitalWrite(SOFST, HIGH); // Activate the MOSFET to allow current flow
}

void loop()
{
 
  // Robot Activation Logic
  int Startup = digitalRead(ActPlug);
  
  if (Startup == HIGH) {
    RobotActivated = true;
    Serial2.println("Activate Main Robot");    //Going to be a micro Ros Topic Message in the future
  }

  //Current Measurement Value
   int sensorcurrentvalue = ReadCurrent(); // Read current value from the function
  int sensorvoltagevalue = ReadVoltage(); // Read voltage value from the function

  if (sensorcurrentvalue == 30) { // If current exceeds 30A (mapped value), send a warning message
  
    Serial.println("Warning: High Current Detected!"); // Going to be a micro Ros Topic Message in the future
    
    Serial2.println("High Current Detected!"); // Going to be a micro Ros Topic Message in the future
    digitalWrite(MOSF, LOW); // Deactivate the MOSFET to cut off power
  };

    //Graphical Representation of Voltage and Current Values (for displaying purposes

}

  int ReadCurrent() {
    
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




