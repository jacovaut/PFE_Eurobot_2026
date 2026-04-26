#include <Arduino.h>
#include <ESP32Encoder.h>
#include <stdlib.h>

ESP32Encoder encoder;
 
// ---- Hardware pins ----
struct EncoderPins
{
  const int a_pin;
  const int b_pin;
  const int x_pin;
};
 
struct MotorPins
{
  int in1_pin;
  int in2_pin;
};
 
static const EncoderPins pick_encoder = {21, 19, 18};
static const MotorPins pick_motor = {5, 4};

// ---- Encoder ----
// AMT102-V at 2048 PPR, counting A-rising only → 2048 counts/rev
static const float COUNTS_PER_REV = 1050.0f;

// The angle (deg) the X index pulse represents mechanically — adjust to your setup
static const float HOME_ANGLE_DEG = 0.0f;

volatile bool pick_homed = false;
volatile long home_encoder_offset = 0; // = (HOME_ANGLE_DEG / 360) * COUNTS_PER_REV
 
// ---- Control ----
static const float LOOP_HZ = 500.0f;
static const float DT = 1.0f / LOOP_HZ;
float PWM_MIN = 255.0f; // stiction compensation threshold (tune)

float KP = 1.0f;
float KI = 0.0f;
float KD = 0.01f;
static const float INTEGRALE_MAX = 255.0f - PWM_MIN; // anti-windup clamp (PWM units)

// Velocity low-pass filter: α=0.05 → ~10x smoother, tune up if too sluggish
static const float VEL_ALPHA = 0.005f;

float target_angle = 0.0f;

// ---- State ----
float last_angle = 0.0f;
float prev_error = 0.0f;
float integrale = 0.0f;
float curr_vel = 0.0f;
int final_output = 0;
float debug_error = 0.0f;
 
// ---- Pulse → angle ----
float pulseToAngle(long counts)
{
  float angle = counts / COUNTS_PER_REV * 360.0f;
  return angle;
}
 
// ---- Motor write ----
static void motor_write(int pwm)
{
  // LEDC channel configuration
  const int channel_in1 = 0;  // Channel for IN1
  const int channel_in2 = 1;  // Channel for IN2
  
  if (pwm > 0)
  {
    ledcWrite(channel_in1, abs(pwm));
    ledcWrite(channel_in2, 0);
  }
  else if (pwm < 0)
  {
    ledcWrite(channel_in1, 0);
    ledcWrite(channel_in2, abs(pwm));
  }
  else
  {
    ledcWrite(channel_in1, 1);
    ledcWrite(channel_in2, 1);
  }
}
 
// ---- home ISRs ----
void IRAM_ATTR handlePickXReset()
{
  encoder.setCount(home_encoder_offset); // integer-only, FPU-safe
  if (!pick_homed)
    pick_homed = true;
}
 
// ---- Control loop ( 500 Hz) ----
void onTimer()
{
 
  if (!pick_homed)
    return;
 
  float current_angle = pulseToAngle(encoder.getCount());
 
  // Shortest-path error
  float err = target_angle - current_angle;
 
  // PID
  integrale += err * DT;
  float d_error = (err - prev_error) / DT;
  float pid = err * KP + integrale * KI + d_error * KD;
  float pid_out = pid; // add feedforward term

  final_output = (int)pid_out;
 
  // Stiction compensation
  if (final_output > 0 && final_output < (int)PWM_MIN)
    final_output = (int)PWM_MIN;
  else if (final_output < 0 && final_output > -(int)PWM_MIN)
    final_output = -(int)PWM_MIN;
  if (final_output > 255)
    final_output = 255;
  else if (final_output < -255)
    final_output = -255;

  // Deadband: stop jitter at rest
  if (abs(err) < 6.0f)
  {
    final_output = 0;
    integrale = 0.0f;
  }
 
  motor_write(-1 * final_output);
 
  last_angle = current_angle;
  prev_error = err;
  debug_error = err;
}
 
// ---- Homing: spin until X index fires ----
void homePick()
{
  Serial.println("Homing pick arm...");
  while (!pick_homed)
  {
    motor_write(180);
    delay(10);
  }
  motor_write(0);
  Serial.println("Pick arm homed.");
}
 
// ---- Setup ----
void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(100);
  delay(200);
  Serial.println("Pick arm controller starting...");
 
  // Pre-compute integer offset — no FPU inside ISR
  home_encoder_offset = (long)((HOME_ANGLE_DEG / 360.0f) * COUNTS_PER_REV);
 
// Configure LEDC for motor PWM
  const int frequency = 5000;  // 5 kHz PWM frequency
  const int resolution = 8;    // 8-bit resolution (0-255)
  const int channel_in1 = 0;
  const int channel_in2 = 1;
  
  ledcSetup(channel_in1, frequency, resolution);
  ledcSetup(channel_in2, frequency, resolution);
  ledcAttachPin(pick_motor.in1_pin, channel_in1);
  ledcAttachPin(pick_motor.in2_pin, channel_in2);
 
  pinMode(pick_encoder.a_pin, INPUT);
  pinMode(pick_encoder.b_pin, INPUT);
  pinMode(pick_encoder.x_pin, INPUT_PULLUP);

  pinMode(15, OUTPUT);
  pinMode(2, OUTPUT);

    digitalWrite(15, LOW);
    digitalWrite(2, LOW);

  encoder.attachHalfQuad(pick_encoder.a_pin, pick_encoder.b_pin);
  attachInterrupt(digitalPinToInterrupt(pick_encoder.x_pin), handlePickXReset, RISING);
 
  Serial.println("Ready. Commands: T:<deg>   KP:<val>   KD:<val>");
 
  homePick();
 }
 
// ---- Loop: Teleplot serial output + serial tuning commands ----
void loop()
{
//   Teleplot format: >varName:value
  Serial.print(">angle:");
  Serial.println(pulseToAngle(encoder.getCount()));
 
  Serial.print(">target:");
  Serial.println(target_angle);
 
  Serial.print(">error:");
  Serial.println(debug_error);
 
  Serial.print(">output:");
  Serial.println(final_output);
 
  Serial.print(">PWM_MIN:");
  Serial.println(PWM_MIN);
 
  Serial.print(">encoderCount:");
  Serial.println(encoder.getCount());
 
  // Serial tuning commands
  if (Serial.available() > 0)
  {
    String line = Serial.readStringUntil('\n');
    line.trim();
 
    if (line.startsWith("KP"))
    {
      int colon = line.indexOf(':');
      if (colon > 0)
      {
        float v = line.substring(colon + 1).toFloat();
        if (v >= 0.0f)
        {
          KP = v;
          Serial.printf("KP set to: %.3f\n", KP);
        }
      }
    }
    else if (line.startsWith("KI"))
    {
      int colon = line.indexOf(':');
      if (colon > 0)
      {
        float v = line.substring(colon + 1).toFloat();
        if (v >= 0.0f)
        {
          KI = v;
          integrale = 0.0f; // reset on gain change
          Serial.printf("KI set to: %.4f\n", KI);
        }
      }
    }
    else if (line.startsWith("KD"))
    {
      int colon = line.indexOf(':');
      if (colon > 0)
      {
        float v = line.substring(colon + 1).toFloat();
        if (v >= 0.0f)
        {
          KD = v;
          Serial.printf("KD set to: %.3f\n", KD);
        }
      }
    }
    else if (line.startsWith("T:"))
    {
      float v = line.substring(2).toFloat();
      target_angle = v;
      Serial.printf("Target set to: %.1f deg\n", target_angle);
    }
    else if (line.startsWith("PM:"))
    {
      float v = line.substring(3).toFloat();
      if (v >= 0.0f)
      {
        PWM_MIN = v;
        Serial.printf("PWM_MIN set to: %.0f\n", PWM_MIN);
      }
    }
  }
 
    static uint32_t last = micros();
    uint32_t now = micros();

    if (now - last >= DT) // 500 Hz
    {
        last += DT;
        onTimer();
    }

}