// BTS7960 dual motor driver control
// Replaces L298N H-Bridge

#include <PID_v1.h>

// BTS7960 Motor A (Right Wheel)
#define RPWM_A 9   // PWM - Forward
#define LPWM_A 10  // PWM - Reverse
#define EN_A   11  // Enable (bridge R_EN + L_EN together on the driver board)

// BTS7960 Motor B (Left Wheel) — LPWM/RPWM inverted to compensate for reversed mounting
#define RPWM_B 6   // PWM - Forward (physically reverse due to motor orientation)
#define LPWM_B 5   // PWM - Reverse
#define EN_B   7   // Enable (bridge R_EN + L_EN together on the driver board)

// Wheel Encoders Connection PINs
#define right_encoder_phaseA 2  // Interrupt
#define right_encoder_phaseB 4
#define left_encoder_phaseA  3  // Interrupt
#define left_encoder_phaseB  8

// Encoders
unsigned int right_encoder_counter = 0;
unsigned int left_encoder_counter = 0;
String right_wheel_sign = "p";  // 'p' = positive, 'n' = negative
String left_wheel_sign  = "p";
unsigned long last_millis = 0;
const unsigned long interval = 100;

// Interpret Serial Messages
bool is_right_wheel_cmd   = false;
bool is_left_wheel_cmd    = false;
bool is_right_wheel_forward = true;
bool is_left_wheel_forward  = true;
char value[] = "00.00";
uint8_t value_idx    = 0;
bool is_cmd_complete = false;

// PID
// Setpoint - Desired
double right_wheel_cmd_vel = 0.0;   // rad/s
double left_wheel_cmd_vel  = 0.0;   // rad/s
// Input - Measurement
double right_wheel_meas_vel = 0.0;  // rad/s
double left_wheel_meas_vel  = 0.0;  // rad/s
// Output - Command
double right_wheel_cmd = 0.0;       // 0-255
double left_wheel_cmd  = 0.0;       // 0-255
// Tuning
double Kp_r = 10.0;
double Ki_r = 20.0;
double Kd_r = 0.0;
double Kp_l = 10.0;
double Ki_l = 15.0;
double Kd_l = 0.0;
// Controllers
PID rightMotor(&right_wheel_meas_vel, &right_wheel_cmd, &right_wheel_cmd_vel, Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor (&left_wheel_meas_vel,  &left_wheel_cmd,  &left_wheel_cmd_vel,  Kp_l, Ki_l, Kd_l, DIRECT);

// Helper: write to a BTS7960 channel
// A positive speed drives RPWM, zero drives LPWM (and vice versa for reverse)
void setMotorA(double speed)
{
  if (speed >= 0)
  {
    analogWrite(RPWM_A, (int)speed);
    analogWrite(LPWM_A, 0);
  }
  else
  {
    analogWrite(RPWM_A, 0);
    analogWrite(LPWM_A, (int)(-speed));
  }
}

// Left motor has RPWM/LPWM swapped in defines to correct physical orientation
void setMotorB(double speed)
{
  if (speed >= 0)
  {
    analogWrite(RPWM_B, (int)speed);
    analogWrite(LPWM_B, 0);
  }
  else
  {
    analogWrite(RPWM_B, 0);
    analogWrite(LPWM_B, (int)(-speed));
  }
}

void setup()
{
  // Init BTS7960 pins
  pinMode(RPWM_A, OUTPUT);
  pinMode(LPWM_A, OUTPUT);
  pinMode(EN_A,   OUTPUT);
  pinMode(RPWM_B, OUTPUT);
  pinMode(LPWM_B, OUTPUT);
  pinMode(EN_B,   OUTPUT);

  // Enable both drivers
  digitalWrite(EN_A, HIGH);
  digitalWrite(EN_B, HIGH);

  // Start with motors stopped
  setMotorA(0);
  setMotorB(0);

  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);
  Serial.begin(115200);

  // Init encoders
  pinMode(right_encoder_phaseB, INPUT);
  pinMode(left_encoder_phaseB,  INPUT);
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA),  leftEncoderCallback,  RISING);
}

void loop()
{
  // Read and interpret wheel velocity commands from serial
  if (Serial.available())
  {
    char chr = Serial.read();

    // Right wheel
    if (chr == 'r')
    {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd  = false;
      value_idx = 0;
      is_cmd_complete = false;
    }
    // Left wheel
    else if (chr == 'l')
    {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd  = true;
      value_idx = 0;
    }
    // Positive direction
    else if (chr == 'p')
    {
      if (is_right_wheel_cmd && !is_right_wheel_forward)
        is_right_wheel_forward = true;
      else if (is_left_wheel_cmd && !is_left_wheel_forward)
        is_left_wheel_forward = true;
    }
    // Negative direction
    else if (chr == 'n')
    {
      if (is_right_wheel_cmd && is_right_wheel_forward)
        is_right_wheel_forward = false;
      else if (is_left_wheel_cmd && is_left_wheel_forward)
        is_left_wheel_forward = false;
    }
    // Separator — apply buffered value
    else if (chr == ',')
    {
      if (is_right_wheel_cmd)
        right_wheel_cmd_vel = atof(value);
      else if (is_left_wheel_cmd)
      {
        left_wheel_cmd_vel = atof(value);
        is_cmd_complete = true;
      }
      // Reset value buffer
      value_idx = 0;
      value[0] = '0'; value[1] = '0'; value[2] = '.';
      value[3] = '0'; value[4] = '0'; value[5] = '\0';
    }
    // Numeric characters
    else
    {
      if (value_idx < 5)
      {
        value[value_idx] = chr;
        value_idx++;
      }
    }
  }

  // Encoder velocity computation and PID update
  unsigned long current_millis = millis();
  if (current_millis - last_millis >= interval)
  {
    right_wheel_meas_vel = (10 * right_encoder_counter * (60.0 / 330.0)) * 0.10472;
    left_wheel_meas_vel  = (10 * left_encoder_counter  * (60.0 / 330.0)) * 0.10472;

    rightMotor.Compute();
    leftMotor.Compute();

    // Zero output when command is zero (overcome PID integrator windup at rest)
    if (right_wheel_cmd_vel == 0.0) right_wheel_cmd = 0.0;
    if (left_wheel_cmd_vel  == 0.0) left_wheel_cmd  = 0.0;

    // Send encoder readings back over serial
    String encoder_read = "r" + right_wheel_sign + String(right_wheel_meas_vel)
                        + ",l" + left_wheel_sign  + String(left_wheel_meas_vel) + ",";
    Serial.println(encoder_read);

    last_millis = current_millis;
    right_encoder_counter = 0;
    left_encoder_counter  = 0;

    // Apply PID output — direction flag determines sign passed to motor helpers
    double r_out = is_right_wheel_forward ?  right_wheel_cmd : -right_wheel_cmd;
    double l_out = is_left_wheel_forward  ?  left_wheel_cmd  : -left_wheel_cmd;
    setMotorA(r_out);
    setMotorB(l_out);
  }
}

void rightEncoderCallback()
{
  right_wheel_sign = (digitalRead(right_encoder_phaseB) == HIGH) ? "p" : "n";
  right_encoder_counter++;
}

void leftEncoderCallback()
{
  left_wheel_sign = (digitalRead(left_encoder_phaseB) == HIGH) ? "p" : "n";
  left_encoder_counter++;
}