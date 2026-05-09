// bldc_control.ino
// Arduino Nano #2 — Sunnysky V3508-19 x2 via BlHeli 30A ESCs
//
// Serial protocol (115200 baud):
//   Pi -> Arduino:  "1\n"  = motors ON  (full throttle)
//                   "0\n"  = motors OFF (neutral / disarmed)
//
// ESC signal:
//   1000 µs = minimum / disarmed
//   1500 µs = neutral (use this for "off" on bidirectional ESCs)
//   2000 µs = maximum throttle
//
// BlHeli arming sequence: send 1000 µs for ~2 s on power-up, then
// the ESC accepts commands. This sketch handles that in setup().

#include <Servo.h>

#define ESC_A_PIN 9
#define ESC_B_PIN 10

// Adjust these to match your desired "on" throttle.
// 2000 = full throttle. Start lower (e.g. 1600) during testing!
#define THROTTLE_OFF  1000
#define THROTTLE_ON   1100  // <-- tune this safely before going higher

Servo escA;
Servo escB;

bool motors_enabled = false;

void setMotors(bool enable)
{
  int signal = enable ? THROTTLE_ON : THROTTLE_OFF;
  escA.writeMicroseconds(signal);
  escB.writeMicroseconds(signal);
  motors_enabled = enable;
}

void setup()
{
  Serial.begin(115200);

  escA.attach(ESC_A_PIN);
  escB.attach(ESC_B_PIN);

  // BlHeli arming: hold minimum signal for 2 seconds
  escA.writeMicroseconds(THROTTLE_OFF);
  escB.writeMicroseconds(THROTTLE_OFF);
  delay(2000);

  Serial.println("BLDC ready");
}

void loop()
{
  if (Serial.available())
  {
    String msg = Serial.readStringUntil('\n');
    msg.trim();

    if (msg == "1") {
      setMotors(true);
      Serial.println("ON");
    }
    else if (msg == "0") {
      setMotors(false);
      Serial.println("OFF");
    }
  }
}
