#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define L1 50   // length of hip joint
#define L2 75   // length of upper leg joint
#define L3 100  // length of lower leg joint
#define L  150  // distance between legs
#define H  30   // height of step
#define T  2000 // period of step
#define A  0    // body angle

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

struct JointAngles {
  float q1;
  float q2;
  float q3;
};

JointAngles IK(float x, float y, float z) {
  JointAngles angles;
  float r = sqrt(x*x + y*y);
  float D = (z - L1)*(z - L1) + r*r;
  angles.q1 = atan2(y, x) * 180 / M_PI;  // angle 1
  angles.q2 = -atan2(z - L1, r) * 180 / M_PI;  // angle 2
  angles.q3 = acos((D - L2*L2 - L3*L3) / (2 * L2*L3)) * 180 / M_PI - 90 - angles.q2;  // angle 3
  return angles;
}

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'f':  // move forward
        for (int i = 0; i < 4; i++) {
          float x = -L / 2 + i * L / 2;                          // x position
          float y = H * sin(2 * M_PI * t / T);                    // y position
          float z = L1 + H * cos(2 * M_PI * t / T) - H;           // z position
          JointAngles angles = IK(x, y, z);                       // compute joint angles
          angles.q1 += A;                                         // adjust angle 1 for body angle
          pwm.setPWM(i * 3, 0, map(angles.q1, 0, 180, 150, 600));  // hip servo
          pwm.setPWM(i * 3 + 1, 0, map(angles.q2, 0, 180, 150, 600));  // upper leg servo
          pwm.setPWM(i * 3 + 2, 0, map(angles.q3, 0, 180, 150, 600));  // lower leg servo
        }
        t += 10;  // increment time by 10 ms
        break;
      case 'b':  // move backward
        for (int i = 0; i < 4; i++) {
          float x = -L / 2 + i * L / 2;                          // x position
          float y = H * sin(2 * M_PI * (t - T) / T);              // y position
          float z = L1 + H * cos(2 * M_PI * (t - T) / T) - H;     // z position
          JointAngles angles = IK(x, y, z);                       // compute joint angles
          angles.q1 += A;                                         // adjust angle 1 for body
          pwm.setPWM(i * 3, 0, map(angles.q1, 0, 180, 150, 600)); // hip servo
          pwm.setPWM(i * 3 + 1, 0, map(angles.q2, 0, 180, 150, 600)); // upper leg servo
          pwm.setPWM(i * 3 + 2, 0, map(angles.q3, 0, 180, 150, 600)); // lower leg servo
          }
          t -= 10; // decrement time by 10 ms
          break;
          case 'l': // move left
          for (int i = 0; i < 4; i++) {
          float x = -L / 2 + i * L / 2 - H * sin(2 * M_PI * t / T); // x position
          float y = 0; // y position
          float z = L1 + H * cos(2 * M_PI * t / T) - H; // z position
          JointAngles angles = IK(x, y, z); // compute joint angles
          angles.q1 += A; // adjust angle 1 for body angle
          pwm.setPWM(i * 3, 0, map(angles.q1, 0, 180, 150, 600)); // hip servo
          pwm.setPWM(i * 3 + 1, 0, map(angles.q2, 0, 180, 150, 600)); // upper leg servo
          pwm.setPWM(i * 3 + 2, 0, map(angles.q3, 0, 180, 150, 600)); // lower leg servo
          }
          t += 10; // increment time by 10 ms
          break;
          case 'r': // move right
          for (int i = 0; i < 4; i++) {
          float x = -L / 2 + i * L / 2 + H * sin(2 * M_PI * t / T); // x position
          float y = 0; // y position
          float z = L1 + H * cos(2 * M_PI * t / T) - H; // z position
          JointAngles angles = IK(x, y, z); // compute joint angles
          angles.q1 += A; // adjust angle 1 for body angle
          pwm.setPWM(i * 3, 0, map(angles.q1, 0, 180, 150, 600)); // hip servo
          pwm.setPWM(i * 3 + 1, 0, map(angles.q2, 0, 180, 150, 600)); // upper leg servo
          pwm.setPWM(i * 3 + 2, 0, map(angles.q3, 0, 180, 150, 600)); // lower leg servo
          }
          t += 10; // increment time by 10 ms
          break;
          case 'j': // jump
          for (int i = 0; i < 4; i++) {
          float x = -L / 2 + i * L / 2; // x position
          float y = H * sin(2 * M_PI * t / T) + H/2; // y position
          float z = L1 + H * cos(2 * M_PI * t / T) - H; // z      JointAngles angles = IK(x, y, z);                      // compute joint angles
      angles.q1 += A;                                        // adjust angle 1 for body angle
      pwm.setPWM(i * 3, 0, map(angles.q1, 0, 180, 150, 600)); // hip servo
      pwm.setPWM(i * 3 + 1, 0, map(angles.q2, 0, 180, 150, 600));  // upper leg servo
      pwm.setPWM(i * 3 + 2, 0, map(angles.q3, 0, 180, 150, 600));  // lower leg servo
    }
    t += 10;  // increment time by 10 ms
    break;
  case 't':  // trot
    for (int i = 0; i < 4; i++) {
      float x = -L / 2 + i * L / 2;                          // x position
      float y = A * sin(2 * M_PI * t / T);                    // y position
      float z = L1 + H * cos(2 * M_PI * t / T) - H;           // z position
      JointAngles angles = IK(x, y, z);                      // compute joint angles
      angles.q1 += A * cos(2 * M_PI * t / T);                 // adjust angle 1 for body angle
      pwm.setPWM(i * 3, 0, map(angles.q1, 0, 180, 150, 600)); // hip servo
      pwm.setPWM(i * 3 + 1, 0, map(angles.q2, 0, 180, 150, 600));  // upper leg servo
      pwm.setPWM(i * 3 + 2, 0, map(angles.q3, 0, 180, 150, 600));  // lower leg servo
    }
    t += 10;  // increment time by 10 ms
    break;
  case 's':  // stop
    for (int i = 0; i < 4; i++) {
      JointAngles angles = {90, 90, 90};                     // set joint angles to default position
      pwm.setPWM(i * 3, 0, map(angles.q1, 0, 180, 150, 600)); // hip servo
      pwm.setPWM(i * 3 + 1, 0, map(angles.q2, 0, 180, 150, 600));  // upper leg servo
      pwm.setPWM(i * 3 + 2, 0, map(angles.q3, 0, 180, 150, 600));  // lower leg servo
    }
    break;
}
}
}
