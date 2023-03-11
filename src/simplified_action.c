#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Lengths of the three leg segments and the distance between legs
struct LegDimensions {
  float L1;   // length of hip joint
  float L2;   // length of upper leg joint
  float L3;   // length of lower leg joint
  float L;    // distance between legs
};

// Parameters for walking gait
struct WalkParameters {
  float H;    // height of step
  float T;    // period of step
  float A;    // body angle
};

// Joint angles for a given position in space
struct JointAngles {
  float q1;
  float q2;
  float q3;
};

// Initialize the servo driver
void initializeServos(Adafruit_PWMServoDriver& pwm) {
  pwm.begin();
  pwm.setPWMFreq(60);
}

// Convert joint angles to servo PWM values
void setServos(Adafruit_PWMServoDriver& pwm, int i, JointAngles angles) {
  pwm.setPWM(i * 3, 0, map(angles.q1, 0, 180, 150, 600));     // hip servo
  pwm.setPWM(i * 3 + 1, 0, map(angles.q2, 0, 180, 150, 600)); // upper leg servo
  pwm.setPWM(i * 3 + 2, 0, map(angles.q3, 0, 180, 150, 600)); // lower leg servo
}

// Compute joint angles for a given position in space
JointAngles computeJointAngles(float x, float y, float z, LegDimensions dims) {
  JointAngles angles;
  float r = sqrt(x * x + y * y);
  float D = (z - dims.L1) * (z - dims.L1) + r * r;
  angles.q1 = atan2(y, x) * 180 / M_PI;  // angle 1
  angles.q2 = -atan2(z - dims.L1, r) * 180 / M_PI;  // angle 2
  angles.q3 = acos((D - dims.L2 * dims.L2 - dims.L3 * dims.L3) / (2 * dims.L2 * dims.L3)) * 180 / M_PI - 90 - angles.q2;  // angle 3
  return angles;
}

// Move the robot forward by one step
void moveForward(Adafruit_PWMServoDriver& pwm, float t, LegDimensions dims, WalkParameters params) {
  for (int i = 0; i < 4; i++) {
    float x = -dims.L / 2 + i * dims.L / 2;                      // x position
    float y = params.H * sin(2 * M_PI * t / params.T);           // y position
    float z = dims.L1 + params.H * cos(2 * M_PI * t / params.T) - params.H;  // z position
    JointAngles angles = computeJointAngles(x, y, z, dims);      // compute joint angles
    angles.q1 += params.A;                                       // adjust angle 1 for body angle
    setServos(pwm, i, angles);                                   // set servo PWM values
  }
}

// Move the robot backward by one step
void moveBackward(Adafruit_PWMServoDriver& pwm, float t, LegDimensions dims, WalkParameters params) {
  for (int i = 0; i < 4; i++) {
float x = -dims.L / 2 + i * dims.L / 2; // x position
float y = params.H * sin(2 * M_PI * (t - params.T) / params.T); // y position
float z = dims.L1 + params.H * cos(2 * M_PI * (t - params.T) / params.T) - params.H; // z position
JointAngles angles = computeJointAngles(x, y, z, params.A); // compute joint angles
setServoPulse(pwm, i, angles.q1, angles.q2, angles.q3); // set servo pulse for each joint
}
}

// Move the robot left by one step
void moveLeft(Adafruit_PWMServoDriver& pwm, float t, LegDimensions dims, WalkParameters params) {
for (int i = 0; i < 4; i++) {
float x = -dims.L / 2 + i * dims.L / 2 - params.H * sin(2 * M_PI * t / params.T); // x position
float y = 0; // y position
float z = dims.L1 + params.H * cos(2 * M_PI * t / params.T) - params.H; // z position
JointAngles angles = computeJointAngles(x, y, z, params.A); // compute joint angles
setServoPulse(pwm, i, angles.q1, angles.q2, angles.q3); // set servo pulse for each joint
}
}

// Move the robot right by one step
void moveRight(Adafruit_PWMServoDriver& pwm, float t, LegDimensions dims, WalkParameters params) {
for (int i = 0; i < 4; i++) {
float x = -dims.L / 2 + i * dims.L / 2 + params.H * sin(2 * M_PI * t / params.T); // x position
float y = 0; // y position
float z = dims.L1 + params.H * cos(2 * M_PI * t / params.T) - params.H; // z position
JointAngles angles = computeJointAngles(x, y, z, params.A); // compute joint angles
setServoPulse(pwm, i, angles.q1, angles.q2, angles.q3); // set servo pulse for each joint
}
}

// Make the robot jump
void jump(Adafruit_PWMServoDriver& pwm, float t, LegDimensions dims, WalkParameters params) {
for (int i = 0; i < 4; i++) {
float x = -dims.L / 2 + i * dims.L / 2; // x position
float y = params.H * sin(2 * M_PI * t / params.T) + params.H / 2; // y position
float z = dims.L1 + params.H * cos(2 * M_PI * t / params.T) - params.H; // z position
JointAngles angles = computeJointAngles(x, y, z, params.A); // compute joint angles
setServoPulse(pwm, i, angles.q1, angles.q2, angles.q3); // set servo pulse for each joint
}
}void trot(Adafruit_PWMServoDriver& pwm, float t, LegDimensions dims, WalkParameters params) {
  float x_pos[4];
  float y_pos[4];
  float z_pos[4];

  // Compute the position of each foot for this time step
  for (int i = 0; i < 4; i++) {
    float phase_offset = i * 0.25;
    x_pos[i] = computeXPos(t, phase_offset, params);
    y_pos[i] = computeYPos(t, phase_offset, params);
    z_pos[i] = computeZPos(t, phase_offset, params);
  }

  // Compute the joint angles for each foot
  JointAngles leg_angles[4];
  for (int i = 0; i < 4; i++) {
    leg_angles[i] = computeJointAngles(x_pos[i], y_pos[i], z_pos[i], dims);
  }

  // Set the servo positions for each foot
  for (int i = 0; i < 4; i++) {
    float hip_pos = map(leg_angles[i].q1 + params.body_angle, 0, 180, 150, 600);
    float upper_pos = map(leg_angles[i].q2, 0, 180, 150, 600);
    float lower_pos = map(leg_angles[i].q3, 0, 180, 150, 600);

    int hip_channel = i * 3;
    int upper_channel = i * 3 + 1;
    int lower_channel = i * 3 + 2;

    pwm.setPWM(hip_channel, 0, hip_pos);
    pwm.setPWM(upper_channel, 0, upper_pos);
    pwm.setPWM(lower_channel, 0, lower_pos);
  }
}
void stop(Adafruit_PWMServoDriver& pwm, LegDimensions dims) {
for (int i = 0; i < 4; i++) {
JointAngles angles = {90, 90, 90}; // set joint angles to default position
pwm.setPWM(i * 3, 0, map(angles.q1, 0, 180, dims.minPulse, dims.maxPulse)); // hip servo
pwm.setPWM(i * 3 + 1, 0, map(angles.q2, 0, 180, dims.minPulse, dims.maxPulse)); // upper leg servo
pwm.setPWM(i * 3 + 2, 0, map(angles.q3, 0, 180, dims.minPulse, dims.maxPulse)); // lower leg servo
}
}

void loop() {
if (Serial.available() > 0) {
char cmd = Serial.read();
switch (cmd) {
case 'f': // move forward
moveForward(pwm, t, dims, params);
t += params.stepTime;
break;
case 'b': // move backward
moveBackward(pwm, t, dims, params);
t += params.stepTime;
break;
case 'l': // move left
moveLeft(pwm, t, dims, params);
t += params.stepTime;
break;
case 'r': // move right
moveRight(pwm, t, dims, params);
t += params.stepTime;
break;
case 'j': // jump
jump(pwm, t, dims, params);
t += params.stepTime;
break;
case 't': // trot
trot(pwm, t, dims, params);
t += params.stepTime;
break;
case 's': // stop
stop(pwm, dims);
break;
}
}
}
