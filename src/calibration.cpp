// calibration.cpp
#include "calibration.h"
#define GLIDER false

uint16_t rudderMax   = 0;
uint16_t rudderMin   = 2047;
uint16_t aileronMax  = 0;
uint16_t aileronMin  = 2047;
uint16_t elevatorMax = 0;
uint16_t elevatorMin = 2047;
uint16_t brakeMax    = 0;
uint16_t brakeMin    = 2047;
uint16_t throttleMax  = 0;
uint16_t throttleMin  = 2047; 

void autoCalibrate(
  uint16_t roll,
  uint16_t pitch,
  uint16_t rudder,
  uint16_t throttle,
  uint16_t airbrake,
  Joystick_& joystick
) {
  if (rudder  > rudderMax)   rudderMax   = rudder;
  if (rudder  < rudderMin)   rudderMin   = rudder;

  if (roll    > aileronMax)  aileronMax  = roll;
  if (roll    < aileronMin)  aileronMin  = roll;

  if (pitch   > elevatorMax) elevatorMax = pitch;
  if (pitch   < elevatorMin) elevatorMin = pitch;

  if (throttle > throttleMax)   throttleMax    = throttle;
  if (throttle < throttleMin)   throttleMin    = throttle;

  if (airbrake > brakeMax)   brakeMax    = airbrake;
  if (airbrake < brakeMin)   brakeMin    = airbrake;

  joystick.setXAxisRange(aileronMin, aileronMax);
  joystick.setYAxisRange(elevatorMin, elevatorMax);
  joystick.setRudderRange(rudderMin, rudderMax);
  joystick.setThrottleRange(throttleMin, throttleMax);
  if (GLIDER) joystick.setThrottleRange(brakeMin, brakeMax);
}
