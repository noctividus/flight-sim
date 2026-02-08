// calibration.h
#pragma once
#include <Arduino.h>
#include <Joystick.h>

extern uint16_t rudderMax, rudderMin;
extern uint16_t aileronMax, aileronMin;
extern uint16_t elevatorMax, elevatorMin;
extern uint16_t brakeMax, brakeMin;
extern uint16_t throttleMax, throttleMin;

void autoCalibrate(
  uint16_t roll,
  uint16_t pitch,
  uint16_t rudder,
  uint16_t throttle,
  uint16_t airbrake,
  Joystick_& joystick
);
