#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_MotorShield.h>

#include "robotconfig.h"

class Motor {
 public:
  bool enable;
  Adafruit_DCMotor* ptr;
  int16_t pwm;

 public:
  void init(Adafruit_DCMotor* mot_ptr);
  void setPWM(int16_t new_pwm);
};

#endif
