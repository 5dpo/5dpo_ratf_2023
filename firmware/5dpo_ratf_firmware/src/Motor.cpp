#include "Motor.h"

void Motor::init(Adafruit_DCMotor* mot_ptr) {
  ptr = mot_ptr;

  enable = true;
  pwm = 0;
  setPWM(0);
}

void Motor::setPWM(int16_t new_pwm) {
  // Saturation
  if (new_pwm > kMotPWMmax) {
    new_pwm = kMotPWMmax;
  } else if (new_pwm < -kMotPWMmax) {
    new_pwm = -kMotPWMmax;
  }

  // Reset if disabled
  if (!enable) {
    new_pwm = 0;
  }

  // Limit PWM change
  if (kMotPWMDeltaMaxEnabled) {
    if (new_pwm - pwm > kMotPWMDeltaMax) {
      new_pwm = pwm + kMotPWMDeltaMax;
    } else if (new_pwm - pwm < -kMotPWMDeltaMax) {
      new_pwm = pwm - kMotPWMDeltaMax;
    }
  }

  // Set pwm
  if (enable) {
    if (new_pwm >= 0) {
      ptr->setSpeed((uint8_t) new_pwm);
      ptr->run(FORWARD);

    } else {
      ptr->setSpeed((uint8_t) (-new_pwm));
      ptr->run(BACKWARD);
    }

  } else {
    ptr->run(RELEASE);
  }

  // Save pwm value
  pwm = new_pwm;
}
