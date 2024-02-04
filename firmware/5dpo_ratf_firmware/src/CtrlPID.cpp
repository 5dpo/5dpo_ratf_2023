#include "CtrlPID.h"

void CtrlPID::update(float new_w) {
  if (active) {
    float e_der, e_sum_tmp;

    // Current angular velocity
    w = new_w;

    // Error
    e_prev = e;
    e = w_ref - w;

    e_sum_tmp = e_sum + e * dt;
    e_der = (e - e_prev) / dt;

    // Remove integration for zero reference
    if (w_ref == 0) {
      e_sum_tmp = 0;
    }

    // PID output
    m = kp * e + ki * e_sum_tmp + kd * e_der + kf * w_ref;

    // Anti windup
    if (((m > m_max) && (e_sum_tmp > 0)) || ((m < -m_max) && (e_sum_tmp < 0))) {
      m = m + ki * (e_sum - e_sum_tmp);
    } else {
      e_sum = e_sum_tmp;
    }

    // Dead-zone compensation
    hammerstein(m);

    // Saturation
    if (m > m_max) {
      m = m_max;
    } else if (m < -m_max) {
      m = -m_max;
    }
  }
}

void CtrlPID::reset(void) {
  e_sum = 0;
  m = 0;
}

void CtrlPID::enable(bool e) {
  if (active != e) {
    active = e;
    reset();
  }
}

void CtrlPID::hammerstein(float &mmot) {
  if (mmot > hamm_vd) {
    mmot = (mmot - hamm_vd) + hamm_v0;
  } else if (mmot <= -hamm_vd) {
    mmot = (mmot + hamm_vd) - hamm_v0;
  } else {
    if (hamm_vd != 0 ) {
      mmot = mmot * hamm_v0 / hamm_vd;
    } else {
      mmot = 0;
    }
  }
}
