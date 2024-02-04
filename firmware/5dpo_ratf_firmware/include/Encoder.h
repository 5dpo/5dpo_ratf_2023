#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

void updateEncodersState(void);

class Encoder {
 public:
  uint32_t tick = 0;
  uint32_t tick_last = 0;
  volatile int16_t delta;
  int32_t odo;
  uint8_t state;

 public:
  void updateDelta(uint8_t &new_state);
  void updateTick(void);
};

extern Encoder *encoders;

#endif
