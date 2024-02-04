#include "Encoder.h"

#pragma GCC optimize ("O3")

const int8_t kEncoderTable[16] =
    {  0,  1, -1,  0,
      -1,  0,  0,  1,
       1,  0,  0, -1,
       0, -1,  1,  0 };

void updateEncodersState(void) {
  uint8_t b, new_state;
  b = PINA;

  // Put encoder 0 channels in the lowest bits
  new_state = b & 0b00000011;
  encoders[0].updateDelta(new_state);

  // Put encoder 1 channels in the lowest bits
  b = (b >> 2);
  new_state = b & 0b00000011;
  encoders[1].updateDelta(new_state);

  // Put encoder 2 channels in the lowest bits
  b = (b >> 2);
  new_state = b & 0b00000011;
  encoders[2].updateDelta(new_state);
  
  // Put encoder 3 channels in the lowest bits
  b = (b >> 2);
  new_state = b & 0b00000011;
  encoders[3].updateDelta(new_state);
}

void Encoder::updateDelta(uint8_t &new_state) {
  delta += kEncoderTable[state | new_state];
  state = new_state << 2;
}

void Encoder::updateTick(void) {
  tick_last = tick;
  cli();
  odo = delta;
  delta = 0;
  sei();
  tick += odo;
}
