#pragma once

#include <serial_communication_channels/serial_communication_channels.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct StructSerialChannelsConfig {
  // Arduino >>> PC
  // - encoders:
  int32_t channel_g;
  int32_t channel_h;
  int32_t channel_i;
  int32_t channel_j;
  // - sample time of the motors
  int32_t channel_k;
  // - box switch
  int32_t channel_s;

  // PC >>> Arduino
  // - motors angular speed reference:
  float channel_G;
  float channel_H;
  float channel_I;
  float channel_J;
  // - PWM:
  int32_t channel_K;
  // - solenoid
  int32_t channel_L;
} SerialChannelsConfig;

SerialChannelsConfig* InitCommunications();

#ifdef __cplusplus
}
#endif
