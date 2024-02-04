#include "sdpo_ratf_ros_driver/SerialChannelsConfig.h"

SerialChannelsConfig serial_channels_config_;

SerialChannelsConfig* InitCommunications() {
  AddValueToChannel_int32_t('g', &serial_channels_config_.channel_g);
  AddValueToChannel_int32_t('h', &serial_channels_config_.channel_h);
  AddValueToChannel_int32_t('i', &serial_channels_config_.channel_i);
  AddValueToChannel_int32_t('j', &serial_channels_config_.channel_j);
  AddValueToChannel_int32_t('k', &serial_channels_config_.channel_k);
  AddValueToChannel_int32_t('s', &serial_channels_config_.channel_s);

  AddValueToChannel_float('G', &serial_channels_config_.channel_G);
  AddValueToChannel_float('H', &serial_channels_config_.channel_H);
  AddValueToChannel_float('I', &serial_channels_config_.channel_I);
  AddValueToChannel_float('J', &serial_channels_config_.channel_J);
  AddValueToChannel_int32_t('K', &serial_channels_config_.channel_K);
  AddValueToChannel_int32_t('L', &serial_channels_config_.channel_L);

  return &serial_channels_config_;
}
