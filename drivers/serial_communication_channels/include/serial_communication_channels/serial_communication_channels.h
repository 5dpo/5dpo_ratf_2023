#ifndef SRC_SERIAL_COMMUNICATION_CHANNELS_H
#define SRC_SERIAL_COMMUNICATION_CHANNELS_H

#define COMMUNICATION_CHANNELS_MAX_SIZE 253

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

void FreeChannelsMemory();
const char* SendChannel(char channel);
char ProcessChannelsSerialData(char data);

void AddValueToChannel_int8_t(char channel, int8_t *value_addr);
void AddValueToChannel_uint8_t(char channel, uint8_t *value_addr);
void AddValueToChannel_int16_t(char channel, int16_t *value_addr);
void AddValueToChannel_uint16_t(char channel, uint16_t *value_addr);
void AddValueToChannel_int32_t(char channel, int32_t *value_addr);
void AddValueToChannel_uint32_t(char channel, uint32_t *value_addr);
void AddValueToChannel_int64_t(char channel, int64_t *value_addr);
void AddValueToChannel_float(char channel, float *value_addr);

#ifdef __cplusplus
}
#endif

#endif //SRC_SERIAL_COMMUNICATION_CHANNELS_H
