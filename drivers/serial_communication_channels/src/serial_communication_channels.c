#include "serial_communication_channels/serial_communication_channels.h"
#include <stdlib.h>

// Local defines
#define CHANNELS_SIZE 'z'-'f'+'Z'-'F'

char ChannelDataToSendBuffer_[COMMUNICATION_CHANNELS_MAX_SIZE+1+1];  //one for channel identification and another for /0
unsigned char ChannelsSize_[CHANNELS_SIZE]={0};
unsigned char** ChannelsValuesAddrTable2_[CHANNELS_SIZE];

void FreeChannelsMemory()
{
  unsigned char i;

  for(i=0; i<(CHANNELS_SIZE); ++i){
    free(ChannelsValuesAddrTable2_[i]);
  }
}

char CheckChannel(char data)
{
  if (((data >= 'g') && (data <= 'z')) || ((data >= 'G') && (data <= 'Z'))){
    return 1;
  }
  else{
    return 0;
  }
}

char HexNibbleToNum(char c)
{
  if ((c>='0') && (c<='9')){
    return (c-'0');
  }
  else if((c>='A') && (c<='F')){
    return (c-'A'+10);
  }
  else{
    return -1;
  }
}

int ChannelCharToChannelNum(char channel){
  if ((channel >= 'g') && (channel <= 'z')){
    return channel-'g';
  }
  else if ((channel >= 'G') && (channel <= 'Z')){
    return channel-'G'+'z'-'f';
  }
  else{
    return -1;
  }
}

char ChannelNumToChannelChar(int num){
  if ((num >=0) && (num < 'z'-'f')){
    return num+'g';
  }
  else if (num < 'Z'-'F'+'z'-'f'){
    return num+'G'-('z'-'f');
  }
  else{
    return '\0';
  }
}

char ProcessChannelsSerialData(char data)
{
  static enum TComState{IDLE=0, WAITING_START, WAITING_MS_NIBBLE, WAITING_LS_NIBBLE}comState=IDLE;
  static unsigned char **pchannelDataIt;
  static unsigned char **pchannelEnd;
  static unsigned char actualChannel;
  static char dataBuffer;
  char nibble;

  switch (comState) {
    case IDLE:
      if(CheckChannel(data)){
        actualChannel=ChannelCharToChannelNum(data);
        if(ChannelsSize_[actualChannel]>0){
          comState=WAITING_MS_NIBBLE;
          pchannelDataIt=ChannelsValuesAddrTable2_[actualChannel];
          pchannelEnd=pchannelDataIt+ChannelsSize_[actualChannel];
          return 0;
        }
        else{
          comState=WAITING_START;
          return (ChannelNumToChannelChar(actualChannel));
        }
      }
      else
      {
        return 0;
      }
    case WAITING_START:
      if(CheckChannel(data)){
        actualChannel=ChannelCharToChannelNum(data);
        if(ChannelsSize_[actualChannel]>0){
          comState=WAITING_MS_NIBBLE;
          pchannelDataIt=ChannelsValuesAddrTable2_[actualChannel];
          pchannelEnd=pchannelDataIt+ChannelsSize_[actualChannel];
          return 0;
        }
        else{
          return (ChannelNumToChannelChar(actualChannel));
        }
      }
      else
      {
        comState=IDLE;
        return -1;
      }
    case WAITING_MS_NIBBLE:
      nibble=HexNibbleToNum(data);
      if(nibble>=0){
        dataBuffer=nibble<<4;
        comState=WAITING_LS_NIBBLE;
        return 0;
      }
      else{
        if(CheckChannel(data)){
          actualChannel=ChannelCharToChannelNum(data);
          if(ChannelsSize_[actualChannel]>0){
            comState=WAITING_MS_NIBBLE;
            pchannelDataIt=ChannelsValuesAddrTable2_[actualChannel];
            pchannelEnd=pchannelDataIt+ChannelsSize_[actualChannel];
          }
          else{
            comState=IDLE;
          }
        }
        else{
          comState=IDLE;
        }
        return -2;
      }
    case WAITING_LS_NIBBLE:
      nibble=HexNibbleToNum(data);
      if(nibble>=0){
        dataBuffer=dataBuffer | nibble;
        **pchannelDataIt=dataBuffer;
        ++pchannelDataIt;
        if(pchannelDataIt>=pchannelEnd){
          comState=WAITING_START;
          return (ChannelNumToChannelChar(actualChannel));
        }
        else{
          comState=WAITING_MS_NIBBLE;
          return 0;
        }
      }
      else{
        if(CheckChannel(data)){
          actualChannel=ChannelCharToChannelNum(data);
          if(ChannelsSize_[actualChannel]>0){
            comState=WAITING_MS_NIBBLE;
            pchannelDataIt=ChannelsValuesAddrTable2_[actualChannel];
            pchannelEnd=pchannelDataIt+ChannelsSize_[actualChannel];
          }
          else{
            comState=IDLE;
          }
        }
        return -3;
      }
    default:
      comState=IDLE;
      return -4;
  }
}

char NumToHexNibble(uint8_t num)
{
  if (num<10){
    return (num+'0');
  }
  else{
    return (num+'A'-10);
  }
}

char* NumToHexNibbles(unsigned char num, char *nibbles_addr)
{
  *nibbles_addr=NumToHexNibble(num>>4);
  ++nibbles_addr;
  *nibbles_addr=NumToHexNibble(num&0x0F);
  ++nibbles_addr;
  return nibbles_addr;
}

const char *SendChannel(char channel)
{
  char *dataToSendIt=ChannelDataToSendBuffer_;
  unsigned char **pchannelDataIt;
  unsigned char **pchannelEnd;
  unsigned char channelAux;

  *dataToSendIt=channel;
  ++dataToSendIt;
  channelAux=ChannelCharToChannelNum(channel);
  pchannelDataIt=ChannelsValuesAddrTable2_[channelAux];
  pchannelEnd=pchannelDataIt+ChannelsSize_[channelAux];

  for(; pchannelDataIt<pchannelEnd; ++pchannelDataIt){
    dataToSendIt=NumToHexNibbles(**pchannelDataIt, dataToSendIt);
  }
  *dataToSendIt='\0';

  return ChannelDataToSendBuffer_;
}

void AddAddrsToChannelsValuesAddrTable(char channel, unsigned char* addr, unsigned char size)
{
  uint16_t sample=0xff00; //for bigEndian Test
  unsigned char *psample;
  unsigned char channelAux;
  unsigned char i;

  if(CheckChannel(channel)){
    channelAux=ChannelCharToChannelNum(channel);

    //allocate memmory
    if(ChannelsSize_[channelAux]<=COMMUNICATION_CHANNELS_MAX_SIZE){
      ChannelsValuesAddrTable2_[channelAux]=(unsigned char**)realloc(ChannelsValuesAddrTable2_[channelAux], (ChannelsSize_[channelAux]+size)*sizeof(unsigned char*));
      if(ChannelsValuesAddrTable2_[channelAux]==NULL){
          exit(EXIT_FAILURE);
      }
    }
    else{
      exit(EXIT_FAILURE);
    }

    psample=(unsigned char *)&sample;
    for(i=0; i<size; ++i){
      if(*psample){ //bigEndianTest
          (ChannelsValuesAddrTable2_[channelAux])[ChannelsSize_[channelAux]]=addr+i;
      }
      else{
          (ChannelsValuesAddrTable2_[channelAux])[ChannelsSize_[channelAux]]=addr+size-1-i;
      }
      ++ChannelsSize_[channelAux];
    }
  }
  else{
    exit(EXIT_FAILURE);
  }
}

void AddValueToChannel_int8_t(char channel, int8_t *value_addr)
{
  AddAddrsToChannelsValuesAddrTable(channel, (unsigned char*) value_addr, sizeof(int8_t));
}

void AddValueToChannel_int16_t(char channel, int16_t *value_addr)
{
  AddAddrsToChannelsValuesAddrTable(channel, (unsigned char*) value_addr, sizeof(int16_t));
}

void AddValueToChannel_uint16_t(char channel, uint16_t *value_addr)
{
  AddAddrsToChannelsValuesAddrTable(channel, (unsigned char*) value_addr, sizeof(uint16_t));
}

void AddValueToChannel_uint8_t(char channel, uint8_t *value_addr)
{
  AddAddrsToChannelsValuesAddrTable(channel, (unsigned char*) value_addr, sizeof(uint8_t));
}

void AddValueToChannel_int32_t(char channel, int32_t *value_addr)
{
  AddAddrsToChannelsValuesAddrTable(channel, (unsigned char*) value_addr, sizeof(int32_t));
}

void AddValueToChannel_uint32_t(char channel, uint32_t *value_addr)
{
  AddAddrsToChannelsValuesAddrTable(channel, (unsigned char*) value_addr, sizeof(uint32_t));
}

void AddValueToChannel_int64_t(char channel, int64_t *value_addr)
{
  AddAddrsToChannelsValuesAddrTable(channel, (unsigned char*) value_addr, sizeof(int64_t));
}

void AddValueToChannel_float(char channel, float *value_addr)
{
  AddAddrsToChannelsValuesAddrTable(channel, (unsigned char*) value_addr, sizeof(float));
}
