#ifndef TYPES_H
#define TYPES_h

#include <stdint.h>

#define MAX_PACKET_PAYLOAD 1024

struct Packet 
{
  uint16_t destId;
  uint8_t payload[MAX_PACKET_PAYLOAD];
  uint8_t payloadLen;
};

#endif // TYPES_H