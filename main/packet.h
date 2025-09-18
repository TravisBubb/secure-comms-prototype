#ifndef PACKET_H
#define PACKET_H

#include <stddef.h>
#include <stdint.h>

#define PAYLOAD_MAX 256

typedef struct {
  uint8_t cmd;
  size_t len;
  uint8_t payload[PAYLOAD_MAX];
} packet_t;

#endif // PACKET_H
