#include "frame.h"
#include "bytes.h"
#include <string.h>

bool frame_serialize(const Frame &frame, uint8_t *out, size_t *len)
{
  if (!out || !len || frame.payloadLen > MAX_FRAME_PAYLOAD_LEN) return false;

  // Header (AAD, 8 bytes)
  out[0] = frame.ver;
  out[1] = frame.flags;
  write_u16_be(frame.dev_id, &out[2]);
  write_u32_be(frame.seq, &out[4]);

  // Payload length (1 byte)
  out[8] = (uint8_t)frame.payloadLen;

  // Payload (N bytes)
  memcpy(&out[9], frame.payload, frame.payloadLen);

  // Auth tag (16 bytes)
  memcpy(&out[frame.payloadLen + 9], frame.authTag, sizeof(frame.authTag));

  // CRC (2 bytes)
  size_t crc_start = frame.payloadLen + 9 + sizeof(frame.authTag);
  write_u16_be(frame.crc, &out[crc_start]);

  *len = 9 + frame.payloadLen + sizeof(frame.authTag) + 2;

  return true;
}

bool frame_deserialize(Frame &frame, const uint8_t *data, size_t len)
{
  if (!data || len == 0) return false;

  frame.payloadLen = data[8];

  size_t expected = 9 + frame.payloadLen + sizeof(frame.authTag) + 2;
  if (len < expected || frame.payloadLen > MAX_FRAME_PAYLOAD_LEN) return false;

  frame_reset(frame);

  // Header (AAD, 8 bytes)
  frame.ver = data[0];
  frame.flags = data[1];
  frame.dev_id = read_u16_be(&data[2]);
  frame.seq = read_u32_be(&data[4]);

  // Payload length (1 byte)
  frame.payloadLen = data[8];

  // Payload (N bytes)
  memcpy(frame.payload, &data[9], frame.payloadLen);

  // Auth tag (16 bytes)
  memcpy(frame.authTag, &data[9 + frame.payloadLen], sizeof(frame.authTag));

  // CRC (2 bytes)
  size_t crc_start = frame.payloadLen + 9 + sizeof(frame.authTag);
  frame.crc = read_u16_be(&data[crc_start]);

  return true;
}

void frame_reset(Frame &frame)
{
  memset(&frame, 0, sizeof(Frame));
}