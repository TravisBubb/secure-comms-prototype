#ifndef RADIO_FRAME_H
#define RADIO_FRAME_H

#include <stdint.h>
#include <stddef.h>

#define MAX_FRAME_PAYLOAD_LEN 200

struct Frame
{
  uint8_t ver;
  uint8_t flags;
  uint16_t dev_id;
  uint32_t seq;
  uint8_t payloadLen;
  uint8_t payload[MAX_FRAME_PAYLOAD_LEN];
  uint8_t auth_tag[16];
  uint16_t crc;
};

/**
 * @brief Serialize the Frame into a byte buffer.
 * 
 * @return true on success. 'len' will be set to the number of bytes written.
 */
bool frame_serialize(const Frame &frame, uint8_t *out, size_t *len);

/**
 * @brief Deserialize a byte buffer into a Frame.
 * 
 * @return true on success.
 */
bool frame_deserialize(Frame &frame, const uint8_t *data, size_t len);

/**
 * @brief Reset all fields in the Frame to defaults.
 */
void frame_reset(Frame &frame);

#endif // RADIO_FRAME_H