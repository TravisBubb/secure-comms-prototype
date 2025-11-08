#ifndef BYTES_H
#define BYTES_H

#include <stdint.h>

/**
 * @brief Write a uint16_t to the out buffer as big endian
 */
static inline void write_u16_be(uint16_t val, uint8_t *out)
{
  out[0] = (val >> 8) & 0xFF;
  out[1] = val & 0xFF;
}

/**
 * @brief Write a uint32_t to the out buffer as big endian
 */
static inline void write_u32_be(uint32_t val, uint8_t *out)
{
  out[0] = (val >> 24) & 0xFF;
  out[1] = (val >> 16) & 0xFF;
  out[2] = (val >> 8) & 0xFF;
  out[3] = val & 0xFF;
}

/**
 * @brief Read a uint16_t from the in buffer as big endian
 */
static inline uint16_t read_u16_be(const uint8_t *in)
{
  if (!in) return 0;
  return ((uint16_t)in[0] << 8) | in[1];
}

/**
 * @brief Read a uint32_t from the in buffer as big endian
 */
static inline uint32_t read_u32_be(const uint8_t *in)
{
  if (!in) return 0;
  return ((uint32_t)in[0] << 24) | ((uint32_t)in[1] << 16) | ((uint32_t)in[2] << 8) | in[3];
}

#endif // BYTES_H
