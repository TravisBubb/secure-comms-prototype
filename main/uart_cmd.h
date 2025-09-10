#ifndef UART_CMD_H
#define UART_CMD_H

#include <stdint.h>

/**
 * @brief Represents a particular command type
 */
typedef enum {
  UART_CMD_SEND = 0x01,
  UART_CMD_STATUS = 0x02,
  UART_CMD_KEYLOAD = 0x03,
  UART_CMD_RESET = 0x04
} uart_cmd_t;

/**
 * @brief Represents a UART command packet
 */
typedef struct {
  uint8_t cmd;
  uint8_t len;
  uint8_t payload[256];
} uart_packet_t;

#endif // UART_CMD_H
