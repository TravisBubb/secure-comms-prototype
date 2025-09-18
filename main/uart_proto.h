#ifndef UART_PROTO_H
#define UART_PROTO_H

#include "packet.h"
#include <stddef.h>
#include <stdint.h>

#define UART_MAX_PAYLOAD 256
#define UART_START_BYTE 0xAA

/**
 * @brief Represents a certain state for the parsing state machine
 */
typedef enum {
  UART_STATE_WAIT_FOR_START = 0,
  UART_STATE_READ_CMD,
  UART_STATE_READ_LEN,
  UART_STATE_READ_PAYLOAD,
  UART_STATE_READ_CRC
} uart_proto_state_t;

/**
 * @brief Represents possible return codes when reading data over UART
 */
typedef enum {
  UART_RX_ERROR = -1,
  UART_RX_IN_PROGRESS = 0,
  UART_RX_PACKET_COMPLETE = 1
} uart_proto_result_t;

/**
 * @brief Represents a UART packet parser
 */
typedef struct {
  uart_proto_state_t state;

  uint8_t cmd;
  size_t length;
  uint8_t buffer[UART_MAX_PAYLOAD];
  size_t index;

  uint16_t crc_recvd; // CRC from the packet
  uint16_t crc_index; // How many CRC bytes received so far
  uint16_t crc_calc;  // Incremental CRC as bytes are read
} uart_proto_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize/reset a parser
 */
void uart_proto_init(uart_proto_t *p);

/**
 * @brief Feed a single byte into the parser
 */
uart_proto_result_t uart_proto_feed(uart_proto_t *p, uint8_t b);

/**
 * @brief Map a uart_proto_t to a packet_t
 */
packet_t uart_proto_to_packet(const uart_proto_t *p);

#ifdef __cplusplus
}
#endif

#endif // UART_PROTO_H
