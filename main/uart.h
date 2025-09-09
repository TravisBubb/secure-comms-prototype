#ifndef UART_H
#define UART_H

#include <stddef.h>
#include <stdint.h>

#define UART_MAX_PAYLOAD 256
#define UART_START_BYTE 0xAA

/**
 * @brief Represents a certain state for the parsing state machine
 */
typedef enum {
  UART_STATE_WAIT_FOR_START,
  UART_STATE_READ_CMD,
  UART_STATE_READ_LEN,
  UART_STATE_READ_PAYLOAD,
  UART_STATE_READ_CRC
} uart_rx_state;

/**
 * @brief Represents possible return codes when reading data over UART
 */
typedef enum {
  UART_RX_ERROR = -1,
  UART_RX_IN_PROGRESS = 0,
  UART_RX_PACKET_COMPLETE = 1
} uart_rx_rc;

/**
 * @brief Represents a particular command type
 */
typedef enum {
  CMD_SEND = 0x01,
  CMD_STATUS = 0x02,
  CMD_KEYLOAD = 0x03,
  CMD_RESET = 0x04
} uart_cmd_t;

/**
 * @brief Represents a UART packet parser
 */
typedef struct {
  uart_rx_state state;
  uart_cmd_t cmd;
  size_t length;
  uint8_t buffer[UART_MAX_PAYLOAD];
  size_t index;
  uint16_t crc_recvd; // CRC from the packet
  uint16_t crc_index; // How many CRC bytes received so far
  uint16_t crc_calc;  // Incremental CRC as bytes are read
} uart_parser_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize/reset a parser
 */
void uart_parser_init(uart_parser_t *parser);

/**
 * @brief Feed a single byte into the parser
 */
uart_rx_rc uart_rx_handle(uart_parser_t *parser, uint8_t b);

#ifdef __cplusplus
}
#endif

#endif // UART_H
