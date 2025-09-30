#ifndef UART_TRANSPORT_H
#define UART_TRANSPORT_H

#include <driver/uart.h>
#include <stddef.h>

/**
 * @brief Initialize UART transport with specified UART port and baud rate.
 *
 * @param uart UART port number (e.g., UART_NUM_0, UART_NUM_1, etc.)
 * @param baud_rate Communication speed in bits per second.
 */
void uart_transport_init(uart_port_t uart, int baud_rate);

/**
 * @brief Read data from UART transport.
 *
 * @param uart UART port number.
 * @param buf Buffer to store received data.
 * @param max_len Maximum length of data to read.
 * @param timeout Timeout in ticks to wait for data.
 *
 * @return Number of bytes read from UART transport.
 */
size_t uart_transport_read(uart_port_t uart, uint8_t *buf, size_t max_len, TickType_t timeout);

/**
 * @brief Write data to UART transport.
 *
 * @param uart UART port number.
 * @param buf Buffer containing data to send.
 * @param len Length of data to send.
 */
void uart_transport_write(uart_port_t uart, const uint8_t *buf, size_t len);

#endif // UART_TRANSPORT_H