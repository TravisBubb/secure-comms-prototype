#pragma once
#include "freertos/FreeRTOS.h"
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Initialize the UART hardware interface
 *
 * Initialize UART0 with standard configuration:
 *  - 115200 baud, 8 data bits, no parity, 1 stop bit
 *  - No flow control
 *  - Pins left as default (USB bridge)
 *
 * Notes:
 *  - UART0 is shared with ESP-IDF logging. Do not use ESP_LOGI while sending
 *    CLI messages to prevent garbled output.
 *  - In the future, move to UART1 for a dedicated channel once a USB-to-UART
 *    bridge is available.
 */
void uart_init(void);

/**
 * @brief Send raw bytes over UART
 * @param data Pointer to buffer to send over UART
 * @param len Size of the buffer to send
 */
void uart_send(const uint8_t *data, size_t len);

/**
 * @brief Receive bytes from UART with timeout (ticks)
 * @param buf Pointer to buffer to store received bytes
 * @param maxlen Maximum bytes to read
 * @param timeout FreeRTOS ticks
 *
 * @returns Number of bytes received
 */
int uart_recv(uint8_t *buf, size_t maxlen, TickType_t timeout);
