#include "uart_driver.h"
#include "driver/uart.h"

#define UART_NUM UART_NUM_0 // Using UART0 for now (USB bridge on Heltec V3)
static const int BUF_SIZE = 1024;

void uart_init(void) {
  uart_config_t uart_config = {.baud_rate = 115200,
                               .data_bits = UART_DATA_8_BITS,
                               .parity = UART_PARITY_DISABLE,
                               .stop_bits = UART_STOP_BITS_1,
                               .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
  uart_param_config(UART_NUM, &uart_config);
  uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
}

void uart_send(const uint8_t *data, size_t len) {
  uart_write_bytes(UART_NUM, (const char *)data, len);
}

int uart_recv(uint8_t *buf, size_t maxlen, TickType_t timeout) {
  return uart_read_bytes(UART_NUM, buf, maxlen, timeout);
}
