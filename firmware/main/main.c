#include "uart_driver.h"
#include <string.h>

static const size_t BUF_SIZE = 128;

void app_main(void) {
  uart_init();

  uint8_t buf[BUF_SIZE];
  size_t pos = 0;

  uart_send((const uint8_t *)"Hello UART!\n", 13);

  while (1) {
    // Read one byte at a time with short timeout
    int len = uart_recv(&buf[pos], 1, 10 / portTICK_PERIOD_MS);
    if (len > 0) {
      uint8_t c = buf[pos];

      // If newline, echo message
      if (c == '\r' || c == '\n') {
        buf[pos] = '\0'; // null-terminate the string

        // Send the echoed line on a NEW line
        uart_send(buf, pos);                 // the echoed line
        uart_send((const uint8_t *)"\n", 2); // newline after for readability

        pos = 0; // reset buffer
      } else {
        pos++;
        if (pos >= BUF_SIZE - 1)
          pos = 0; // prevent overflow
      }
    }
  }
}
