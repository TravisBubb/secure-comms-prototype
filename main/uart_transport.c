#include "uart_transport.h"

void uart_transport_init(uart_port_t uart, int baud_rate)
{
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122,
    };

    ESP_ERROR_CHECK(uart_param_config(uart, &uart_config));
}

size_t uart_transport_read(uart_port_t uart, uint8_t *buf, size_t max_len, TickType_t timeout)
{
    size_t len = 0;

    uart_get_buffered_data_len(uart, &len);

    if (len > max_len)
        len = max_len;

    if (len > 0)
    {
        size_t r = uart_read_bytes(uart, buf, len, timeout);
        if (r > 0)
            return r;
    }

    return 0;
}

void uart_transport_write(uart_port_t uart, const uint8_t *buf, size_t len)
{
    uart_write_bytes(uart, (const char *)buf, len);
}