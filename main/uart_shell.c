#include "uart_shell.h"
#include "uart_transport.h"
#include <driver/uart.h>
#include <esp_log.h>
#include <string.h>

#define UART_BUFFER_SIZE 128
#define SHELL_MAX_COMMANDS 10

typedef struct
{
  const char *name;
  void (*handler)(void);
} shell_command_t;

static shell_command_t commands[SHELL_MAX_COMMANDS];
static int command_count = 0;
static uart_port_t shell_uart = UART_NUM_0;
static const char *TAG = "UART_SHELL";
static const int baud_rate = 115200;

static void handle_command(uint8_t *buf, size_t len);

void uart_shell_init(uart_port_t uart)
{
  shell_uart = uart;
  uart_transport_init(shell_uart, baud_rate);
  ESP_LOGI(TAG, "UART shell initialized on UART%d at %d baud.", shell_uart, baud_rate);
}

void uart_shell_task(void *arg)
{
  uint8_t buf[UART_BUFFER_SIZE];
  while (1)
  {
    size_t len =
        uart_transport_read(shell_uart, buf, UART_BUFFER_SIZE - 1, 10 / portTICK_PERIOD_MS);
    if (len > 0)
      handle_command(buf, len);

    vTaskDelay(10 / portTICK_PERIOD_MS); // yield
  }
}

int register_shell_command(const char *name, void (*handler)(void))
{
  if (command_count >= SHELL_MAX_COMMANDS)
    return -1;

  commands[command_count++] = (shell_command_t){.name = name, .handler = handler};

  return 0;
}

static void handle_command(uint8_t *buf, size_t len)
{
  buf[len] = '\0';                       // null terminate
  buf[strcspn((char *)buf, "\r\n")] = 0; // remove newline characters

  for (size_t i = 0; i < command_count; i++)
  {
    if (strcmp((char *)buf, commands[i].name) == 0)
    {
      commands[i].handler();
      return;
    }
  }
}