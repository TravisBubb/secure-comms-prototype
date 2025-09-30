/**
 * @file uart_shell.h
 * @brief Header file for UART shell interface.
 */

#ifndef UART_SHELL_H
#define UART_SHELL_H

#include <driver/uart.h>

/**
 * @brief Initialize the UART shell.
 *
 * @param uart UART port number (e.g., UART_NUM_0, UART_NUM_1, etc.)
 */
void uart_shell_init(uart_port_t uart);

/**
 * @brief UART shell task function.
 *
 * @param arg Pointer to task arguments (if any).
 */
void uart_shell_task(void *arg);

/**
 * @brief Register a command with the UART shell.
 *
 * @param name Name of the command.
 * @param handler Function pointer to the command handler.
 *
 * @return 0 on success, -1 if the command list is full.
 */
int register_shell_command(const char *name, void (*handler)(void));

#endif // UART_SHELL_H