# UART Driver Design - Secure Comms Prototype

## Overview

The UART driver provides a simple interface for bidirectional communication with the Heltec V3 development board. It is designed to support a **command-line interface (CLI)** for prototyping secure communications, with future plans for LoRa integration, encryption, and a host CLI in Rust.

Currently, UART0 is used for CLI access via the Heltec V3's onboard USB bridge. This decision simplifies initial prototyping and testing without the need for an external USB-to-UART hardware adapter. UART1 may be adopted in future iterations to dedicate a separate channel for CLI and debugging, leaving UART0 open purely for logging.

---

## Table of Contents

1. [UART Selection](#uart-selection)
2. [Driver Architecture](#driver-architecture)
3. [Buffering and Reads](#buffering-and-reads)

---

## UART Selection

### UART0 (current)

- Why used:
    - On the Heltec V3 board, UART0 is connected to the built-in USB-to-UART bridge, allowing easy communication with a host PC using `screen` or `idf.py monitor`.
    - Requires no additional hardware for prototyping.
- Pros:
    - Immediate testing and development.
    - Visible in terminal tools without adapters.
- Cons:
    - Shared with ESP-IDF logging, which can occassionally interfere with communication output.

### UART1/UART2 (future)

- Why consider:
    - Provides a dedicated channel for CLI or secure messaging.
    - Allows UART0 to remain connected to ESP logs.
- Requirements:
    - USB-to-UART adapter or breakout board.
    - Update `uart_set_pin()` to select the correct GPIO pins.

---

## Driver Architecture

### `uart_driver.h`

Defines a minimal interface:

```c
void uart_init(void);
void uart_send(const uint8_t *data, size_t len);
void uart_recv(uint8_t *buf, size_t maxlen, TickType_t timeout);
```

### `uart_driver.c`

- Initializes UART0 with standard configuration:
Parameter | Value
-----------------
Baud rate | 115200
Data bits | 8
Parity | None
Stop bits | 1
Flow control | None
Pins | Default (USB bridge)
- Installs the UART driver with a receive buffer of `2 x BUF_SIZE` to accomodate bursts of incoming data.
- `uart_send` wraps `uart_write_bytes` for transmitting raw data.
- `uart_recv` wraps `uart_read_bytes`, allowing non-blocking reads with a FreeRTOS timeout.

---

## Buffering and Reads

- Input is read into a buffer of size `BUF_SIZE`.
- `uart_recv()` uses a timeout in FreeRTOS tickets, which means:
    - The calling task blocks for up to the timeout while waiting for bytes
    - If data arrives sooner, the function returns immediately.
    - If no data arrives, the function returns 0 after the timeout expires.
- This provides **bounded blocking**: the task will not be stalled indefinitely, but it is not fully non-blocking.
- Other FreeRTOS tasks can still run because the scheduler will switch to them while this task is waiting.
- Buffer overflow is prevented by wrapping around additional bytes.
- For fully asynchronous behavior, one could use interrupts or event-driven callbacks, but for v0.1 this timeout-based approach is sufficient.
