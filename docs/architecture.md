# Secure Comms Prototype - Architecture

This document describes the layered architecture fo the UART + LoRa communications stack.

## Overview

the stack is designed with clear separation of concerns:
```plaintext
+-------------------+
| Application Layer |
| (CLI, commands)   |
+-------------------+
|
v
+-------------------+
| LoRa Protocol     |
| - Packet structs  |
| - ACK/retry logic |
| - Routing (future)|
+-------------------+
|
v
+-------------------+
| LoRa Transport    |
| - Framing/parsing |
| - CRC/checksums   |
| - Serialization   |
+-------------------+
|
v
+-------------------+
| LoRa Driver       |
| - SPI register I/O|
| - Radio config    |
| - Send/recv bytes |
+-------------------+
|
v
+-------------------+
| Hardware          |
| SX127x/SX126x     |
+-------------------+
```

## Layer Responsibilities

- **Application Layer**  
  Business logic, commands, and user interaction.

- **LoRa Protocol (`lora_proto`)**  
  Defines structured packets, optional acknowledgements, retries, and routing extensions.

- **LoRa Transport (`lora_transport`)**  
  Frames/unframes byte streams, adds checksums, serializes/deserializes packet structures.

- **LoRa Driver (`lora_driver`)**  
  Direct SPI register control, radio init/config, send/receive raw byte buffers.

## Design Goals

- Clear separation of hardware and logic.
- Testability: higher layers can be tested with mocks.
- Future extensibility: add ACKs, retries, or mesh routing without touching the driver.