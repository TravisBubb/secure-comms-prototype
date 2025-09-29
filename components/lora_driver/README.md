# LoRa Driver

The LoRa Driver component provides **low-level access to the LoRa radio chip** (SX126x) using SPI. It is the **hardware abstraction layer** for the LoRa stack.

This component is concerned only with **register configuration** and **raw byte transmission/reception**.

## Responsibilities
- Initialize and configure the LoRa radio via SPI:
    - Frequency
    - Bandwidth
    - Spreading factor
    - Coding rate
    - Transmit power
- Provide functions to send and receive raw byte buffers.
- Expose radio state (e.g., idle, TX, RX).
- Handle low-level IRQs and events (RX done, TX done, timeouts).

## Non-Responsibilities
- No byte framing or checksums (handled by `lora_transport`).
- No packet structures, ACKS, or routing (handled by `lora_proto`).
- No application-specific logic.

## Public API
```c
// Initialize radio and SPI
esp_err_t lora_driver_init(void);

// Set basic parameters
esp_err_t lora_driver_set_frequency(uint32_t hz);
esp_err_t lora_driver_set_tx_power(int level);

// Send and receive raw bytes
esp_err_t lora_driver_send(const uint8_t *data, size_t len);
esp_err_t lora_driver_receive(uint8_t *buffer, size_t max_len, size_t *out_len);
```

## Usage Example
```c
// Initialize the radio
lora_driver_init();

// Configure basic parameters
lora_driver_set_frequency(915e6);
lora_driver_set_tx_power(17);

// Transmit raw bytes
uint8_t raw[] = { 0xDE, 0xAD, 0xBE, 0xEF };
lora_driver_send(raw, sizeof(raw));

// Receive raw bytes (blocking or via callback)
uint8_t buffer[256];
size_t len = 0;
if (lora_driver_receive(buffer, sizeof(buffer), &len)) {
    // buffer contains raw bytes from the radio
}
```

## Notes
- Designed to be a thin abstraction over SPI register access.
- Should work across different LoRa radios with minimal changes.
- Keep this layer as simple as possible; Higher-level features belong in `lora_transport` or `lora_proto`.