# LoRa Transport

The LoRa Transport component provides **framing, deframing, and error checking** for byte streams exchanged with the LoRa driver. 

It sits **above `lora_driver`** (which only handles register access and raw byte TX/RX), and **below `lora_proto`** (which interprets packets with higher-level semantics).

## Responsibilities
- Add and strip **frame boundaries** (e.g., start/end markers, length field, or COBS).
- Compute and verify **checksums/CRC** to detect corrupted frames.
- Ensure the application receives **whole frames**, not partial or interleaved bytes.
- Expose a simple API for sending and receiving **raw framed buffers**.

## Non-Responsibilities
- No knowledge of packet structure (`src`, `dst`, `type`, etc.).
- No retries, acknowledgements, or routing logic.
- No SPI register interaction (that belongs to `lora_driver`).

## Public API
```c
esp_err_t lora_transport_init(void);
esp_err_t lora_transport_send(const uint8_t *data, size_t len);
esp_err_t lora_transport_receive(uint8_t *buf, size_t max_len, size_t *out_len);
```

## Example Usage
```c
// Send raw bytes
const char *msg = "hello";
ESP_ERROR_CHECK(lora_transport_send((const uint8_t *)msg, strlen(msg)));

// Poll for receive
uint8_t rx_buf[256];
size_t rx_len = 0;
if (lora_transport_receive(rx_buf, sizeof(rx_buf), &rx_len) == ESP_OK) {
    printf("Received %u bytes: %.*s\n", (unsigned)rx_len, (int)rx_len, rx_buf);
}
```