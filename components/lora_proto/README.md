# LoRa Protocol

The LoRa Protocol component defines the **packet format, reliability mechanisms, and higher-level semantics** on top of framed byte delivery provided by `lora_transport`. 

It is the **application-facing layer** of the LoRa stack.

## Responsibilities
- Define a standard `lora_packet_t` structure, including fields such as:
    - Source / destination IDs
    - Packet type (message, ACK, command, etc.)
    - Sequence number
    - Payload
- Serialize / deserialize packets to and from byte buffers.
- Implement reliability features:
    - Acknowledgements (ACK/NACK)
    - Retransmission on timeout
- Provide hooks for future features:
    - Mesh routing
    - Broadcast and multicast
    - Different packet types for telemetry v.s. commands

## Non-Responsibilities
- No SPI register interaction (belongs to `lora_driver`).
- No byte-level framing or CRC checks (belongs to `lora_transport`).

## Public API
```c
// Initialize protocol layer
esp_err_t lora_proto_init(void);

// Send with reliability (handles ACKs/retries internally)
esp_err_t lora_proto_send(const lora_packet_t *packet);

// Receive and validate a packet
esp_err_t lora_proto_receive(lora_packet_t *packet);
```

## Example Usage
```c
lora_proto_init();

lora_packet_t pkt = {
    .type = PACKET_DATA,
    .length = strlen("ping"),
    .payload = (uint8_t *)"ping"
};

lora_proto_send(&pkt);

lora_packet_t recv;
if (lora_proto_receive(&recv) == ESP_OK) {
    printf("Received: %.*s\n", recv.length, recv.payload);
}
```

## Future Expansion
This layer is where features like **routing, retries, and advanced reliability schemes** can be added without touching the transport or driver layers.