# Library Structure

## Hierarchy

```
Application
  ↓
SecureLink (Security Layer)
  ↓
DataLink (Reliability + Framing Layer)
  ↓
Radio (Physical Interface Layer)
```

Rules:
 - Each layer in the stack only talks to the one immediately below it.
 - Each layer has a clear data type boundary and contract.

## Layer Responsibilities and Boundaries

### Application Layer

**Interface:** `SecureLink`  
**Input/Output Types:** `Packet` (plaintext payloads only)  

Responsibilities:
 - Business logic
 - Sending/receiving packets
 - Doesn't know anything about encryption, CRC, retransmission, etc.

### SecureLink Layer

**Interface:** `SecureLink::send(const Packet&)`, `SecureLink::receive(Packet&)`  
**Input:** Plaintext `Packet`  
**Output (to DataLink):** Authenticated + encrypted `Frame`

Responsibilities:
 - AES-GCM encryption/decryption
 - Authentication tag generation/verification
 - Packet -> Frame and Frame -> Packet conversion
 - Sequence numbers and device IDs (for their tie to the crypto)

Does **not** touch CRC, retransmission, fragmentation, or low-level radio access. It simply handles *confidentiality* and *integrity*.

### DataLink Layer

**Interface:** `DataLink::send(const Frame&)`, `DataLink::receive(Frame&)`  
**Input:** Fully constructed (and encrypted) `Frame`  
**Output:** Raw bytes to Radio  

Responsibilities:
 - Frame serialization/deserialization to/from byte arrays
 - CRC generation/validation
 - Fragmentation and reassembly
 - Retransmission and ACK handling
 - Error detection

Does **not** know or care what is in the encrypted payload. It treats the Frame as an opaque blob of bytes.

### Radio Layer

**Interface:** `Radio::transmit(const uint8_t *data, size_t len)`, `Radio::receive(uint8_t *out, size_t *len)`  
**Input/Output:** Byte buffers  

Responsibilities:
 - Hardware initialization (pins, SPI, etc.)
 - Interfacing with `RadioLib`
 - Low-level transmit/receive operations

No packet structure knowledge.