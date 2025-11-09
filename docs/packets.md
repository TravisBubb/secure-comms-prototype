# LoRa Packet Framing Protocol

## Design Choices

- AES-GCM for authenticated encryption.
    - Single operation for confidentiality and integrity.
- Pre-shared symmetric key for now (for demo purposes).
    - Add ECDH (X25519) handshake later for key agreement.
- Sequence counter + device ID in nonce to prevent reuse.
- Small authenticated header (AAD).
- Sliding window ACKs (small window) + re-transmit with jitter/backoff.

## Frame Layout

- `Preamble`: implicit at PHY/LoRa level (not included in application-level protocol).
- `Header (AAD, 8 bytes total)`: authenticated but not encrypted.
    - `ver` (1 byte): protocol version (0x01).
    - `flags` (1 byte): bitfield (bit0=ACK required, bit1=fragmented, bits2-7 reserved).
    - `dev_id` (2 bytes): device short id (big-endian).
    - `seq` (4 bytes): unsigned monotonic packet counter (big-endian).
- `PayloadLen` (1 byte): length of encrypted payload (0-200; ensure < LoRa payload limit).
- `EncryptedPayload` (variable): AES-GCM ciphertext of the application payload (encrypted with AAD above).
- `AuthTag` (16 bytes): AES-GCM tag.

## Nonce Construction

The nonce should be 12 bytes which are never reused.

`nonce = dev_id(2 bytes) || 6 bytes zero-padding || seq(4 bytes)`
    - Keep device id in nonce to separate counters by device.
    - 4-byte seq gives 4-byte monotonic counter (wrap at 2^32, rotate keys before wrap).
    - The 6 zero bytes give room.
    - *Never* reuse a nonce for the same key.

## Key Lifecycle

- Use a 128- or 256-bit symmetric key per device.
- Rotate key after N messages (e.g., 1 million messages) or after T time (e.g., 30 days).
- For now, provision keys as pre-shared via secure channel for testing purposes.
    - Add X25519-based ECDH + HKDF derivation later for on-field ability to rekey.

## Fragmentation

- If payload > `MaxPayload` (chosen based on SF and region), set `flags.fragmented=1` and include small fragmentation header in payload:
    - `frag_id` (1 byte)
    - `frag_index` (1 byte)
    - `frag_total` (1 byte)
- Reassemble fragments at receiver with a timeout (e.g., 5s).
- If not all fragments received, sender retransmits missing frags using ACK bitmap.

## ACK & Retransmission

- Simple ACK packet: header with `flags=ACK`, `seq=acking_seq`, zero-length payload.
- Receiver must send ACK within ACK_WINDOW (e.g., 2s).
- Retransmit policy: up to 5 tries, exponential backoff with random jitter. Sliding window size 4.
- If ACK not received, increase SF or drop to lower data rate depending on policy (can be pushed until later for advanced tuning functionality).

## Error Detection Under Interference & Graceful Failures

- If AES tag fails, discard silently and increment `auth_fail` counter.
- Log RSSI and SNR for every received packet. These can be used for transmission parameter updates.
- If repeated send fails (3 attempts) mark peer as "unreachable" and notify operator through UART shell with detailed counters (retries, last RSSI, last SNR, last auth_fail).

## Telemetry & Diagnostics

- Every sent/received packet stored in a circular log entry with:
    - timestamp
    - dev_id
    - seq
    - flags
    - payload_len
    - retransmits
    - RSSI
    - SNR
    - auth_ok

---

# Implementation Roadmap

1. **Week 1 - Establish a functional (not pretty) UART shell with commands**: `send`, `recv`, `status`, `load_key`, `show_log`.
2. **Week 2 - Implement framing & AES-GCM**: header (AAD), nonce scheme, encrypt/decrypt with tag check.
    - Should create a fuzzer to simulate packet corruption and ensure proper handling.
3. **Week 3 - Add seq, ACK logic, retransmit**: implement sliding window (size 4), ACK packets, retransmit with backoff, and log each event.
4. **Week 4 - Add fragmentation & reassembly**: simple frag headers, reassembly w/ timeout, retransmit missing fragments, handle out-of-order fragments.
    - Should test with simulated loss.
5. **Week 5 - Field test & tune**: run over real LoRa with noise, capture logs, iterate.
    - Try to force "real" noice by putting one device behind attenuation or a metal box.