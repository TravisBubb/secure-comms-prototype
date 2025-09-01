# Secure Comms Prototype

A prototype secure communications system built with an ESP32 (Heltec V3 development board) and an administration CLI. Demonstrates point-to-point encrypted LoRa messaging, key lifecycle management, and replay protection.

---

## Features (v0.1)

- LoRa point-to-point mesaging
- Authenticated encryption with AES-256-GCM
- UART CLI for send/recv, device status, and key management
- Key management: import, show/debug, wipe/reset
- Replay protection via monotonic counters + nonce derivation
- Persistent storage of keys/counters in NVS
- Host CLI in Rust to interact with device over UART

---

## Repo Structure

- 'firmware/' -> C code for ESP32 firmware (UART CLI + LoRa + crypto)
- 'cli/' -> Rust host CLI tool (UART bridge + admin)
- 'docs/' -> Protocol spec, threat model, design notes

---

## Roadmap

- **v0.1** - UART CLI, LoRa P2P, AEAD crypto, key lifecycle, replay protection
- **v0.2** - Add BLE interface for mobile integration
- **v0.3** - Hardware keyboard + display for standalone device
