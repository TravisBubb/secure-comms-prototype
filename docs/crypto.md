# Crypography and Key Management

## 1. Overview

This document describes the key management and encryption schemes used in the project. The current implementation focuses on secure generation, storage, and rotation of symmetric keys on a single device.

Key storage and management is handled via:
- `FlashKeyStorage`: Persistent flash-backed storage with two-slot atomic updates, CRC32 integrity checking, and versioning.
- `KeyManager`: In-memory key management, generation using an mbedTLS CTR-DRBG, atomic save/rotate, and zeroization.

## 2. Key Storage

### FlashKeyStorage

- Uses **two flash slots**: `km_slot0` and `km_slot1`.
- Active slot index stored in `km_idx`.
- Each slot contains:
    - Magic value for verification (`KM_SLOT_MAGIC`)
    - Version counter (monotonically increasing)
    - Key length (16 or 32 bytes)
    - Flags (currently unused)
    - Key bytes
    - CRC32 for integrity
- Atomic save process:
    1. Write new key to **inactive slot**.
    2. Update active slot index to point to new slot.
- Provides `loadKey()`, `saveKey()`, and `eraseKey()` methods.

```
            ┌───────────────┐
            │ KeyManager    │
            │  (RAM buffer) │
            └───────┬───────┘
                    │
          generate / load
                    │
                    ▼
        ┌───────────────────────┐
        │ FlashKeyStorage       │
        │  Two Slots:           │
        │  km_slot0 / km_slot1  │
        └───────┬───────────────┘
                │
       read active slot (idx = 0/1)
                │
       ┌────────▼────────┐
       │ Pick slot with  │
       │ highest version │
       └────────┬────────┘
                │
        copy key into RAM
                │
                ▼
         KeyManager in use
                │
         rotate / save
                │
                ▼
      ┌───────────────────────┐
      │ Write new key to      │
      │ inactive slot         │
      │ (idx = !active)       │
      └────────┬──────────────┘
               │
          flip active idx
               │
               ▼
      ┌───────────────────────┐
      │ Old key zeroized in   │
      │ RAM                   │
      └───────────────────────┘
```

## 3. Key Management

### KeyManager

- Template over storage backend implementing the `is_key_storage` trait (e.g., `FlashKeyStorage`).
- Responsibilities:
    - Generate symmetric keys (16 or 32 bytes).
    - Load keys from storage.
    - Save or rotate keys atomically.
    - Zeroize in-RAM key material.
- Uses mbedTLS CTR-DRBG for secure random key generation.

## 4. Encryption Scheme

- Current design is **symmetric key based**: the same key is used for encryption and decryption on the device.
- Keys are local to the device and never exported.
- No encryption/decryption primitives are included in this document yet; this focused on key lifecycle management.

## 5. Security Considerations

- **Key confidentiality:** Keys are never exported from device memory or flash. Use `volatile` when zeroing to prevent compiler optimizations.
- **Integrity protection:** CRC32 guards against accidental corruption and incomplete writes but is **not a cryptographic integrity check.**
- **Tamper resistance:** Flash storage is not protected against a bad actor with physical access. Consider a secure element or HMAC-based storage for stronger guarantees.
- **Key rotation:** Always zeroize old keys immediately after rotation to prevent leakage.
- **Entropy:** CTR-DRBG seeded with platform entropy; ensure that the entropy source is properly initialized.
- **Key derivation and sharing:** Not implemented yet. Sharing a symmetric key across devices without secure provisioning is unsafe.

## 6. Future Enhancements / Notes

- Currently, this system only supports **single-device keys**.
- If multiple devices need to communicate using the same key, we will need:
    - Key derivation or distribution mechanism.
    - Secure key provisioning (manual, over-the-air, or derived from shared secret).
    - Rotation and revocation policy for multi-device keys.