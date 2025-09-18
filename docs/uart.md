# UART Driver Design - Secure Comms Prototype

## Packet Framing

Frame format: START | CMD | LENGTH | PAYLOAD | CRC

Sample:
 - Byte 0        : START (0xAA)
 - Byte 1        : CMD
 - Byte 2        : LEN
 - Byte 3..N     : PAYLOAD (LEN bytes)
 - Byte N+1..N+2 : CRC16 (2 bytes)
