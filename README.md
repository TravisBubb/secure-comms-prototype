# Secure Comms Prototype

> ⚠️ Work in Progress — this repository is an active development effort and is not yet stable. Expect rapid iteration, breaking changes, and incomplete functionality.

## Hardware Setup

This project uses a **Heltec WiFi LoRa 32 V3** development board and an **external USB-to-UART adapter** for a dedicated software UART shell. The wiring is as follows:

### Software UART (Shell)

| Heltec Pin  | Connected To            | Notes                                            |
| ----------- | ----------------------- | ------------------------------------------------ |
| GPIO45 (TX) | RX of USB-UART adapter  | Sends data to host terminal                      |
| GPIO46 (RX) | TX of USB-UART adapter  | Receives data from host terminal                 |
| GND         | GND of USB-UART adapter | Common ground required for proper UART operation |

### USB Serial (Logging/Flashing)

- Connect the Heltec board via USB to your PC. This is used for logging (`Serial`) and for flashing the firmware.
- **Important:** Disconnect external peripherals (including the software UART adapter) when flashing, otherwise the board may fail to enter download mode.

### Other Connected Hardware

- SX1262 LoRa transceiver: connected to SPI pins defined in the code (`PIN_LORA_NSS`, `PIN_LORA_DIO1`, `PIN_LORA_RST`, `PIN_LORA_BUSY`, `PIN_LORA_MOSI`, `PIN_LORA_MISO`)
- OLED display: connected to GPIO 17/18 (SDA/SCL)

## Software Setup

- PlatformIO (recommended) or Arduino IDE
- Libraries:
  - [RadioLib](https://github.com/jgromes/RadioLib) (v7.4.0)
  - [EspSoftwareSerial](https://github.com/plerup/espsoftwareserial) (v8.2.0)
- PlatformIO board: `heltec_wifi_lora_32_V3`

## Building the Code

The code can be built using PlatformIO's `pio` utility. Either run `make build` or manually call `pio run` to build the binary.

## Flashing / Upload

1. Disconnect external USB-UART adapter to prevent electrical issues.
2. Connect the Heltec board via USB to your PC.
3. Run `pio run --target upload --upload-port [port]` (or `make upload PORT=[port]`) to flash the firmware.
4. Reconnect the USB-UART adapter after flashing.
