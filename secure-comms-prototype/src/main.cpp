#include "SoftwareSerial.h"
#include "heltec_wifi_lora_v3.h"
#include <Arduino.h>
#include <RadioLib.h>

SPIClass spiLoRa(HSPI);
SX1262 radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO1, PIN_LORA_RST, PIN_LORA_BUSY, spiLoRa);
SoftwareSerial shellSerial(PIN_SOFTWARE_RX, PIN_SOFTWARE_TX);

void setup()
{
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("USB Serial interface initialized.");

  shellSerial.begin(115200);
  while (!shellSerial) {}
  Serial2.println("Shell interface initialized.");

  Serial.println("Initializing SPI...");
  spiLoRa.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_NSS);
  Serial.println("SPI initialized.");

  Serial.println("Initializing SX1262...");
  int state = radio.begin(LORA_FREQUENCY_MHZ,
                          LORA_BANDWIDTH_KHZ,
                          LORA_SPREADING_FACTOR,
                          LORA_CODING_RATE_DENOMINATOR,
                          LORA_SYNCWORD,
                          LORA_POWER_DBM,
                          LORA_PREAMBLE_LENGTH);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print("SX1262 initialization failed, code: ");
    Serial.println(state);
    while (true) {}
  }

  Serial.println("SX1262 initialized.");
}

void loop()
{
  while (shellSerial.available())
  {
    char c = shellSerial.read();
    Serial.write(c);
  }
  while (Serial.available())
  {
    char c = Serial.read();
    shellSerial.write(c);
  }
}
