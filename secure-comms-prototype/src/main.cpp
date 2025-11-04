#include "Shell.h"
#include "heltec_wifi_lora_v3.h"
#include <Arduino.h>
#include <RadioLib.h>

SPIClass spiLoRa(HSPI);
SX1262 radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO1, PIN_LORA_RST, PIN_LORA_BUSY, spiLoRa);
Shell shell;

void ping(uint8_t argc, char *argv[])
{
  Serial.println("PONG!");
  if (argc > 0)
  {
    Serial.print(" Args: ");
    for (uint8_t i = 0; i < argc; i++)
    {
      Serial.print(argv[i]);
      if (i < argc - 1) Serial.print(", ");
    }
  }
  Serial.println();
}

void setup()
{
  Serial.begin(115200);
  delay(1);
  Serial.println("USB Serial interface initialized.");

  shell.registerCommand("ping", ping);

  // Serial.println("Initializing SPI...");
  // spiLoRa.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_NSS);
  // Serial.println("SPI initialized.");

  // Serial.println("Initializing SX1262...");
  // int state = radio.begin(LORA_FREQUENCY_MHZ,
  //                         LORA_BANDWIDTH_KHZ,
  //                         LORA_SPREADING_FACTOR,
  //                         LORA_CODING_RATE_DENOMINATOR,
  //                         LORA_SYNCWORD,
  //                         LORA_POWER_DBM,
  //                         LORA_PREAMBLE_LENGTH);
  // if (state != RADIOLIB_ERR_NONE)
  // {
  //   Serial.print("SX1262 initialization failed, code: ");
  //   Serial.println(state);
  //   while (true) {}
  // }

  // Serial.println("SX1262 initialized.");

  Serial.print("> ");
}

void loop()
{
  shell.loop();
}
