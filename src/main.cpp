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

void send(uint8_t argc, char *argv[])
{
  if (argc == 0)
  {
    Serial.println("[error] Usage: send <message> OR send \"multi word message\"");
    return;
  }

  char message[128] = {0};
  bool inQuote = false;
  size_t msgLen = 0;

  for (uint8_t i = 0; i < argc; i++)
  {
    char *arg = argv[i];
    for (size_t j = 0; arg[j] != '\0' && msgLen < sizeof(message) - 1; j++)
    {
      char c = arg[j];

      if (c == '"')
      {
        inQuote = !inQuote; // toggle quote state
        continue;           // skip the quote itself
      }

      message[msgLen++] = c;
    }

    // Add a space *only* if we’re inside a quote and there’s another token coming
    if (i < argc - 1 && inQuote)
    {
      if (msgLen < sizeof(message) - 1) message[msgLen++] = ' ';
    }
  }

  // If we ended still "inside quotes", that's an error.
  if (inQuote)
  {
    Serial.println("[error] Unclosed quote in message.");
    return;
  }

  // Trim trailing spaces
  while (msgLen > 0 && message[msgLen - 1] == ' ')
    message[--msgLen] = '\0';

  if (msgLen == 0)
  {
    Serial.println("[error] Empty message.");
    return;
  }

  int state = radio.transmit(message);
  if (state == RADIOLIB_ERR_NONE) { Serial.printf("\n[info] Sent message: \"%s\"\n", message); }
  else
  {
    Serial.printf("\n[error] TX failed, code: %d\n", state);
  }
}

void receive(uint8_t argc, char *argv[])
{
  uint8_t buffer[128];

  int state = radio.receive(buffer, 128, 10000);
  if (state == RADIOLIB_ERR_NONE)
  {
    size_t len = radio.getPacketLength();
    if (len >= sizeof(buffer)) len = sizeof(buffer) - 1;
    buffer[len] = '\0';

    Serial.printf("\n[info] Received message: %s\n", (const char *)buffer);
  }
  else if (state == RADIOLIB_ERR_RX_TIMEOUT)
  {
    Serial.println("\n[error] RX Timeout.");
    return;
  }
  else
  {
    Serial.printf("\n[error] RX failed, code: %d\n", state);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1);
  Serial.println("USB Serial interface initialized.");

  shell.registerCommand("ping", ping);
  shell.registerCommand("rcv", receive);
  shell.registerCommand("send", send);

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

  Serial.print("> ");
}

void loop()
{
  shell.loop();
}
