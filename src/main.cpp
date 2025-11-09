#include "Shell.h"
#include "heltec_wifi_lora_v3.h"
#include "securelink.h"
#include <Arduino.h>
#include <string.h>

Shell shell;
SecureLink secureLink(SecureLinkConfig{.gpio = {.misoPin = PIN_LORA_MISO,
                                          .mosiPin = PIN_LORA_MOSI,
                                          .nssPin = PIN_LORA_NSS,
                                          .sckPin = PIN_LORA_SCK,
                                          .dio1Pin = PIN_LORA_DIO1,
                                          .rstPin = PIN_LORA_RST,
                                          .busyPin = PIN_LORA_BUSY},
                                 .radio =
                                     {
                                         .freqHz = 915000000,
                                         .bwHz = 125000,
                                         .sf = 7,
                                         .crDen = 5,
                                         .syncWord = 0x12,
                                         .powerDbm = 10,
                                         .preambleLen = 8,
                                     },
                                 .dll = {.enableCrc = true, .enableFragmentation = true}});

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

  Packet pkt = {.destId = 0x1234, .payloadLen = strlen(message)};
  memcpy(&pkt.payload, message, pkt.payloadLen);
  int state = secureLink.send(pkt);
  if (state == RADIOLIB_ERR_NONE) { Serial.printf("\n[info] Sent message: \"%s\"\n", message); }
  else
  {
    Serial.printf("\n[error] TX failed, code: %d\n", state);
  }
}

void receive(uint8_t argc, char *argv[])
{
  Packet pkt;
  int state = secureLink.receive(pkt, 10000);
  if (state == RADIOLIB_ERR_NONE)
  {
    char text[200];
    memcpy(text, pkt.payload, pkt.payloadLen);
    text[pkt.payloadLen] = '\0';
    Serial.printf("\n[info] Received message: %s\n", text);
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
  Serial.println("[Init] USB Serial interface initialized.");

  shell.registerCommand("ping", ping);
  shell.registerCommand("rcv", receive);
  shell.registerCommand("send", send);

  Serial.println("[Init] Initializing SX1262...");
  int state = secureLink.begin();
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print("[Init] SX1262 initialization failed, code: ");
    Serial.println(state);
    while (true) {}
  }

  Serial.println("[Init] SX1262 initialized.");
  delay(100);
  Serial.print("> ");
}

void loop()
{
  shell.loop();
}
