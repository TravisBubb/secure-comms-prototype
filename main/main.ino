#include "LoRaWan_APP.h"
#include "Arduino.h"

#define RF_FREQUENCY                  915000000 // Hz
#define TX_OUTPUT_POWER               5         // dBm

#define LORA_BANDWIDTH                0         // 125 kHz
#define LORA_SPREADING_FACTOR         7         // SF7
#define LORA_CODINGRATE               1         // 4/5
#define LORA_PREAMBLE_LENGTH          8
#define LORA_SYMBOL_TIMEOUT           0
#define LORA_FIX_LENGTH_PAYLOAD_ON    false
#define LORA_IQ_INVERSION_ON          false

#define RX_TIMEOUT_VALUE              1000
#define BUFFER_SIZE                   30

char tx_packet[BUFFER_SIZE];
char rx_packet[BUFFER_SIZE];
double tx_number;
bool lora_idle = true;
static RadioEvents_t RadioEvents;

void OnTxDone(void);
void OnTxTimeout(void);

void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  tx_number = 0;

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
      LORA_SPREADING_FACTOR, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
}

void loop() {
  if (lora_idle) {
    delay(1000);
    tx_number += 0.01;
    sprintf(tx_packet, "Hello world number %0.2f", tx_number);

    Serial.printf("\nSending packet \"%s\" , length %d'\n", tx_packet, strlen(tx_packet));

    Radio.Send((uint8_t *)tx_packet, strlen(tx_packet));
    lora_idle = false;
  }
  Radio.IrqProcess();
}

void OnTxDone(void) {
  Serial.println("TX done...");
  lora_idle = true;
}

void OnTxTimeout(void) {
  Radio.Sleep();
  Serial.println("TX timeout...");
  lora_idle = true;
}
