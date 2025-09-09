#include "Arduino.h"
#include "uart.h"

uart_parser_t parser;

void setup() {
  Serial.begin(115200);
  uart_parser_init(&parser);
  Serial.println("UART parser test starting...");
}

void loop() {
  while (Serial.available()) {
    uint8_t b = Serial.read();

    Serial.print("RX byte: 0x");
    if (b < 0x10) Serial.print("0");
    Serial.println(b, HEX);

    Serial.print(" STATE: ");
    Serial.println(parser.state);

    uart_rx_rc result = uart_rx_handle(&parser, b);

    if (result == UART_RX_PACKET_COMPLETE) {
      Serial.print("Packet received. CMD: ");
      Serial.print(parser.cmd, HEX);
      Serial.print(" LEN: ");
      Serial.println(parser.length);

      Serial.print("Payload: ");
      for (size_t i = 0; i < parser.length; i++) {
          Serial.print(parser.buffer[i], HEX);
          Serial.print(" ");
      }
      Serial.println();
      uart_parser_init(&parser);
    } else if (result == UART_RX_ERROR) {
      Serial.println("Packet error.");
      uart_parser_init(&parser);
    }
  }
}
