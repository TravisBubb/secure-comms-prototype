#include "Arduino.h"
#include "commands.h"
#include "packet.h"
#include "uart_proto.h"

uart_proto_t parser;
packet_t pkt;

void handle_ping(const packet_t *pkt) {
  Serial.println("Pong!");
}

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing UART parser...");
  uart_proto_init(&parser);
  Serial.println("Registering command: ping...");
  register_command(CMD_PING, handle_ping);

  Serial.println("Finished initialization, board ready...");
}

void loop() {
  while (Serial.available()) {
    uint8_t b = Serial.read();
    uart_proto_result_t res = uart_proto_feed(&parser, b);
    if (res == UART_RX_PACKET_COMPLETE) {
      pkt = uart_proto_to_packet(&parser);
      dispatch_command(&pkt);
      uart_proto_init(&parser);
    }
  }
}
