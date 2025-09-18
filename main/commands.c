#include "commands.h"
#include <stdio.h>

#define MAX_COMMANDS 256

static command_handler_t handlers[MAX_COMMANDS] = {0};

void register_command(command_t cmd, command_handler_t handler) {
  handlers[cmd] = handler;
}

void dispatch_command(const packet_t *pkt) {
  if (pkt->cmd < MAX_COMMANDS && handlers[pkt->cmd]) {
    handlers[pkt->cmd](pkt);
  } else {
    printf("Unknown command: 0x%02X\n", pkt->cmd);
  }
}
