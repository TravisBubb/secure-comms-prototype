#ifndef COMMANDS_H
#define COMMANDS_H

#include "packet.h"

typedef enum {
  CMD_SEND = 0x01,
  CMD_STATUS = 0x02,
  CMD_KEYLOAD = 0x03,
  CMD_RESET = 0x04,
  CMD_PING = 0x05,
} command_t;

typedef void (*command_handler_t)(const packet_t *pkt);

#ifdef __cplusplus
extern "C" {
#endif

void register_command(command_t cmd, command_handler_t handler);
void dispatch_command(const packet_t *pkt);

#ifdef __cplusplus
}
#endif

#endif // COMMANDS_H
