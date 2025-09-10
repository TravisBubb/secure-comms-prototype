#include "uart_proto.h"
#include <stdint.h>

static uint16_t crc16_update(uint16_t crc, uint8_t data);
static uart_proto_result_t handle_wait_for_start(uart_proto_t *p, uint8_t b);
static uart_proto_result_t handle_read_cmd(uart_proto_t *p, uint8_t b);
static uart_proto_result_t handle_read_len(uart_proto_t *p, uint8_t b);
static uart_proto_result_t handle_read_payload(uart_proto_t *p, uint8_t b);
static uart_proto_result_t handle_read_crc(uart_proto_t *p, uint8_t b);

void uart_proto_init(uart_proto_t *p) {
  if (!p)
    return;

  p->state = UART_STATE_WAIT_FOR_START;
  p->cmd = 0;
  p->index = 0;
  p->length = 0;
  p->crc_recvd = 0;
  p->crc_index = 0;
  p->crc_calc = 0xFFFF;
}

uart_proto_result_t uart_proto_feed(uart_proto_t *p, uint8_t b) {
  if (!p)
    return UART_RX_ERROR;

  switch (p->state) {
  case UART_STATE_WAIT_FOR_START:
    return handle_wait_for_start(p, b);
  case UART_STATE_READ_CMD:
    return handle_read_cmd(p, b);
  case UART_STATE_READ_LEN:
    return handle_read_len(p, b);
  case UART_STATE_READ_PAYLOAD:
    return handle_read_payload(p, b);
  case UART_STATE_READ_CRC:
    return handle_read_crc(p, b);
  }
  return UART_RX_ERROR;
}

uart_proto_result_t handle_wait_for_start(uart_proto_t *p, uint8_t b) {
  if (b == UART_START_BYTE)
    p->state = UART_STATE_READ_CMD;
  return UART_RX_IN_PROGRESS;
}

uart_proto_result_t handle_read_cmd(uart_proto_t *p, uint8_t b) {
  p->cmd = b;
  p->crc_calc = crc16_update(p->crc_calc, b);
  p->state = UART_STATE_READ_LEN;
  return UART_RX_IN_PROGRESS;
}

uart_proto_result_t handle_read_len(uart_proto_t *p, uint8_t b) {
  p->length = b;
  p->crc_calc = crc16_update(p->crc_calc, b);
  p->state = p->length == 0 ? UART_STATE_READ_CRC : UART_STATE_READ_PAYLOAD;
  return UART_RX_IN_PROGRESS;
}

uart_proto_result_t handle_read_payload(uart_proto_t *p, uint8_t b) {
  if (p->index >= UART_MAX_PAYLOAD)
    return UART_RX_ERROR;
  p->buffer[p->index++] = b;
  p->crc_calc = crc16_update(p->crc_calc, b);
  if (p->index == p->length)
    p->state = UART_STATE_READ_CRC;
  return UART_RX_IN_PROGRESS;
}

uart_proto_result_t handle_read_crc(uart_proto_t *p, uint8_t b) {
  p->crc_recvd = p->crc_index == 0 ? (b << 8) : (p->crc_recvd | b);
  p->crc_index++;

  if (p->crc_index != 2)
    return UART_RX_IN_PROGRESS;

  uart_proto_result_t result =
      p->crc_calc == p->crc_recvd ? UART_RX_PACKET_COMPLETE : UART_RX_ERROR;
  return result;
}

static uint16_t crc16_update(uint16_t crc, uint8_t data) {
  crc ^= (uint16_t)data << 8;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x8000)
      crc = (crc << 1) ^ 0x1021;
    else
      crc <<= 1;
  }
  return crc;
}
