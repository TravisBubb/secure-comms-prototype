#ifndef LORA_DRIVER_H
#define LORA_DRIVER_H

#include "driver/spi_master.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum
{
  LORA_DEVICE_TYPE_SX1262 = 0x00,
  LORA_DEVICE_TYPE_SX1261 = 0x01,
} lora_device_type_t;

typedef struct
{
  int mosi;
  int miso;
  int sck;
  int nss;
  int reset;
  int busy;
  int dio1;
  long frequency;
  uint8_t duty_cycle;
  uint8_t hp_max;
  lora_device_type_t device_sel;
  int8_t tx_power;
  uint8_t ramp_time;
  spi_host_device_t spi_host;
  uint8_t sf;
  uint8_t bw;
  uint8_t cr;
  uint8_t ldro;
  uint16_t syncword;
  void (*irq_callback)(void *arg);
} lora_config_t;

typedef struct
{
  lora_config_t cfg;
  spi_device_handle_t spi_handle;
  SemaphoreHandle_t spi_mutex;
  bool initialized;
} lora_t;

typedef enum
{
  LORA_PACKET_TYPE_GFSK = 0x00,
  LORA_PACKET_TYPE_LORA = 0x01,
  LORA_PACKET_TYPE_LR_FHSS = 0x03,
} lora_packet_type_t;

#ifdef __cplusplus
extern "C"
{
#endif

  esp_err_t lora_init(lora_t *dev, const lora_config_t *cfg);
  // esp_err_t lora_reset(lora_t *dev);
  // esp_err_t lora_send(lora_t *dev, const uint8_t *data, size_t len);
  // esp_err_t lora_receive(lora_t *dev, uint8_t *buffer, size_t max_len, size_t *out_len);
  esp_err_t lora_deinit(lora_t *dev);

#ifdef __cplusplus
}
#endif

#endif // LORA_DRIVER_H