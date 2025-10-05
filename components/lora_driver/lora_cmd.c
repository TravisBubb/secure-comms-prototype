#include "lora_cmd.h"
#include "lora_spi.h"
#include <esp_log.h>

static const char *TAG = "LORA_CMD";
static const uint32_t LORA_FREQ_XTAL_HZ = 32000000; // 32 MHz

typedef enum
{
  LORA_OP_SET_STANDBY = 0x80,
  LORA_OP_SET_RF_FREQUENCY = 0x86,
  LORA_OP_SET_PACKET_TYPE = 0x8A,
} lora_opcode_t;

esp_err_t lora_set_standby(lora_standby_mode_t mode)
{
  esp_err_t ret;
  uint8_t tx_buffer[] = {LORA_OP_SET_STANDBY, mode};

  ESP_LOGI(TAG, "Attempting to set standby mode.");

  ret = lora_spi_send(tx_buffer, sizeof(tx_buffer), NULL, 0, true);
  if (ret != ESP_OK)
  {
    return ret;
  }

  ESP_LOGI(TAG, "Standby mode set.");
  return ESP_OK;
}

esp_err_t lora_set_packet_type(lora_packet_type_t type)
{
  esp_err_t ret;
  uint8_t tx_buffer[] = {LORA_OP_SET_PACKET_TYPE, type};

  ESP_LOGI(TAG, "Attempting to set packet type.");

  ret = lora_spi_send(tx_buffer, sizeof(tx_buffer), NULL, 0, true);
  if (ret != ESP_OK)
  {
    return ret;
  }

  ESP_LOGI(TAG, "Packet type set.");
  return ESP_OK;
}

esp_err_t lora_set_rf_frequency(uint32_t hz)
{
  esp_err_t ret;
  uint32_t freq = (uint32_t)(((uint64_t)hz << 25) / LORA_FREQ_XTAL_HZ);
  uint8_t tx_buffer[] = {
      LORA_OP_SET_RF_FREQUENCY,
      (freq >> 24) & 0xFF,
      (freq >> 16) & 0xFF,
      (freq >> 8) & 0xFF,
      freq & 0xFF,
  };

  ESP_LOGI(TAG, "Attempting to set RF frequency.");

  ret = lora_spi_send(tx_buffer, sizeof(tx_buffer), NULL, 0, true);
  if (ret != ESP_OK)
  {
    return ret;
  }

  ESP_LOGI(TAG, "RF frequency set to %dhz.", hz);
  return ESP_OK;
}