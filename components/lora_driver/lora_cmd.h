/**
 * @file lora_cmd.h
 * @brief Responsible for turning high-level operations into SPI commands.
 */

#ifndef LORA_CMD_H
#define LORA_CMD_H

#include <esp_err.h>

/**
 * @brief Represents the available standby modes for the SX1262 LoRa chip.
 */
typedef enum
{
  LORA_STDBY_RC = 0x00,
  LORA_STDBY_XOSC = 0x01,
} lora_standby_mode_t;

/**
 * @brief Represents the available packet types for the SX1262 LoRa chip.
 */
typedef enum
{
  LORA_PACKET_TYPE_GFSK = 0x00,
  LORA_PACKET_TYPE_LORA = 0x01,
  LORA_PACKET_TYPE_LR_FHSS = 0x03,
} lora_packet_type_t;

/**
 * @brief Sets the LoRa chip into the given standby mode.
 *
 * @param mode The standby mode to select.
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t lora_set_standby(lora_standby_mode_t mode);

/**
 * @brief Sets the LoRa chip's packet type.
 *
 * @param type The packet type to select.
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t lora_set_packet_type(lora_packet_type_t type);

/**
 * @brief Sets the LoRa chip's RF frequency.
 *
 * @param hz The desired RF frequency in Hz.
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t lora_set_rf_frequency(uint32_t hz);

#endif // LORA_CMD_H