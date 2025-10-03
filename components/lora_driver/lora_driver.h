#ifndef LORA_DRIVER_H
#define LORA_DRIVER_H

#include <esp_err.h>

/**
 * @brief Initialize the LoRa driver.
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t lora_driver_init(void);

/**
 * @brief Set the LoRa frequency.
 *
 * @param hz Frequency in Hertz.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t lora_driver_set_frequency(uint32_t hz);

/**
 * @brief Set the LoRa transmission power.
 *
 * @param level Power level (e.g., 0-20 dBm).
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t lora_driver_set_tx_power(int level);

/**
 * @brief Send data over LoRa.
 *
 * @param data Pointer to the data buffer to send.
 * @param len Length of the data in bytes.
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t lora_driver_send(const uint8_t *data, size_t len);

/**
 * @brief Receive data over LoRa.
 *
 * @param buffer Pointer to the buffer to store received data.
 * @param max_len Maximum length of the buffer.
 * @param out_len Pointer to store the actual length of received data.
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t lora_driver_receive(uint8_t *buffer, size_t max_len, size_t *out_len);

/**
 * @brief Deinitialize the LoRa driver and free resources.
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t lora_driver_deinit(void);

#endif // LORA_DRIVER_H