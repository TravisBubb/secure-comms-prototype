/**
 * @file lora_spi.h
 * @brief SPI transaction queue, DMA handling, SPI-safe alloc/free
 */

#ifndef LORA_SPI_H
#define LORA_SPI_H

#include <esp_err.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Initialize the LoRa SPI interface.
 *
 * @return esp_err_t ESP_OK on success, or error code on failure.
 */
esp_err_t lora_spi_init(void);

/**
 * @brief De-Initialize the LoRa SPI interface.
 *
 * @return esp_err_t ESP_OK on success, or error code on failure.
 */
esp_err_t lora_spi_deinit(void);

/**
 * @brief Send a transaction to the LoRa SPI interface.
 *
 * @param tx_buffer Pointer to the buffer to send to the SPI interface.
 * @param tx_bytes Number of bytes to send to the SPI interface.
 * @param rx_buffer Pointer to the buffer to receive data from the SPI interface.
 * @param rx_bytes Number of bytes to read from the SPI interface.
 * @param ensure_dma_safe Flag to indicate whether DMA-safe memory should automatically be allocated
 * and freed.
 *
 * @return esp_err_t ESP_OK on success, or error code on failure.
 */
esp_err_t lora_spi_send(const uint8_t *tx_buffer,
                        size_t tx_bytes,
                        uint8_t *rx_buffer,
                        size_t rx_bytes,
                        bool ensure_dma_safe);

#endif // LORA_SPI_H