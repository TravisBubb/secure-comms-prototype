/**
 * @file lora_cmd_sx1262.h
 * @brief LoRa command definitions for the SX1262 module.
 *
 * NOTE: This file is an internal part of the LoRa driver and should not
 *       be included directly by application code.
 */

#ifndef LORA_CMD_SX1262_H
#define LORA_CMD_SX1262_H

#include <esp_err.h>

// --- Low-level command access --

/**
 * @brief Send a low-level command to the SX1262 with optional arguments.
 *
 * This function writes a command opcode and associated arguments to the SX1262.
 * Intended for internal driver use only.
 *
 * @param opcode The command opcode to send.
 * @param args Pointer to the arguments buffer (can be NULL if no arguments).
 * @param args_len Length of the arguments in bytes.
 *
 * @return ESP_OK if successful, or an error code otherwise.
 */
esp_err_t sx1262_cmd_write(uint8_t opcode, const uint8_t *args, size_t args_len);

/**
 * @brief Read data from the SX1262 after sending a command.
 *
 * Sends a command opcode to request data from the chip and reads the response.
 * Intended for internal driver use only.
 *
 * @param opcode The command opcode to send.
 * @param out Pointer to the buffer to store the read data.
 * @param out_len Length of the buffer in bytes.
 *
 * @return ESP_OK if successful, or an error code otherwise.
 */
esp_err_t sx1262_cmd_read(uint8_t opcode, uint8_t *out, size_t out_len);

// --- Chip control ---

/**
 * @brief Set the SX1262 to standby mode.
 *
 * Required before most other operations. Blocks until the chip is ready.
 *
 * @return ESP_OK if successful, or an error code otherwise.
 */
esp_err_t sx1262_cmd_set_standby(void);

/**
 * @brief Start a transmission on the SX1262.
 *
 * Begins transmitting the data currently loaded in the TX buffer.
 *
 * @param timeout_ms Timeout in milliseconds (0 for no timeout / blocking indefinitely).
 * @return ESP_OK if successful, or an error code otherwise.
 */
esp_err_t sx1262_cmd_set_tx(uint32_t timeout_ms);

/**
 * @brief Set the SX1262 to receive mode.
 *
 * Begins listening for incoming packets. Can block until timeout or indefinitely.
 *
 * @param timeout_ms Timeout in milliseconds (0 for continuous reception).
 * @return ESP_OK if successful, or an error code otherwise.
 */
esp_err_t sx1262_cmd_set_rx(uint32_t timeout_ms);

// --- Status / IRQ ---

/**
 * @brief Get the current status of the SX1262.
 *
 * Reads the status byte from the chip, including busy flag and mode information.
 *
 * @param status Pointer to store the returned status byte.
 * @return ESP_OK if successful, or an error code otherwise.
 */
esp_err_t sx1262_cmd_get_status(uint8_t *status);

/**
 * @brief Clear specific IRQ flags on the SX1262.
 *
 * @param irq_mask Mask of IRQ flags to clear (bitwise OR of SX1262 IRQ bits).
 * @return ESP_OK if successful, or an error code otherwise.
 */
esp_err_t sx1262_cmd_clear_irq(uint16_t irq_mask);


/**
 * @brief Get the current IRQ status from the SX1262.
 *
 * @param irq_status Pointer to store IRQ status flags.
 * @return ESP_OK if successful, or an error code otherwise.
 */
esp_err_t sx1262_cmd_get_irq_status(uint16_t *irq_status);

// --- Buffer management ---

/**
 * @brief Set the base addresses for TX and RX buffers in the SX1262.
 *
 * @param tx_base Starting address for the TX buffer.
 * @param rx_base Starting address for the RX buffer.
 * @return ESP_OK if successful, or an error code otherwise.
 */
esp_err_t sx1262_cmd_set_buffer_base_address(uint8_t tx_base, uint8_t rx_base);

/**
 * @brief Write data into the SX1262 TX buffer.
 *
 * @param offset Offset into the TX buffer to start writing.
 * @param data Pointer to the data to write.
 * @param len Number of bytes to write.
 * @return ESP_OK if successful, or an error code otherwise.
 */
esp_err_t sx1262_cmd_write_buffer(uint8_t offset, const uint8_t *data, size_t len);

/**
 * @brief Read data from the SX1262 RX buffer.
 *
 * @param offset Offset into the RX buffer to start reading.
 * @param data Pointer to store the read data.
 * @param len Number of bytes to read.
 * @return ESP_OK if successful, or an error code otherwise.
 */
esp_err_t sx1262_cmd_read_buffer(uint8_t offset, uint8_t *data, size_t len);

#endif // LORA_CMD_SX1262_H
