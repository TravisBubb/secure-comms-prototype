/**
 * @file lora_hw.h
 * @brief SX1262 pin configuration, BUSY/DIO helpers, low-level hardware routines
 */

#ifndef LORA_HW_H
#define LORA_HW_H

#include <esp_err.h>

/**
 * @brief Wait for the BUSY pin to go low.
 *
 * @return esp_err_t ESP_OK on success, or error code on failure.
 */
esp_err_t lora_hw_wait_busy(void);

#endif // LORA_HW_H