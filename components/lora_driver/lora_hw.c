#include "lora_hw.h"
#include "board_heltec_v3.h"
#include "freertos/FreeRTOS.h"
#include <driver/gpio.h>
#include <esp_log.h>

static const char *TAG = "LORA_HW";

esp_err_t lora_hw_wait_busy(void)
{
  const TickType_t start = xTaskGetTickCount();
  const TickType_t timeout = pdMS_TO_TICKS(100);

  ESP_LOGD(TAG, "Waiting for BUSY pin to go low.");

  while (gpio_get_level(LORA_BUSY_PIN) == 1)
  {
    if ((xTaskGetTickCount() - start) > timeout)
    {
      ESP_LOGE(TAG, "Timeout waiting for BUSY pin to go low.");
      return ESP_ERR_TIMEOUT;
    }
    vTaskDelay(pdMS_TO_TICKS(1)); // short delay to yield CPU
  }

  ESP_LOGD(TAG, "BUSY pin is low.");

  return ESP_OK;
}