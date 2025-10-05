#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora_driver.h"
#include <stdio.h>

void app_main(void)
{
  printf("Hello, ESP32-S3!\n");

  ESP_LOGI("Hello", "Hello from ESP-IDF!");

  lora_driver_init();

  while (1)
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  lora_driver_deinit();
}