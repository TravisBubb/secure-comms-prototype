#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <sx126x/sx126x.h>

static void esp_log_wrapper(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    char buffer[128];
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    ESP_LOGI("SX126X", "%s", buffer);

    va_end(args);
}

void app_main(void)
{
  printf("Hello, ESP32-S3!\n");

  ESP_LOGI("Hello", "Hello from ESP-IDF!");

  sx126x_config_t config = {
      .log = esp_log_wrapper,
      .frequency_hz = 915000000, // 915 MHz
  };

  sx126x_init(NULL, NULL, &config);

  while (1)
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}