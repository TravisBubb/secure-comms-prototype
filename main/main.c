#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

void app_main(void)
{
    printf("Hello, ESP32-S3!\n");

    ESP_LOGI("Hello", "Hello from ESP-IDF!");

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}