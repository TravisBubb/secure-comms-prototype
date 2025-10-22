#include "board_heltec_v3.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdio.h>
// #include <sx126x/hal.h>
// #include <sx126x/hal_esp32.h>
// #include <sx126x/sx126x.h>

void app_main(void)
{
  printf("Hello, ESP32-S3!\n");

  ESP_LOGI("Hello", "Hello from ESP-IDF!");

  // sx126x_config_t dev_cfg = {
  //     .chip = SX126X_CHIP_SX1262,
  //     .frequency_hz = 915000000, // 915 MHz
  //     .pa_profile = SX126X_PA_MEDIUM_POWER,
  //     .modem = SX126X_MODEM_LORA,
  //     .power_dbm = 17,
  //     .power_ramp_time = SX126X_PWR_RAMP_TIME_200U,
  //     .lora_sf = SX126X_LORA_SF_7,
  //     .lora_bw = SX126X_LORA_BW_125,
  //     .lora_cr = SX126X_LORA_CR_4_5,
  //     .lora_ldro = false,
  // };

  // sx126x_hal_esp32_cfg_t hal_cfg = {
  //     .spi_host = SPI3_HOST,
  //     .spi_mosi_pin = LORA_MOSI_PIN,
  //     .spi_miso_pin = LORA_MISO_PIN,
  //     .spi_sclk_pin = LORA_SCLK_PIN,
  //     .spi_max_transfer_size = 0,
  //     .spi_clock_speed_hz = 1 * 1000 * 1000, // 1 MHz
  //     .spi_cs_pin = LORA_CS_PIN,
  //     .spi_queue_size = 8,
  // };

  // sx126x_hal_esp32_t hal;
  // sx126x_hal_esp32_init(&hal, &hal_cfg);

  // sx126x_bus_t *bus = sx126x_hal_get_bus(&hal);
  // sx126x_t *dev = sx126x_hal_get_device(&hal);

  // sx126x_init(dev, bus, &dev_cfg);

  while (1)
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // sx126x_hal_esp32_deinit(&hal);
  // sx126x_deinit(dev);
}