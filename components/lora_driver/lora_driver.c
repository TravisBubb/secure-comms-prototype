#include "lora_driver.h"
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>

static const char *TAG = "LORA_DRIVER";

static SemaphoreHandle_t spi_mutex;
spi_device_handle_t lora_handle;

esp_err_t lora_driver_init(void)
{
  esp_err_t ret;

  spi_mutex = xSemaphoreCreateMutex();
  if (!spi_mutex)
  {
    ESP_LOGE(TAG, "Failed to create SPI mutex.");
    return ESP_ERR_NO_MEM;
  }

  spi_bus_config_t buscfg = {
      .mosi_io_num = GPIO_NUM_10, // connect to LoRa MOSI ("Master Out, Slave In")
      .miso_io_num = GPIO_NUM_11, // connect to LoRa MISO ("Master In, Slave Out")
      .sclk_io_num = GPIO_NUM_9,  // connect to LoRa SCK (Synchronization Clock)
      .quadwp_io_num = -1,        // only for quad-SPI
      .quadhd_io_num = -1,        // only for quad-SPI
      .max_transfer_sz = 0        // 0 means use default buffer size
  };

  ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to init SPI bus with status: %d.", ret);
    return ret;
  }

  spi_device_interface_config_t devcfg = {
      .clock_speed_hz = 1 * 1000 * 1000, // 1MHz for now
      .mode = 0,                         // SPI mode 0 for SX127x
      .spics_io_num = GPIO_NUM_8,        // chip select pin
      .queue_size = 1                    // minimal queue for testing
  };

  ret = spi_bus_add_device(SPI3_HOST, &devcfg, &lora_handle);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to add SPI device with status: %d.", ret);
    return ret;
  }

  ESP_LOGI(TAG, "SPI initialized successfully.");

  return ESP_OK;
}