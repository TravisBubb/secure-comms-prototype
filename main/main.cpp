#include "esp_log.h"
#include "lora_driver.h"

#define BAND 915E6
#define LORA_NSS 8
#define LORA_SCK 9
#define LORA_MOSI 10
#define LORA_MISO 11
#define LORA_RESET 12
#define LORA_BUSY 13
#define LORA_DIO1 14

static const char *TAG = "MAIN";

extern "C" void app_main(void)
{
  esp_err_t ret;

  ESP_LOGI(TAG, "Hello, World!");

  lora_t dev = {0};
  lora_config_t cfg = {0};
  cfg.nss = LORA_NSS;
  cfg.sck = LORA_SCK;
  cfg.mosi = LORA_MOSI;
  cfg.miso = LORA_MISO;
  cfg.reset = LORA_RESET;
  cfg.busy = LORA_BUSY;
  cfg.dio1 = LORA_DIO1;
  cfg.spi_host = SPI2_HOST;
  cfg.irq_callback = nullptr;

  ESP_LOGI(TAG, "Initializing LoRa device...");
  ret = lora_init(&dev, &cfg);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to initialize LoRa device: %d.", ret);
    return;
  }
  ESP_LOGI(TAG, "LoRa device initialized successfully.");

  ESP_LOGI(TAG, "Deinitializing LoRa device...");
  ret = lora_deinit(&dev);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "LoRa device deinitialization failed: %d.", ret);
    return;
  }
  ESP_LOGI(TAG, "LoRa device deinitialized successfully.");
}
