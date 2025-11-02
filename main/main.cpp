#include "esp_log.h"
#include "lora_driver.h"
#include <string.h>
#include <driver/gpio.h>

#define BAND 915E6
#define LORA_NSS GPIO_NUM_8
#define LORA_SCK GPIO_NUM_9
#define LORA_MOSI GPIO_NUM_10
#define LORA_MISO GPIO_NUM_11
#define LORA_RESET GPIO_NUM_12
#define LORA_BUSY GPIO_NUM_13
#define LORA_DIO1 GPIO_NUM_14

static const char *TAG = "MAIN";

extern "C" void app_main(void)
{
  esp_err_t ret;

  ESP_LOGI(TAG, "Hello, World!");

  lora_t dev;
  lora_config_t cfg;

  cfg.nss = LORA_NSS;
  cfg.sck = LORA_SCK;
  cfg.mosi = LORA_MOSI;
  cfg.miso = LORA_MISO;
  cfg.reset = LORA_RESET;
  cfg.busy = LORA_BUSY;
  cfg.dio1 = LORA_DIO1;
  cfg.spi_host = SPI2_HOST;

  cfg.irq_callback = nullptr;

  cfg.frequency = 915E6;

  cfg.duty_cycle = 0x02;
  cfg.hp_max = 0x02;
  cfg.device_sel = LORA_DEVICE_TYPE_SX1262;
  cfg.tx_power = 14;    // dBm
  cfg.ramp_time = 0x04; // 200 us ramp

  cfg.sf = 0x07;   // SF7
  cfg.bw = 0x04;   // 125kHz
  cfg.cr = 0x01;   // 4/5
  cfg.ldro = 0x00; // OFF

  cfg.syncword = 0x1424;


  ESP_LOGI(TAG, "Initializing LoRa device...");
  ret = lora_init(&dev, &cfg);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to initialize LoRa device: %d.", ret);
    return;
  }
  ESP_LOGI(TAG, "LoRa device initialized successfully.");

  ESP_LOGI(TAG, "Sending test message...");
  const char *payload = "HELLO";
  const uint8_t *buf = (const uint8_t *)payload;
  size_t len = strlen(payload);
  ret = lora_send(&dev, buf, len);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to send message (%s) with length %d.", "HELLO", 5);
    return;
  }
  ESP_LOGI(TAG, "Test message sent.");

  ESP_LOGI(TAG, "Deinitializing LoRa device...");
  ret = lora_deinit(&dev);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "LoRa device deinitialization failed: %d.", ret);
    return;
  }
  ESP_LOGI(TAG, "LoRa device deinitialized successfully.");
}
