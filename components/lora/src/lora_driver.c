#include "lora_driver.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

#define LORA_READY_TIMEOUT_MS 100
#define LORA_READY_POLL_INTERVAL_MS 1

static const char *TAG = "LORA";

typedef enum
{
  LORA_CMD_SET_STANDBY = 0x80,
  LORA_CMD_GET_STATUS = 0xC0,
} lora_cmd_t;

static esp_err_t lora_spi_init(lora_t *dev, const lora_config_t *cfg);
static esp_err_t lora_spi_deinit(lora_t *dev);
static esp_err_t lora_spi_transfer(lora_t *dev,
                                   lora_cmd_t cmd,
                                   const uint8_t *tx_data,
                                   size_t tx_len,
                                   uint8_t *rx_data,
                                   size_t rx_len);
static esp_err_t lora_spi_transfer_safe(lora_t *dev,
                                        lora_cmd_t cmd,
                                        const uint8_t *tx_data,
                                        size_t tx_len,
                                        uint8_t *rx_data,
                                        size_t rx_len);
static esp_err_t lora_wait_ready(lora_t *dev);
static esp_err_t lora_cmd_get_status(lora_t *dev, uint8_t *chip_mode, uint8_t *cmd_status);
static esp_err_t lora_cmd_set_standby(lora_t *dev);
static void lora_print_status(uint8_t chip_mode, uint8_t cmd_status);

esp_err_t lora_init(lora_t *dev, const lora_config_t *cfg)
{
  if (!dev || !cfg) return ESP_ERR_INVALID_ARG;

  esp_err_t ret;

  vTaskDelay(pdMS_TO_TICKS(50));

  dev->spi_mutex = xSemaphoreCreateMutex();
  if (!dev->spi_mutex)
  {
    ESP_LOGE(TAG, "Failed to create SPI mutex.");
    return ESP_ERR_NO_MEM;
  }

  ret = lora_spi_init(dev, cfg);
  if (ret != ESP_OK) return ret;

  uint8_t chip_mode, cmd_status;
  ret = lora_cmd_get_status(dev, &chip_mode, &cmd_status);
  if (ret != ESP_OK) return ret;

  lora_print_status(chip_mode, cmd_status);

  ESP_LOGI(TAG, "Setting LoRa to standby mode...");
  ret = lora_cmd_set_standby(dev);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to set LoRa to standby mode: %d.", ret);
    return ret;
  }
  ESP_LOGI(TAG, "LoRa set to standby mode.");

  vTaskDelay(pdMS_TO_TICKS(10));

  lora_print_status(chip_mode, cmd_status);

  // SetPacketType
  // SetRfFrequency
  // SetPaConfig
  // SetTxParams
  // SetBufferBaseAddress
  // SetModulationParams
  // SetDioIrqParams
  // WriteReg (sync word)

  return ESP_OK;
}

esp_err_t lora_deinit(lora_t *dev)
{
  if (!dev) return ESP_ERR_INVALID_ARG;

  esp_err_t ret;

  ret = lora_spi_deinit(dev);
  if (ret != ESP_OK) return ret;

  if (dev->spi_mutex)
  {
    vSemaphoreDelete(dev->spi_mutex);
    dev->spi_mutex = NULL;
  }

  return ESP_OK;
}

static esp_err_t lora_spi_init(lora_t *dev, const lora_config_t *cfg)
{
  if (!dev || !cfg) return ESP_ERR_INVALID_ARG;
  esp_err_t ret;
  dev->cfg = *cfg;

  spi_bus_config_t buscfg = {
      .miso_io_num = cfg->miso,
      .mosi_io_num = cfg->mosi,
      .sclk_io_num = cfg->sck,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 0,
  };
  ret = spi_bus_initialize(cfg->spi_host, &buscfg, SPI_DMA_CH_AUTO);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to initialize SPI bus: %d.", ret);
    return ret;
  }

  spi_device_interface_config_t devcfg = {
      .clock_speed_hz = 10 * 1000 * 1000, // 10 MHz
      .mode = 0,
      .spics_io_num = cfg->nss,
      .queue_size = 8,
  };
  ret = spi_bus_add_device(cfg->spi_host, &devcfg, &dev->spi_handle);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to add SPI device: %d.", ret);
    return ret;
  }

  gpio_config_t io_cfg = {
      .pin_bit_mask = 1ULL << cfg->busy,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };

  ret = gpio_config(&io_cfg);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to configure BUSY pin: %d.", ret);
    return ret;
  }

  return ESP_OK;
}

static esp_err_t lora_spi_deinit(lora_t *dev)
{
  if (!dev) return ESP_ERR_INVALID_ARG;

  esp_err_t ret;

  ret = spi_bus_remove_device(dev->spi_handle);
  if (ret != ESP_OK) return ret;

  ret = spi_bus_free(dev->cfg.spi_host);
  if (ret != ESP_OK) return ret;

  return ESP_OK;
}

static esp_err_t lora_spi_transfer(lora_t *dev,
                                   lora_cmd_t cmd,
                                   const uint8_t *tx_data,
                                   size_t tx_len,
                                   uint8_t *rx_data,
                                   size_t rx_len)
{
  if (!dev) return ESP_ERR_INVALID_ARG;

  esp_err_t ret;
  size_t total_len = 1 + (tx_len > 0 ? tx_len : 0) + (rx_len > 0 ? rx_len : 0);
  uint8_t tx[total_len];
  uint8_t rx[total_len];

  memset(tx, 0, sizeof(tx));
  memset(rx, 0, sizeof(rx));

  tx[0] = cmd;

  if (tx_data && tx_len > 0) memcpy(tx + 1, tx_data, tx_len);

  spi_transaction_t t = {
      .length = total_len * 8,
      .tx_buffer = tx,
      .rx_buffer = rx,
  };

  ret = spi_device_transmit(dev->spi_handle, &t);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "SPI transfer failed: %d.", ret);
    return ret;
  }

  if (rx_data && rx_len > 0) memcpy(rx_data, rx + 1 + tx_len, rx_len);

  return ESP_OK;
}

static esp_err_t lora_spi_transfer_safe(lora_t *dev,
                                        lora_cmd_t cmd,
                                        const uint8_t *tx_data,
                                        size_t tx_len,
                                        uint8_t *rx_data,
                                        size_t rx_len)
{
  if (!dev || !dev->spi_mutex) return ESP_ERR_INVALID_ARG;

  esp_err_t ret;

  ret = lora_wait_ready(dev);
  if (ret != ESP_OK) return ret;

  if (xSemaphoreTake(dev->spi_mutex, pdMS_TO_TICKS(10)) != pdTRUE) return ESP_ERR_TIMEOUT;

  ret = lora_spi_transfer(dev, cmd, tx_data, tx_len, rx_data, rx_len);

  xSemaphoreGive(dev->spi_mutex);

  ret = lora_wait_ready(dev);
  if (ret != ESP_OK) return ret;

  return ret;
}

static esp_err_t lora_wait_ready(lora_t *dev)
{
  if (!dev) return ESP_ERR_INVALID_ARG;

  uint32_t elapsed = 0;
  while (gpio_get_level(dev->cfg.busy) == 1)
  {
    if (elapsed >= LORA_READY_TIMEOUT_MS)
    {
      ESP_LOGE(TAG, "Timeout waiting for BUSY pin to clear");
      return ESP_ERR_TIMEOUT;
    }

    vTaskDelay(pdMS_TO_TICKS(LORA_READY_POLL_INTERVAL_MS));
    elapsed += LORA_READY_POLL_INTERVAL_MS;
  }

  return ESP_OK;
}

static esp_err_t lora_cmd_get_status(lora_t *dev, uint8_t *chip_mode, uint8_t *cmd_status)
{
  if (!dev || !dev->spi_mutex) return ESP_ERR_INVALID_ARG;

  esp_err_t ret;
  uint8_t status = 0;

  if (xSemaphoreTake(dev->spi_mutex, pdMS_TO_TICKS(10)) != pdTRUE)
    return ESP_ERR_TIMEOUT;

  ret = lora_spi_transfer(dev, LORA_CMD_GET_STATUS, NULL, 0, &status, 1);

  xSemaphoreGive(dev->spi_mutex);

  if (ret != ESP_OK) return ret;

  if (chip_mode) *chip_mode = (status >> 4) & 0x07;
  if (cmd_status) *cmd_status = (status >> 1) & 0x07;

  return ESP_OK;
}

static esp_err_t lora_cmd_set_standby(lora_t *dev)
{
  if (!dev) return ESP_ERR_INVALID_ARG;

  esp_err_t ret;
  uint8_t tx[1] = {0x00}; // STDBY_RC

  ret = lora_spi_transfer_safe(dev, LORA_CMD_SET_STANDBY, tx, 1, NULL, 0);
  if (ret != ESP_OK) return ret;

  return ESP_OK;
}

static void lora_print_status(uint8_t chip_mode, uint8_t cmd_status)
{
  const char *mode_str;
  switch (chip_mode)
  {
  case 0x2: mode_str = "STBY_RC"; break;
  case 0x3: mode_str = "STBY_XOSC"; break;
  case 0x4: mode_str = "FS"; break;
  case 0x5: mode_str = "RX"; break;
  case 0x6: mode_str = "TX"; break;
  default: mode_str = "UNKNOWN"; break;
  }

  // Decode command status
  const char *status_str;
  switch (cmd_status)
  {
  case 0x2: status_str = "Data available"; break;
  case 0x3: status_str = "Command timeout"; break;
  case 0x4: status_str = "Command processing error"; break;
  case 0x5: status_str = "Failure to execute command"; break;
  case 0x6: status_str = "Command TX done"; break;
  default: status_str = "UNKNOWN"; break;
  }

  ESP_LOGI(TAG,
           "Chip mode: %s (%u), Command status: %s (%u)",
           mode_str,
           chip_mode,
           status_str,
           cmd_status);
}