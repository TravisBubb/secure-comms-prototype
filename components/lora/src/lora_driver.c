#include "lora_driver.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <stdbool.h>
#include <string.h>

#define LORA_READY_TIMEOUT_MS 100
#define LORA_READY_POLL_INTERVAL_MS 1
#define LORA_REG_SYNC_WORD_ADDR 0x0740u

static const char *TAG = "LORA";

static volatile bool tx_done_flag = false;
static SemaphoreHandle_t tx_done_sem;

void IRAM_ATTR dio1_isr_handler(void *arg)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  tx_done_flag = true;
  esp_rom_printf("DIO1 ISR fired!\n");
  xSemaphoreGiveFromISR(tx_done_sem, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

typedef enum
{
  LORA_IRQ_NONE = 0x0000, // no IRQ
  LORA_IRQ_TX_DONE = 1 << 0,
  LORA_IRQ_RX_DONE = 1 << 1,
  LORA_IRQ_PREAMBLE_DETECTED = 1 << 2,
  LORA_IRQ_SYNC_WORD_VALID = 1 << 3,
  LORA_IRQ_HEADER_VALID = 1 << 4,
  LORA_IRQ_HEADER_ERR = 1 << 5,
  LORA_IRQ_CRC_ERR = 1 << 6,
  LORA_IRQ_CAD_DONE = 1 << 7,
  LORA_IRQ_CAD_DETECTED = 1 << 8,
  LORA_IRQ_TIMEOUT = 1 << 9,
} lora_irq_t;

typedef enum
{
  LORA_CMD_CLEAR_IRQ_STATUS = 0x02,
  LORA_CMD_SET_DIO_IRQ_PARAMS = 0x08,
  LORA_CMD_WRITE_REGISTER = 0x0D,
  LORA_CMD_WRITE_BUFFER = 0x0E,
  LORA_CMD_GET_PACKET_TYPE = 0x11,
  LORA_CMD_GET_IRQ_STATUS = 0x12,
  LORA_CMD_READ_REGISTER = 0x1D,
  LORA_CMD_SET_STANDBY = 0x80,
  LORA_CMD_SET_TX = 0x83,
  LORA_CMD_SET_RF_FREQUENCY = 0x86,
  LORA_CMD_SET_PACKET_TYPE = 0x8A,
  LORA_CMD_SET_MODULATION_PARAMS = 0x8B,
  LORA_CMD_SET_PACKET_PARAMS = 0x8C,
  LORA_CMD_SET_TX_PARAMS = 0x8E,
  LORA_CMD_SET_BUFFER_BASE_ADDRESS = 0x8F,
  LORA_CMD_SET_PA_CONFIG = 0x95,
  LORA_CMD_GET_STATUS = 0xC0,
} lora_cmd_t;

static const char *lora_cmd_name(lora_cmd_t cmd)
{
  switch (cmd)
  {
  case LORA_CMD_CLEAR_IRQ_STATUS: return "ClearIrqStatus";
  case LORA_CMD_SET_DIO_IRQ_PARAMS: return "SetDioIrqParams";
  case LORA_CMD_WRITE_REGISTER: return "WriteRegister";
  case LORA_CMD_WRITE_BUFFER: return "WriteBuffer";
  case LORA_CMD_GET_PACKET_TYPE: return "GetPacketType";
  case LORA_CMD_GET_IRQ_STATUS: return "GetIrqStatus";
  case LORA_CMD_READ_REGISTER: return "ReadRegister";
  case LORA_CMD_SET_STANDBY: return "SetStandby";
  case LORA_CMD_SET_TX: return "SetTx";
  case LORA_CMD_SET_RF_FREQUENCY: return "SetRfFrequency";
  case LORA_CMD_SET_PACKET_TYPE: return "SetPacketType";
  case LORA_CMD_SET_MODULATION_PARAMS: return "SetModulationParams";
  case LORA_CMD_SET_PACKET_PARAMS: return "SetPacketParams";
  case LORA_CMD_SET_TX_PARAMS: return "SetTxParams";
  case LORA_CMD_SET_BUFFER_BASE_ADDRESS: return "SetBufferBaseAddress";
  case LORA_CMD_SET_PA_CONFIG: return "SetPaConfig";
  case LORA_CMD_GET_STATUS: return "GetStatus";
  default: return "Unknown";
  }
}

static void lora_log_buffer(const char *prefix, const uint8_t *buf, size_t len)
{
  if (!buf || len == 0)
  {
    ESP_LOGI(TAG, "%s <empty>", prefix);
    return;
  }

  char line[128];
  char *ptr = line;
  size_t remaining = sizeof(line);

  int written = snprintf(ptr, remaining, "%s ", prefix);
  ptr += written;
  remaining -= written;

  for (size_t i = 0; i < len; i++)
  {
    if (remaining < 4)
    { // each byte needs up to 3 chars + null
      ESP_LOGI(TAG, "%s", line);
      ptr = line;
      remaining = sizeof(line);
      written = snprintf(ptr, remaining, "%s ", prefix);
      ptr += written;
      remaining -= written;
    }
    written = snprintf(ptr, remaining, "%02X ", buf[i]);
    ptr += written;
    remaining -= written;
  }

  ESP_LOGI(TAG, "%s", line);
}

// Logs both TX and RX buffers in one consistent line
static void lora_log_spi_transaction(
    lora_cmd_t cmd, const uint8_t *tx_buf, size_t tx_len, const uint8_t *rx_buf, size_t rx_len)
{
  ESP_LOGI(TAG, "SPI CMD 0x%02X (%s): %u TX, %u RX", cmd, lora_cmd_name(cmd), tx_len, rx_len);

  if (tx_len > 0)
    lora_log_buffer("TX:", tx_buf, tx_len);
  else
    ESP_LOGI(TAG, "TX: <none>");

  if (rx_len > 0)
    lora_log_buffer("RX:", rx_buf, rx_len);
  else
    ESP_LOGI(TAG, "RX: <none>");
}

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
static esp_err_t lora_cmd_set_standby(lora_t *dev, uint8_t mode);
static esp_err_t lora_cmd_set_packet_type(lora_t *dev, lora_packet_type_t pkt_type);
static esp_err_t lora_cmd_get_packet_type(lora_t *dev, lora_packet_type_t *pkt_type);
static esp_err_t lora_cmd_set_rf_frequency(lora_t *dev, long frequency);
static esp_err_t lora_cmd_set_pa_config(lora_t *dev,
                                        uint8_t duty_cycle,
                                        uint8_t hp_max,
                                        lora_device_type_t device_sel);
static esp_err_t lora_cmd_set_tx_params(lora_t *dev, int8_t power, uint8_t ramp_time);
static esp_err_t lora_cmd_set_buffer_base_address(lora_t *dev, uint8_t tx_addr, uint8_t rx_addr);
static esp_err_t
lora_cmd_set_modulation_params(lora_t *dev, uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro);
static esp_err_t lora_cmd_set_dio_irq_params(
    lora_t *dev, uint16_t irq_mask, uint16_t dio1_mask, uint16_t dio2_mask, uint16_t dio3_mask);
static esp_err_t
lora_cmd_write_register(lora_t *dev, uint16_t addr, const uint8_t *data, size_t len);
static esp_err_t lora_cmd_read_register(lora_t *dev, uint16_t addr, uint8_t *data, size_t len);
static esp_err_t lora_cmd_set_syncword(lora_t *dev, uint16_t syncword);
static esp_err_t
lora_cmd_write_buffer(lora_t *dev, uint8_t offset, const uint8_t *data, size_t len);
static esp_err_t lora_cmd_set_packet_params(lora_t *dev,
                                            uint16_t preamble_len,
                                            uint8_t header_type,
                                            uint8_t payload_len,
                                            uint8_t crc,
                                            uint8_t invert_iq);
static esp_err_t lora_cmd_set_tx(lora_t *dev, uint32_t timeout);
static esp_err_t lora_wait_for_tx_done(lora_t *dev, uint32_t timeout_ms);
static esp_err_t lora_cmd_get_irq_status(lora_t *dev, uint16_t *irq);
static esp_err_t lora_cmd_clear_irq_status(lora_t *dev, uint16_t mask);
static void lora_print_status(uint8_t chip_mode, uint8_t cmd_status);

esp_err_t lora_init(lora_t *dev, const lora_config_t *cfg)
{
  if (!dev || !cfg) return ESP_ERR_INVALID_ARG;

  esp_err_t ret;

  if (cfg->reset >= 0)
  {
    gpio_set_direction(cfg->reset, GPIO_MODE_OUTPUT);
    gpio_set_level(cfg->reset, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(cfg->reset, 1);
    vTaskDelay(pdMS_TO_TICKS(10)); // give it time to boot
    // extra wait for oscillator if needed
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  vTaskDelay(pdMS_TO_TICKS(50));

  tx_done_sem = xSemaphoreCreateBinary();
  gpio_config_t dio_cfg = {
      .pin_bit_mask = 1ULL << cfg->dio1,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_POSEDGE,
  };

  ret = gpio_config(&dio_cfg);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to configure DIO1 pin: %d.", ret);
    return ret;
  }

  ret = gpio_install_isr_service(0);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
  {
    ESP_LOGE(TAG, "gpio_install_isr_service failed: %d", ret);
    return ret;
  }
  ret = gpio_isr_handler_add(cfg->dio1, dio1_isr_handler, NULL);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "gpio_isr_handler_add failed: %d", ret);
    return ret;
  }

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
  ret = lora_cmd_set_standby(dev, 0x00);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to set LoRa to standby mode: %d.", ret);
    return ret;
  }
  ESP_LOGI(TAG, "LoRa set to standby mode.");

  ESP_LOGI(TAG, "Setting LoRa packet type (%d)...", LORA_PACKET_TYPE_LORA);
  ret = lora_cmd_set_packet_type(dev, LORA_PACKET_TYPE_LORA);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to set LoRa packet type: %d.", ret);
    return ret;
  }
  ESP_LOGI(TAG, "LoRa packet type set (%d).", LORA_PACKET_TYPE_LORA);

  ESP_LOGI(TAG, "Verifying LoRa packet type...");
  lora_packet_type_t pkt_type = -1;
  int retries = 5;
  do
  {
    ret = lora_cmd_get_packet_type(dev, &pkt_type);
    if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to get LoRa packet type: %d.", ret);
      return ret;
    }
    if (pkt_type == LORA_PACKET_TYPE_LORA) break;
  } while (--retries);

  if (pkt_type != LORA_PACKET_TYPE_LORA)
  {
    ESP_LOGE(TAG,
             "Packet type verification failed (expected 0x%02X, got 0x%02X).",
             LORA_PACKET_TYPE_LORA,
             pkt_type);
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "LoRa packet type verified (0x%02X).", pkt_type);

  ESP_LOGI(TAG, "Setting RF frequency (%ld)...", cfg->frequency);
  ret = lora_cmd_set_rf_frequency(dev, cfg->frequency);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to set RF frequency: %d.", ret);
    return ret;
  }
  ESP_LOGI(TAG, "RF frequency set (%ld).", cfg->frequency);

  ESP_LOGI(TAG,
           "Setting PA config: duty_cycle (0x%02X) hp_max (0x%02X) device_sel (0x%02X)...",
           cfg->duty_cycle,
           cfg->hp_max,
           cfg->device_sel);
  ret = lora_cmd_set_pa_config(dev, cfg->duty_cycle, cfg->hp_max, cfg->device_sel);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to set PA config: %d.", ret);
    return ret;
  }
  ESP_LOGI(TAG, "PA config set.");

  ESP_LOGI(
      TAG, "Setting TX params: power (%ddBm) ramp_time (0x%02X)...", cfg->tx_power, cfg->ramp_time);
  ret = lora_cmd_set_tx_params(dev, cfg->tx_power, cfg->ramp_time);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to set TX params: %d.", ret);
    return ret;
  }
  ESP_LOGI(TAG, "TX params set.");

  ESP_LOGI(TAG, "Setting buffer base addresses: TX (0x%02X) RX (0x%02X)...", 0x00, 0x00);
  ret = lora_cmd_set_buffer_base_address(dev, 0x00, 0x00);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to set buffer base addresses: %d.", ret);
    return ret;
  }
  ESP_LOGI(TAG, "Buffer base addresses set.");

  ESP_LOGI(TAG,
           "Setting modulation params: SF (0x%02X) BW (0x%02X) CR (0x%02X) LDRO (0x%02X)...",
           cfg->sf,
           cfg->bw,
           cfg->cr,
           cfg->ldro);
  ret = lora_cmd_set_modulation_params(dev, cfg->sf, cfg->bw, cfg->cr, cfg->ldro);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to set modulation params: %d.", ret);
    return ret;
  }
  ESP_LOGI(TAG, "Modulation params set.");

  uint16_t irq_mask = LORA_IRQ_TX_DONE | LORA_IRQ_RX_DONE | LORA_IRQ_TIMEOUT;
  uint16_t dio1_mask = LORA_IRQ_TX_DONE | LORA_IRQ_RX_DONE;
  ESP_LOGI(TAG,
           "Setting DIO IRQ params: irq_mask (0x%02X) dio1 (0x%02X) dio2 (0x%02X) dio3 (0x%02X)...",
           irq_mask,
           dio1_mask,
           LORA_IRQ_NONE,
           LORA_IRQ_NONE);
  lora_cmd_clear_irq_status(dev, 0xFFFF);
  ret = lora_cmd_set_dio_irq_params(dev, irq_mask, dio1_mask, LORA_IRQ_NONE, LORA_IRQ_NONE);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to set DIO IRQ params: %d.", ret);
    return ret;
  }
  ESP_LOGI(TAG, "DIO IRQ params set.");

  ESP_LOGI(TAG, "Setting syncword: 0x%02X", cfg->syncword);
  ret = lora_cmd_set_syncword(dev, cfg->syncword);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to set syncword: %d.", ret);
    return ret;
  }
  ESP_LOGI(TAG, "Sync word set.");

  ESP_LOGI(TAG, "Verifying syncword...");
  uint8_t buf[2] = {0};
  ret = lora_cmd_read_register(dev, LORA_REG_SYNC_WORD_ADDR, buf, 2);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to get syncword: %d.", ret);
    return ret;
  }

  uint16_t syncword = ((uint16_t)buf[0] << 8) | buf[1];
  if (syncword != cfg->syncword)
  {
    ESP_LOGE(TAG, "Syncword failed (expected 0x%02X, got 0x%02X).", cfg->syncword, syncword);
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "Syncword verified (0x%02X).", syncword);

  return ESP_OK;
}

esp_err_t lora_send(lora_t *dev, const uint8_t *data, size_t len)
{
  if (!dev || !data || len == 0) return ESP_ERR_INVALID_ARG;

  esp_err_t ret;

  ESP_LOGI(TAG, "Beginning TX sequence...");

  ESP_LOGI(TAG, "Writing payload to TX buffer...");
  ret = lora_cmd_write_buffer(dev, 0x00, data, len);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to write payload to TX buffer: %d.", ret);
    return ret;
  }
  ESP_LOGI(TAG, "Payload has been written to TX buffer.");

  ESP_LOGI(TAG, "Setting packet params...");
  ret = lora_cmd_set_packet_params(dev,
                                   8,    // preamble
                                   0x00, // variable length
                                   len,
                                   0x01,  // CRC on
                                   0x00); // standard IQ
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to set packet params: %d.", ret);
    return ret;
  }
  ESP_LOGI(TAG, "Packet params have been set.");

  ESP_LOGI(TAG, "Setting TX mode...");
  uint32_t timeout_ms = 5000;
  ret = lora_cmd_set_tx(dev, timeout_ms);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to set TX mode: %d.", ret);
    return ret;
  }
  ESP_LOGI(TAG, "TX set.");

  uint8_t chip_mode = 0, cmd_status = 0;
  if (lora_cmd_get_status(dev, &chip_mode, &cmd_status) == ESP_OK) {
    lora_print_status(chip_mode, cmd_status);
  } else {
    ESP_LOGE(TAG, "Failed to GET_STATUS after SetTx");
  }

  uint16_t irq = 0;
  if (lora_cmd_get_irq_status(dev, &irq) == ESP_OK) {
    ESP_LOGI(TAG, "IRQ status after SetTx: 0x%04X", irq);
  } else {
    ESP_LOGE(TAG, "Failed to GET_IRQ_STATUS after SetTx");
  }

  // Clear them so a subsequent attempt is clean
  lora_cmd_clear_irq_status(dev, 0xFFFF);

  if (xSemaphoreTake(tx_done_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE)
  {
    if (tx_done_flag)
    {
      tx_done_flag = false; // reset
      ESP_LOGI("LORA_ISR", "TX_DONE fired!");
    }
  }
  else
  {
    ESP_LOGE("LORA_ISR", "TX timeout!");
  }

  // ESP_LOGI(TAG, "Waiting for TX_DONE IRQ.");
  // ret = lora_wait_for_tx_done(dev, 500);
  // if (ret != ESP_OK)
  // {
  //   ESP_LOGE(TAG, "Error while waiting for TX_DONE: %d.", ret);
  //   return ret;
  // }
  // ESP_LOGI(TAG, "Transmission completed successfully.");

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
  size_t total_len = 1 + tx_len + rx_len;
  if (total_len > 64) return ESP_ERR_INVALID_SIZE; // sanity limit

  uint8_t tx_buf[64] = {0};
  uint8_t rx_buf[64] = {0};

  tx_buf[0] = cmd;
  if (tx_data && tx_len > 0) memcpy(&tx_buf[1], tx_data, tx_len);

  spi_transaction_t t = {
      .length = total_len * 8,
      .tx_buffer = tx_buf,
      .rx_buffer = rx_buf,
  };

  ret = spi_device_transmit(dev->spi_handle, &t);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "SPI transfer failed: %d", ret);
    return ret;
  }

  if (rx_data && rx_len > 0) memcpy(rx_data, &rx_buf[1 + tx_len], rx_len);

  lora_log_spi_transaction(cmd, tx_buf, 1 + tx_len, rx_buf, 1 + tx_len + rx_len);

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

  esp_rom_delay_us(100); // small delay to let internal state settle

  return ret;
}

static esp_err_t lora_wait_ready(lora_t *dev)
{
  vTaskDelay(pdMS_TO_TICKS(100));
  return ESP_OK;

  if (!dev) return ESP_ERR_INVALID_ARG;

  // Quick probe: read BUSY once and log it to confirm polarity
  int level = gpio_get_level(dev->cfg.busy);
  ESP_LOGI(TAG, "BUSY level=%d (expect 1 while busy for normal boards)", level);

  unsigned int elapsed = 0;
  while (gpio_get_level(dev->cfg.busy) == 1) // keep your existing logic, but watch the logs
  {
    if (elapsed >= LORA_READY_TIMEOUT_MS)
    {
      ESP_LOGE(TAG, "Timeout waiting for BUSY pin to clear (elapsed=%u ms)", elapsed);
      return ESP_ERR_TIMEOUT;
    }
    vTaskDelay(pdMS_TO_TICKS(LORA_READY_POLL_INTERVAL_MS));
    elapsed += LORA_READY_POLL_INTERVAL_MS;
  }
  return ESP_OK;
}

// static esp_err_t lora_wait_ready(lora_t *dev)
// {
//   if (!dev) return ESP_ERR_INVALID_ARG;

//   uint32_t elapsed = 0;
//   while (gpio_get_level(dev->cfg.busy) == 1)
//   {
//     if (elapsed >= LORA_READY_TIMEOUT_MS)
//     {
//       ESP_LOGE(TAG, "Timeout waiting for BUSY pin to clear");
//       return ESP_ERR_TIMEOUT;
//     }

//     vTaskDelay(pdMS_TO_TICKS(LORA_READY_POLL_INTERVAL_MS));
//     elapsed += LORA_READY_POLL_INTERVAL_MS;
//   }

//   return ESP_OK;
// }

static esp_err_t lora_cmd_get_status(lora_t *dev, uint8_t *chip_mode, uint8_t *cmd_status)
{
  if (!dev) return ESP_ERR_INVALID_ARG;

  esp_err_t ret;
  uint8_t tx_buf[1] = {LORA_CMD_GET_STATUS};
  uint8_t rx_buf[1] = {0};

  if (xSemaphoreTake(dev->spi_mutex, pdMS_TO_TICKS(10)) != pdTRUE) return ESP_ERR_TIMEOUT;

  spi_transaction_t t = {0};
  t.length = 8; // 1 byte
  t.tx_buffer = tx_buf;
  t.rx_buffer = rx_buf;

  ret = spi_device_transmit(dev->spi_handle, &t);
  xSemaphoreGive(dev->spi_mutex);
  if (ret != ESP_OK) return ret;

  uint8_t status = rx_buf[0];

  if (chip_mode) *chip_mode = (status >> 4) & 0x07;
  if (cmd_status) *cmd_status = (status >> 1) & 0x07;

  return ESP_OK;
}

static esp_err_t lora_cmd_set_standby(lora_t *dev, uint8_t mode)
{
  if (!dev) return ESP_ERR_INVALID_ARG;

  uint8_t tx[1] = {mode};

  return lora_spi_transfer_safe(dev, LORA_CMD_SET_STANDBY, tx, 1, NULL, 0);
}

static esp_err_t lora_cmd_set_packet_type(lora_t *dev, lora_packet_type_t pkt_type)
{
  if (!dev) return ESP_ERR_INVALID_ARG;

  uint8_t tx[1] = {(uint8_t)pkt_type};

  return lora_spi_transfer_safe(dev, LORA_CMD_SET_PACKET_TYPE, tx, 1, NULL, 0);
}

static esp_err_t lora_cmd_get_packet_type(lora_t *dev, lora_packet_type_t *pkt_type)
{
  if (!dev) return ESP_ERR_INVALID_ARG;

  esp_err_t ret;
  uint8_t rx[2] = {0}; // 1 status + 1 data

  ret = lora_spi_transfer_safe(dev, LORA_CMD_GET_PACKET_TYPE, NULL, 0, rx, sizeof(rx));
  if (ret != ESP_OK) return ret;

  *pkt_type = (lora_packet_type_t)rx[1]; // skip status byte
  return ESP_OK;
}

static esp_err_t lora_cmd_set_rf_frequency(lora_t *dev, long frequency)
{
  if (!dev) return ESP_ERR_INVALID_ARG;

  uint64_t frf = ((uint64_t)frequency << 25) / 32000000;
  uint8_t tx[4] = {
      (uint8_t)((frf >> 24) & 0xFF),
      (uint8_t)((frf >> 16) & 0xFF),
      (uint8_t)((frf >> 8) & 0xFF),
      (uint8_t)(frf & 0xFF),
  };

  return lora_spi_transfer_safe(dev, LORA_CMD_SET_RF_FREQUENCY, tx, sizeof(tx), NULL, 0);
}

static esp_err_t lora_cmd_set_pa_config(lora_t *dev,
                                        uint8_t duty_cycle,
                                        uint8_t hp_max,
                                        lora_device_type_t device_sel)
{
  if (!dev) return ESP_ERR_INVALID_ARG;

  uint8_t tx[4] = {
      duty_cycle,
      hp_max,
      (uint8_t)device_sel,
      0x01, // PA Lut, always 0x01
  };

  return lora_spi_transfer_safe(dev, LORA_CMD_SET_PA_CONFIG, tx, sizeof(tx), NULL, 0);
}

static esp_err_t lora_cmd_set_tx_params(lora_t *dev, int8_t tx_power, uint8_t ramp_time)
{
  if (!dev) return ESP_ERR_INVALID_ARG;

  uint8_t tx[2] = {
      (uint8_t)tx_power,
      ramp_time,
  };

  return lora_spi_transfer_safe(dev, LORA_CMD_SET_TX_PARAMS, tx, sizeof(tx), NULL, 0);
}

static esp_err_t lora_cmd_set_buffer_base_address(lora_t *dev, uint8_t tx_addr, uint8_t rx_addr)
{
  if (!dev) return ESP_ERR_INVALID_ARG;

  uint8_t tx[2] = {
      tx_addr,
      rx_addr,
  };

  return lora_spi_transfer_safe(dev, LORA_CMD_SET_BUFFER_BASE_ADDRESS, tx, sizeof(tx), NULL, 0);
}

static esp_err_t
lora_cmd_set_modulation_params(lora_t *dev, uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro)
{
  if (!dev) return ESP_ERR_INVALID_ARG;

  uint8_t tx[4] = {sf, bw, cr, ldro};

  return lora_spi_transfer_safe(dev, LORA_CMD_SET_MODULATION_PARAMS, tx, sizeof(tx), NULL, 0);
}

static esp_err_t lora_cmd_set_dio_irq_params(
    lora_t *dev, uint16_t irq_mask, uint16_t dio1_mask, uint16_t dio2_mask, uint16_t dio3_mask)
{
  if (!dev) return ESP_ERR_INVALID_ARG;

  uint8_t tx[8] = {
      (uint8_t)(irq_mask >> 8),
      (uint8_t)(irq_mask >> 0),
      (uint8_t)(dio1_mask >> 8),
      (uint8_t)(dio1_mask >> 0),
      (uint8_t)(dio2_mask >> 8),
      (uint8_t)(dio2_mask >> 0),
      (uint8_t)(dio3_mask >> 8),
      (uint8_t)(dio3_mask >> 0),
  };

  return lora_spi_transfer_safe(dev, LORA_CMD_SET_DIO_IRQ_PARAMS, tx, sizeof(tx), NULL, 0);
}

static esp_err_t
lora_cmd_write_register(lora_t *dev, uint16_t addr, const uint8_t *data, size_t len)
{
  if (!dev || (!data && len > 0)) return ESP_ERR_INVALID_ARG;
  if (len == 0) return ESP_OK;

  uint8_t tx[2 + len];
  tx[0] = (uint8_t)(addr >> 8);   // addr MSB
  tx[1] = (uint8_t)(addr & 0xFF); // addr LSB
  memcpy(&tx[2], data, len);

  return lora_spi_transfer_safe(dev, LORA_CMD_WRITE_REGISTER, tx, sizeof(tx), NULL, 0);
}

static esp_err_t lora_cmd_read_register(lora_t *dev, uint16_t addr, uint8_t *data, size_t len)
{
  if (!dev || !data || len == 0) return ESP_ERR_INVALID_ARG;

  uint8_t tx[3]; // 2-byte address + 1 dummy
  tx[0] = (uint8_t)(addr >> 8);
  tx[1] = (uint8_t)(addr & 0xFF);
  tx[2] = 0x00;

  uint8_t rx[len];

  esp_err_t ret =
      lora_spi_transfer_safe(dev, LORA_CMD_READ_REGISTER, tx, sizeof(tx), rx, sizeof(rx));
  if (ret != ESP_OK) return ret;

  memcpy(data, rx, len);
  return ESP_OK;
}

static esp_err_t lora_cmd_set_syncword(lora_t *dev, uint16_t syncword)
{
  if (!dev) return ESP_ERR_INVALID_ARG;

  uint8_t buf[2] = {(uint8_t)(syncword >> 8), (uint8_t)(syncword & 0xFF)};

  return lora_cmd_write_register(dev, LORA_REG_SYNC_WORD_ADDR, buf, sizeof(buf));
}

static esp_err_t lora_cmd_write_buffer(lora_t *dev, uint8_t offset, const uint8_t *data, size_t len)
{
  if (!dev || !data || len == 0) return ESP_ERR_INVALID_ARG;

  uint8_t tx[1 + len];
  tx[0] = offset;
  memcpy(&tx[1], data, len);

  return lora_spi_transfer_safe(dev, LORA_CMD_WRITE_BUFFER, tx, sizeof(tx), NULL, 0);
}

static esp_err_t lora_cmd_set_packet_params(lora_t *dev,
                                            uint16_t preamble_len,
                                            uint8_t header_type,
                                            uint8_t payload_len,
                                            uint8_t crc,
                                            uint8_t invert_iq)
{
  if (!dev) return ESP_ERR_INVALID_ARG;

  uint8_t tx[6] = {
      (uint8_t)(preamble_len >> 8),
      (uint8_t)(preamble_len & 0xFF),
      header_type,
      payload_len,
      crc,
      invert_iq,
  };

  return lora_spi_transfer_safe(dev, LORA_CMD_SET_PACKET_PARAMS, tx, sizeof(tx), NULL, 0);
}

static esp_err_t lora_cmd_set_tx(lora_t *dev, uint32_t timeout)
{
  if (!dev) return ESP_ERR_INVALID_ARG;

  uint8_t tx[3] = {
      (uint8_t)((timeout >> 16) & 0xFF),
      (uint8_t)((timeout >> 8) & 0xFF),
      (uint8_t)(timeout & 0xFF),
  };

  return lora_spi_transfer_safe(dev, LORA_CMD_SET_TX, tx, sizeof(tx), NULL, 0);
}

static esp_err_t lora_wait_for_tx_done(lora_t *dev, uint32_t timeout_ms)
{
  uint16_t irq;
  uint32_t start = esp_log_timestamp();

  while (esp_log_timestamp() - start < timeout_ms)
  {
    esp_err_t ret = lora_cmd_get_irq_status(dev, &irq);
    if (ret != ESP_OK) return ret;

    if (irq & 0x0001)
    {
      ESP_LOGI(TAG, "TX_DONE detected!");
      lora_cmd_clear_irq_status(dev, irq);
      return ESP_OK;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  ESP_LOGE(TAG, "TX timed out waiting for TX_DONE");
  return ESP_ERR_TIMEOUT;
}

static esp_err_t lora_cmd_get_irq_status(lora_t *dev, uint16_t *irq)
{
  uint8_t rx[4];
  esp_err_t ret = lora_spi_transfer_safe(dev, LORA_CMD_GET_IRQ_STATUS, NULL, 0, rx, sizeof(rx));
  if (ret != ESP_OK) return ret;

  *irq = ((uint16_t)rx[2] << 8) | rx[3];
  return ESP_OK;
}

static esp_err_t lora_cmd_clear_irq_status(lora_t *dev, uint16_t mask)
{
  uint8_t tx[2] = {(uint8_t)(mask >> 8), (uint8_t)(mask & 0xFF)};
  return lora_spi_transfer_safe(dev, LORA_CMD_CLEAR_IRQ_STATUS, tx, sizeof(tx), NULL, 0);
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