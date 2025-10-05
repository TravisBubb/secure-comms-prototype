#include "lora_driver.h"
#include "board_heltec_v3.h"
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <stdbool.h>
#include <string.h>

typedef struct
{
  spi_transaction_t tran;
  uint8_t *owned_tx;
  uint8_t *owned_rx;
  bool free_tx;
  bool free_rx;
} lora_spi_command_t;

typedef enum
{
  LORA_OP_SET_STANDBY = 0x80,
  LORA_OP_SET_RF_FREQUENCY = 0x86,
  LORA_OP_SET_PACKET_TYPE = 0x8A,
} lora_opcode_t;

typedef enum
{
  LORA_STDBY_RC = 0x00,
  LORA_STDBY_XOSC = 0x01,
} lora_standby_mode_t;

typedef enum
{
  LORA_PACKET_TYPE_GFSK = 0x00,
  LORA_PACKET_TYPE_LORA = 0x01,
  LORA_PACKET_TYPE_LR_FHSS = 0x03,
} lora_packet_type_t;

static const char *TAG = "LORA_DRIVER";
static const uint32_t LORA_RF_FREQ_HZ = 915000000;  // 915 MHz
static const uint32_t LORA_FREQ_XTAL_HZ = 32000000; // 32 MHz

static volatile bool is_initialized = false;
static volatile bool driver_running = false;
static volatile bool driver_shutdown_requested = false;

static SemaphoreHandle_t spi_mutex;
static spi_device_handle_t lora_handle;
static TaskHandle_t driver_spi_task_handle;

static void lora_driver_spi_task(void *arg);
static lora_spi_command_t *lora_alloc_spi_command(const uint8_t *tx_buffer,
                                                  size_t tx_bytes,
                                                  uint8_t *rx_buffer,
                                                  size_t rx_bytes,
                                                  bool ensure_dma_safe_buffer);
static esp_err_t lora_queue_spi_command(const uint8_t *tx_buffer,
                                        size_t tx_bytes,
                                        uint8_t *rx_buffer,
                                        size_t rx_bytes,
                                        bool ensure_dma_safe_buffer);

static esp_err_t lora_wait_busy(void);
static esp_err_t lora_set_standby(lora_standby_mode_t mode);
static esp_err_t lora_set_packet_type(lora_packet_type_t type);
static esp_err_t lora_set_rf_frequency(uint32_t hz);

esp_err_t lora_driver_init(void)
{
  if (is_initialized)
  {
    ESP_LOGW(TAG, "LoRa driver already initialized.");
    return ESP_OK;
  }

  esp_err_t ret;

  spi_mutex = xSemaphoreCreateMutex();
  if (!spi_mutex)
  {
    ESP_LOGE(TAG, "Failed to create SPI mutex.");
    return ESP_ERR_NO_MEM;
  }

  spi_bus_config_t buscfg = {
      .mosi_io_num = LORA_MOSI_PIN, // connect to LoRa MOSI ("Master Out, Slave In")
      .miso_io_num = LORA_MISO_PIN, // connect to LoRa MISO ("Master In, Slave Out")
      .sclk_io_num = LORA_SCLK_PIN, // connect to LoRa SCK (Synchronization Clock)
      .quadwp_io_num = -1,          // only for quad-SPI
      .quadhd_io_num = -1,          // only for quad-SPI
      .max_transfer_sz = 0          // 0 means use default buffer size
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
      .spics_io_num = LORA_CS_PIN,       // chip select pin
      .queue_size = 8                    // minimal queue for testing
  };

  ret = spi_bus_add_device(SPI3_HOST, &devcfg, &lora_handle);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to add SPI device with status: %d.", ret);
    return ret;
  }

  driver_shutdown_requested = false;
  driver_running = true;

  if (xTaskCreate(lora_driver_spi_task,
                  "lora_driver_spi_task",
                  2048,
                  NULL,
                  tskIDLE_PRIORITY + 1,
                  &driver_spi_task_handle) != pdPASS)
  {
    ESP_LOGE(TAG, "Failed to create SPI task.");
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "SPI initialized successfully.");

  ret = lora_set_standby(LORA_STDBY_RC);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to set standby mode.");
    return ret;
  }

  ret = lora_set_packet_type(LORA_PACKET_TYPE_LORA);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to set packet type.");
    return ret;
  }

  ret = lora_set_rf_frequency(LORA_RF_FREQ_HZ);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to set RF frequency.");
    return ret;
  }

  // TODO:
  // [X] 1. `SetStandby(...)`: go to STDBY_RC mode if not already there
  // [X] 2. `SetPacketType(...)`: select LoRa protocol instead of FSK
  // [X] 3. `SetRfFrequency(...)`: set the RF frequency
  // [ ] 4. `SetPaConfig(...)`: define Power Amplifier configuration
  // [ ] 5. `SetTxParams(...)`: define output power and ramping time
  // [ ] 6. `SetModulationParams(...)`: SF, BW, CR (LoRa)
  // [ ] 7. `SetPacketParams(...)`: preamble, header mode, CRC, payload length mode
  // [ ] 8. `SetDioIrqParams(...)`: map `TxDone`/`RxDone` to DIO pins
  // [ ] 9. `WriteReg(...)`: for SyncWord if a non-default is needed

  is_initialized = true;

  return ESP_OK;
}

esp_err_t lora_driver_send(const uint8_t *data, size_t len)
{
  if (!is_initialized)
  {
    ESP_LOGE(TAG, "Cannot send, driver not initialized.");
    return ESP_ERR_INVALID_STATE;
  }

  // TODO:
  // 1. `SetBufferBaseAddress(txBase, rxBase)`: can do this once if always using the same base
  // 2. `WriteBuffer(offset, payload, length)`: put packet ito radio buffer
  // 3. `SetTx(timeout)`: start transmission
  // 4. Wait for `TxDone` (IRQ) or timeout
  // 5. `ClearIrqStatus(...)`: clear TxDone
  return ESP_OK;
}

esp_err_t lora_driver_receive(uint8_t *buffer, size_t max_len, size_t *out_len)
{
  if (!is_initialized)
  {
    ESP_LOGE(TAG, "Cannot receive, driver not initialized.");
    return ESP_ERR_INVALID_STATE;
  }

  // TODO:
  // 1. `ClearIrqStatus(...)`: clear any pending IRQs before starting a new receive
  // 2. Enter Rx mode with either:
  //     1. `SetRx(timeout)` for finite listening
  //         - Will stop and go back to `STDBY_RC` if no packet
  //     2. `SetRx(0xFFFFFF)` for continuous receive
  //         - Stays in Rx until told otherwise
  // 3. Wait for `RxDone` (IRQ) or timeout
  //     - Can use DIO1 interrupt or poll `GetIrqStatus()`
  // 4. Check CRC: Even if `RxDone`, check IRQ flags for CRC error
  //     - If CRC OK: proceed
  //     - If CRC fail: discard
  // 5. `GetRxBufferStatus(...)`: Get buffer info, gives `payloadLen` and starting offset
  // 6. `ReadBuffer(...)`: pull out the received data
  // 7. `ClearIrqStatus(...)`: clear the status (`IRQ_RX_DONE`, `IRQ_TIMEOUT`, `IRQ_CRC_ERROR`)
  // depending on what fired
  // 8. Decide what's next:
  //     - Stay in continuous Rx, chip keeps listening automatically
  //     - If timeout used, need to call `SetRx()` again in order to keep listening
  return ESP_OK;
}

esp_err_t lora_driver_deinit(void)
{
  if (!is_initialized)
  {
    ESP_LOGW(TAG, "LoRa driver had not been initialized.");
    return ESP_OK;
  }

  driver_shutdown_requested = true;

  // wait for the SPI task to fully exit
  while (driver_running)
  {
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  // tear down SPI
  spi_bus_remove_device(lora_handle);
  spi_bus_free(SPI3_HOST);
  vSemaphoreDelete(spi_mutex);

  is_initialized = false;
  return ESP_OK;
}

static void lora_driver_spi_task(void *arg)
{
  while (!driver_shutdown_requested)
  {
    spi_transaction_t *completed;
    if (spi_device_get_trans_result(lora_handle, &completed, pdMS_TO_TICKS(100)) == ESP_OK &&
        completed)
    {
      lora_spi_command_t *cmd = (lora_spi_command_t *)completed->user;
      if (cmd)
      {
        ESP_LOGD(TAG, "SPI result len %d", completed->length / 8);

        if (cmd->free_tx && cmd->owned_tx)
          heap_caps_free(cmd->owned_tx);

        if (cmd->free_rx && cmd->owned_rx)
          heap_caps_free(cmd->owned_rx);

        heap_caps_free(cmd);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1)); // short delay to yield CPU
  }

  // once shutdown is requested, drain any remaining transactions before exit
  spi_transaction_t *completed;
  while (spi_device_get_trans_result(lora_handle, &completed, pdMS_TO_TICKS(10)) == ESP_OK)
  {
    lora_spi_command_t *cmd = (lora_spi_command_t *)completed->user;
    if (cmd)
    {
      if (cmd->free_tx && cmd->owned_tx)
        heap_caps_free(cmd->owned_tx);
      if (cmd->free_rx && cmd->owned_rx)
        heap_caps_free(cmd->owned_rx);
      heap_caps_free(cmd);
    }
  }

  driver_running = false;
  vTaskDelete(NULL);
}

static esp_err_t lora_wait_busy(void)
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

static lora_spi_command_t *lora_alloc_spi_command(const uint8_t *tx_buffer,
                                                  size_t tx_bytes,
                                                  uint8_t *rx_buffer,
                                                  size_t rx_bytes,
                                                  bool ensure_dma_safe_buffer)
{
  lora_spi_command_t *cmd = heap_caps_malloc(sizeof(*cmd), MALLOC_CAP_DEFAULT);
  if (!cmd)
    return NULL;

  memset(cmd, 0, sizeof(*cmd));
  cmd->tran.length = tx_bytes * 8;
  cmd->tran.rxlength = rx_bytes * 8;
  cmd->tran.flags = 0;
  cmd->tran.user = (void *)cmd;

  if (tx_buffer && tx_bytes)
  {
    if (ensure_dma_safe_buffer)
    {
      // make a DMA-capable copy
      cmd->owned_tx = heap_caps_malloc(tx_bytes, MALLOC_CAP_DMA);
      if (!cmd->owned_tx)
        goto fail_free;
      memcpy(cmd->owned_tx, tx_buffer, tx_bytes);
      cmd->tran.tx_buffer = cmd->owned_tx;
      cmd->free_tx = true;
    }
    else
    {
      // caller promises lifetime & DMA-capable; store pointer but do not free later
      cmd->tran.tx_buffer = tx_buffer;
      cmd->free_tx = false;
    }
  }

  if (rx_buffer && rx_bytes)
  {
    cmd->owned_rx = heap_caps_malloc(rx_bytes, MALLOC_CAP_DMA);
    if (!cmd->owned_rx)
      goto fail_free;
    cmd->tran.rx_buffer = cmd->owned_rx;
    cmd->free_rx = true;
  }

  return cmd;

fail_free:
  if (cmd)
  {
    heap_caps_free(cmd->owned_tx);
    heap_caps_free(cmd->owned_rx);
    heap_caps_free(cmd);
  }
  return NULL;
}

static esp_err_t lora_queue_spi_command(const uint8_t *tx_buffer,
                                        size_t tx_bytes,
                                        uint8_t *rx_buffer,
                                        size_t rx_bytes,
                                        bool ensure_dma_safe_buffer)
{
  esp_err_t ret;
  lora_spi_command_t *cmd =
      lora_alloc_spi_command(tx_buffer, tx_bytes, rx_buffer, rx_bytes, ensure_dma_safe_buffer);
  if (!cmd)
    return ESP_ERR_NO_MEM;

  ret = lora_wait_busy();
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "An error occurred waiting for BUSY pin to go low.");
    goto err;
  }

  if (xSemaphoreTake(spi_mutex, portMAX_DELAY) == pdTRUE)
  {
    ESP_LOGD(TAG, "Sending SPI command.");
    ret = spi_device_queue_trans(lora_handle, &cmd->tran, portMAX_DELAY);
    xSemaphoreGive(spi_mutex);
    if (ret == ESP_OK)
      return ESP_OK;

    ESP_LOGE(TAG, "An error occurred attempting to queue SPI transaction.");
    goto err;
  }
  else
  {
    ESP_LOGE(TAG, "Failed to take SPI mutex.");
    goto err;
  }

  return ESP_OK;

err:
  if (cmd)
  {
    if (cmd->owned_tx)
      heap_caps_free(cmd->owned_tx);

    if (cmd->owned_rx)
      heap_caps_free(cmd->owned_rx);

    heap_caps_free(cmd);
  }
  return ESP_FAIL;
}

static esp_err_t lora_set_standby(lora_standby_mode_t mode)
{
  esp_err_t ret;
  uint8_t tx_buffer[] = {LORA_OP_SET_STANDBY, mode};

  ESP_LOGI(TAG, "Attempting to set standby mode.");

  ret = lora_queue_spi_command(tx_buffer, sizeof(tx_buffer), NULL, 0, true);
  if (ret != ESP_OK)
  {
    return ret;
  }

  ESP_LOGI(TAG, "Standby mode set.");
  return ESP_OK;
}

static esp_err_t lora_set_packet_type(lora_packet_type_t type)
{
  esp_err_t ret;
  uint8_t tx_buffer[] = {LORA_OP_SET_PACKET_TYPE, type};

  ESP_LOGI(TAG, "Attempting to set packet type.");

  ret = lora_queue_spi_command(tx_buffer, sizeof(tx_buffer), NULL, 0, true);
  if (ret != ESP_OK)
  {
    return ret;
  }

  ESP_LOGI(TAG, "Packet type set.");
  return ESP_OK;
}

static esp_err_t lora_set_rf_frequency(uint32_t hz)
{
  esp_err_t ret;
  uint32_t freq = (uint32_t)(((uint64_t)hz << 25) / LORA_FREQ_XTAL_HZ);
  uint8_t tx_buffer[] = {
      LORA_OP_SET_RF_FREQUENCY,
      (freq >> 24) & 0xFF,
      (freq >> 16) & 0xFF,
      (freq >> 8) & 0xFF,
      freq & 0xFF,
  };

  ESP_LOGI(TAG, "Attempting to set RF frequency.");

  ret = lora_queue_spi_command(tx_buffer, sizeof(tx_buffer), NULL, 0, true);
  if (ret != ESP_OK)
  {
    return ret;
  }

  ESP_LOGI(TAG, "RF frequency set to %dhz.", hz);
  return ESP_OK;
}