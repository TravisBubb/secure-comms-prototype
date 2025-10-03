#include "lora_driver.h"
#include "board_heltec_v3.h"
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <stdbool.h>

typedef enum
{
  LORA_OP_SET_STANDBY = 0x80,
} lora_opcode_t;

typedef enum
{
  LORA_STDBY_RC = 0x00,
  LORA_STDBY_XOSC = 0x01,
} lora_standby_mode_t;

static const char *TAG = "LORA_DRIVER";
static bool is_initialized = false;

static SemaphoreHandle_t spi_mutex;
static spi_device_handle_t lora_handle;
static TaskHandle_t driver_spi_task_handle;

static void lora_driver_spi_task(void *arg);
static esp_err_t lora_wait_busy(void);
static esp_err_t lora_set_standby(lora_standby_mode_t mode);

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
      .queue_size = 1                    // minimal queue for testing
  };

  ret = spi_bus_add_device(SPI3_HOST, &devcfg, &lora_handle);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to add SPI device with status: %d.", ret);
    return ret;
  }

  xTaskCreate(lora_driver_spi_task,
              "lora_driver_spi_task",
              2048,
              NULL,
              tskIDLE_PRIORITY + 1,
              &driver_spi_task_handle);

  ESP_LOGI(TAG, "SPI initialized successfully.");

  ret = lora_set_standby(LORA_STDBY_RC);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to set standby mode.");
    return ret;
  }

  // TODO:
  // [X] 1. `SetStandby(...)`: go to STDBY_RC mode if not already there
  // [ ] 2. `SetPacketType(...)`: select LoRa protocol instead of FSK
  // [ ] 3. `SetRfFrequency(...)`: set the RF frequency
  // [ ] 4. `SetPaConfig(...)`: define Power Amplifier configuration
  // [ ] 5. `SetTxParams(...)`: define output power and ramping time
  // [ ] 6. `SetModulationParams(...)`: SF, BW, CR (LoRa)
  // [ ] 7. `SetPacketParams(...)`: preamble, header mode, CRC, payload length mode (explicit/implicit)
  // [ ] 8. `SetDioIrqParams(...)`: map `TxDone`/`RxDone` to DIO pins
  // [ ] 9. `WriteReg(...)`: for SyncWord if a non-default is needed

  is_initialized = true;

  return ESP_OK;
}

esp_err_t lora_driver_send(const uint8_t *data, size_t len)
{
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

  // TODO

  is_initialized = false;
  return ESP_OK;
}

static void lora_driver_spi_task(void *arg)
{
  while (1)
  {
    spi_transaction_t *completed;
    if (spi_device_get_trans_result(lora_handle, &completed, pdMS_TO_TICKS(10)) == ESP_OK &&
        completed)
    {
      ESP_LOGI(TAG, "Got SPI result of length %d", completed->length / 8);
    }
    vTaskDelay(pdMS_TO_TICKS(1)); // short delay to yield CPU
  }
}

static esp_err_t lora_wait_busy(void)
{
  const TickType_t start = xTaskGetTickCount();
  const TickType_t timeout = pdMS_TO_TICKS(100);

  ESP_LOGI(TAG, "Waiting for BUSY pin to go low.");

  while (gpio_get_level(LORA_BUSY_PIN) == 1)
  {
    if ((xTaskGetTickCount() - start) > timeout)
    {
      ESP_LOGE(TAG, "Timeout waiting for BUSY pin to go low.");
      return ESP_ERR_TIMEOUT;
    }
    vTaskDelay(pdMS_TO_TICKS(1)); // short delay to yield CPU
  }

  ESP_LOGI(TAG, "BUSY pin is low.");

  return ESP_OK;
}

static esp_err_t lora_set_standby(lora_standby_mode_t mode)
{
  esp_err_t ret;
  uint8_t tx_buffer[] = {LORA_OP_SET_STANDBY, mode};
  uint8_t rx_buffer[] = {0};

  spi_transaction_t tran = {
      .length = sizeof(tx_buffer) * 8,
      .tx_buffer = tx_buffer,
      .rx_buffer = rx_buffer,
  };

  ret = lora_wait_busy();
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "An error occurred waiting for BUSY pin to go low.");
    return ret;
  }

  if (xSemaphoreTake(spi_mutex, portMAX_DELAY) == pdTRUE)
  {
    ESP_LOGI(TAG, "Sending SetStandby command.");
    ret = spi_device_queue_trans(lora_handle, &tran, portMAX_DELAY);
    xSemaphoreGive(spi_mutex);
    if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "An error occurred attempting to queue SPI transaction.");
      return ret;
    }
  }

  ret = lora_wait_busy();
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "An error occurred waiting for BUSY pin to go low.");
    return ret;
  }

  ESP_LOGI(TAG, "Standby mode set.");
  return ESP_OK;
}