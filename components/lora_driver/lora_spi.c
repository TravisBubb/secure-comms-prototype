#include "lora_spi.h"
#include "board_heltec_v3.h"
#include "lora_hw.h"
#include <driver/spi_master.h>
#include <esp_log.h>
#include <string.h>

static const char *TAG = "LORA_SPI";
static volatile bool is_initialized = false;
static volatile bool driver_running = false;
static volatile bool driver_shutdown_requested = false;

static SemaphoreHandle_t spi_mutex;
static spi_device_handle_t lora_handle;
static TaskHandle_t driver_spi_task_handle;

typedef struct
{
  spi_transaction_t tran;
  uint8_t *owned_tx;
  uint8_t *owned_rx;
  bool free_tx;
  bool free_rx;
} lora_spi_command_t;

static void lora_driver_spi_task(void *arg);
static lora_spi_command_t *lora_alloc_spi_command(const uint8_t *tx_buffer,
                                                  size_t tx_bytes,
                                                  uint8_t *rx_buffer,
                                                  size_t rx_bytes,
                                                  bool ensure_dma_safe_buffer);

esp_err_t lora_spi_init(void)
{
  if (is_initialized)
  {
    ESP_LOGW(TAG, "LoRa SPI already initialized.");
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
  return ESP_OK;
}

esp_err_t lora_spi_deinit(void)
{
  if (!is_initialized)
  {
    ESP_LOGW(TAG, "LoRa SPI has not been initialized.");
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

esp_err_t lora_spi_send(const uint8_t *tx_buffer,
                        size_t tx_bytes,
                        uint8_t *rx_buffer,
                        size_t rx_bytes,
                        bool ensure_dma_safe)
{
  esp_err_t ret;
  lora_spi_command_t *cmd =
      lora_alloc_spi_command(tx_buffer, tx_bytes, rx_buffer, rx_bytes, ensure_dma_safe);

  if (!cmd)
    return ESP_ERR_NO_MEM;

  ret = lora_hw_wait_busy();
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