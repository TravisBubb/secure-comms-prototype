#include "lora_driver.h"
#include "lora_cmd.h"
#include "lora_spi.h"
#include <esp_log.h>

static const char *TAG = "LORA_DRIVER";
static const uint32_t LORA_RF_FREQ_HZ = 915000000; // 915 MHz
static volatile bool is_initialized = false;

esp_err_t lora_driver_init(void)
{
  if (is_initialized)
  {
    ESP_LOGW(TAG, "LoRa driver already initialized.");
    return ESP_OK;
  }

  esp_err_t ret;

  ret = lora_spi_init();
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "SPI initialization failed.");
    return ret;
  }

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

esp_err_t lora_driver_deinit(void)
{
  if (!is_initialized)
  {
    ESP_LOGW(TAG, "LoRa driver has not been initialized.");
    return ESP_OK;
  }

  esp_err_t ret = lora_spi_deinit();
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to de-initialize LoRa SPI.");
    return ret;
  }

  is_initialized = false;
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