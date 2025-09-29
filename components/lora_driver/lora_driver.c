#include "lora_driver.h"
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_log.h>

#define NOP 0x00
#define EXPECTED_VERSION 0x0C

#ifdef CONFIG_LORA_DEBUG
#define LORA_LOG_TX_RX(tag, buf, len, dir) do {          \
    ESP_LOGI(tag, "%s:", dir);                           \
    for (int i = 0; i < len; i++) {                     \
        printf("%02X ", buf[i]);                         \
    }                                                   \
    printf("\n");                                       \
} while (0)
#else
#define LORA_LOG_TX_RX(tag, buf, len, dir) do {} while(0)
#endif


typedef enum
{
    REG_VERSION = 0x42,
} lora_reg_t;

typedef enum
{
    CMD_READ_REG = 0x1D,
} lora_cmd_t;

static esp_err_t lora_driver_spi_transmit(spi_transaction_t *t);
static esp_err_t lora_driver_read_reg(uint8_t reg, uint8_t *out);

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

    // Validate LoRa version register
    uint8_t version;
    ret = lora_driver_read_reg(REG_VERSION, &version);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read LoRa register with status: %d.", ret);
        return ret;
    }

    if (version != EXPECTED_VERSION)
    {
        ESP_LOGE(TAG, "Version mismatch: got 0x%02X, expected 0x%02X", version, EXPECTED_VERSION);
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

static esp_err_t lora_driver_spi_transmit(spi_transaction_t *t)
{
    esp_err_t ret = ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(spi_mutex, portMAX_DELAY) == pdTRUE)
    {
        ret = spi_device_transmit(lora_handle, t);
        xSemaphoreGive(spi_mutex);
    }

    return ret;
}

static esp_err_t lora_driver_read_reg(uint8_t reg, uint8_t *out)
{
    esp_err_t ret;
    uint8_t tx_buffer[] = {CMD_READ_REG, reg >> 8, reg & 0xFF, NOP, NOP};
    uint8_t rx_buffer[sizeof(tx_buffer)];

    spi_transaction_t t = {
        .length = sizeof(tx_buffer) * 8,
        .tx_buffer = tx_buffer,
        .rx_buffer = rx_buffer,
    };

    LORA_LOG_TX_RX(TAG, tx_buffer, sizeof(tx_buffer), "TX");
    ret = lora_driver_spi_transmit(&t);
    LORA_LOG_TX_RX(TAG, rx_buffer, sizeof(rx_buffer), "RX");
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read LoRa register.");
        return ret;
    }

    if (sizeof(rx_buffer) <= 4)
    {
        ESP_LOGE(TAG, "LoRa register response did not have expected size.");
        return ESP_ERR_INVALID_SIZE;
    }

    *out = rx_buffer[4];

    return ESP_OK;
}