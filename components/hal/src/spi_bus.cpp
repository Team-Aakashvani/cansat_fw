/**
 * @file spi_bus.cpp
 * @brief SPI bus implementation.
 */
#include "hal/spi_bus.hpp"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "SPIBus";

namespace hal {

SPIBus::~SPIBus() noexcept {
    if (mutex_) { vSemaphoreDelete(mutex_); mutex_ = nullptr; }
    if (initialised_) { spi_bus_free(SPI2_HOST); initialised_ = false; }
}

esp_err_t SPIBus::init(int mosi, int miso, int sck, uint32_t /*max_hz*/) noexcept {
    mutex_ = xSemaphoreCreateMutex();
    if (!mutex_) return ESP_ERR_NO_MEM;

    spi_bus_config_t cfg{};
    cfg.mosi_io_num   = mosi;
    cfg.miso_io_num   = miso;
    cfg.sclk_io_num   = sck;
    cfg.quadwp_io_num = -1;
    cfg.quadhd_io_num = -1;
    cfg.max_transfer_sz = 1024;
    cfg.flags = SPICOMMON_BUSFLAG_MASTER;

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Bus init: %s", esp_err_to_name(ret)); return ret; }
    initialised_ = true;
    ESP_LOGI(TAG, "SPI2 init (MOSI=%d MISO=%d SCK=%d)", mosi, miso, sck);
    return ESP_OK;
}

esp_err_t SPIBus::add_device(int cs_pin, uint32_t clock_hz,
                              spi_device_handle_t& dev_out) noexcept {
    spi_device_interface_config_t dev_cfg{};
    dev_cfg.command_bits  = 0;
    dev_cfg.address_bits  = 0;
    dev_cfg.dummy_bits    = 0;
    dev_cfg.mode          = 0;     // CPOL=0, CPHA=0 for SX1278
    dev_cfg.clock_speed_hz = (int)clock_hz;
    dev_cfg.spics_io_num  = cs_pin;
    dev_cfg.queue_size    = 4;
    return spi_bus_add_device(SPI2_HOST, &dev_cfg, &dev_out);
}

esp_err_t SPIBus::transfer(spi_device_handle_t dev,
                            const uint8_t* tx, size_t tx_len,
                            uint8_t* rx, size_t rx_len) noexcept {
    if (!initialised_) return ESP_ERR_INVALID_STATE;
    size_t total = (tx_len > rx_len) ? tx_len : rx_len;
    if (total == 0) return ESP_OK;

    if (total > 256) return ESP_ERR_INVALID_SIZE;
    uint8_t tx_buf[256];
    uint8_t rx_buf[256];

    memset(tx_buf, 0, sizeof(tx_buf));
    if (tx && tx_len) memcpy(tx_buf, tx, tx_len);

    spi_transaction_t t{};
    t.length    = total * 8;
    t.tx_buffer = tx_buf;
    t.rx_buffer = rx_buf;

    xSemaphoreTake(mutex_, portMAX_DELAY);
    esp_err_t ret = spi_device_polling_transmit(dev, &t);
    xSemaphoreGive(mutex_);

    if (ret == ESP_OK && rx && rx_len)
        memcpy(rx, rx_buf, rx_len);
    return ret;
}

esp_err_t SPIBus::write_reg(spi_device_handle_t dev, uint8_t reg,
                             const uint8_t* data, size_t len) noexcept {
    if (len > 255) return ESP_ERR_INVALID_SIZE;
    uint8_t buf[256];
    buf[0] = reg | 0x80; // Write flag for SX1278
    if (data && len > 0) {
        memcpy(buf + 1, data, len);
    }
    return transfer(dev, buf, len + 1, nullptr, 0);
}

esp_err_t SPIBus::read_reg(spi_device_handle_t dev, uint8_t reg,
                            uint8_t* buf, size_t len) noexcept {
    if (len > 255) return ESP_ERR_INVALID_SIZE;
    uint8_t tx[256];
    uint8_t rx[256];
    memset(tx, 0, sizeof(tx));
    tx[0] = reg & 0x7F; // Read flag for SX1278
    esp_err_t ret = transfer(dev, tx, len + 1, rx, len + 1);
    if (ret == ESP_OK && buf) memcpy(buf, rx + 1, len);
    return ret;
}

} // namespace hal
