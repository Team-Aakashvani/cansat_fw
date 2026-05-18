/**
 * @file spi_bus.hpp
 * @brief SPI master bus abstraction for ESP32-S3 (ESP-IDF v5.x).
 *
 * Used exclusively by the SX1278 LoRa transceiver.
 * DMA-backed transfers for payloads > 32 bytes.
 */
#pragma once

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include <cstdint>

namespace hal {

class SPIBus {
public:
    SPIBus() noexcept = default;
    ~SPIBus() noexcept;

    /// Initialise SPI2_HOST with given MOSI/MISO/SCK pins at max_hz.
    esp_err_t init(int mosi, int miso, int sck, uint32_t max_hz = 8000000) noexcept;

    /// Add a device with given CS pin and clock speed.
    /// Returns a device handle via `dev_out`.
    esp_err_t add_device(int cs_pin, uint32_t clock_hz,
                         spi_device_handle_t& dev_out) noexcept;

    /// Transfer (full-duplex): writes `tx_len` bytes, reads `rx_len` bytes.
    esp_err_t transfer(spi_device_handle_t dev,
                       const uint8_t* tx, size_t tx_len,
                       uint8_t* rx, size_t rx_len) noexcept;

    /// Write register byte (SX1278 style: cmd | addr, then data)
    esp_err_t write_reg(spi_device_handle_t dev, uint8_t reg,
                        const uint8_t* data, size_t len) noexcept;

    /// Read register
    esp_err_t read_reg(spi_device_handle_t dev, uint8_t reg,
                       uint8_t* buf, size_t len) noexcept;

    bool is_initialised() const noexcept { return initialised_; }

private:
    bool              initialised_ = false;
    SemaphoreHandle_t mutex_       = nullptr;
};

} // namespace hal
