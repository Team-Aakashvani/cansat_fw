/**
 * @file i2c_bus.hpp
 * @brief I2C master bus abstraction for ESP32-S3 (ESP-IDF v5.x driver).
 *
 * Wraps the ESP-IDF i2c_master API with:
 *   - RAII bus handle management
 *   - Timeout-safe read/write with retry
 *   - Thread-safe via FreeRTOS mutex (one per bus)
 *   - Uniform error reporting via esp_err_t
 *
 * Two I2C buses are used:
 *   Bus 0 (I2C_NUM_0): BNO085 + BMP585         (400kHz)
 *   Bus 1 (I2C_NUM_1): SDP31 + SGP41 + SHT4x + INA260 + MAX17048  (400kHz)
 */
#pragma once

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include <cstdint>

namespace hal {

constexpr TickType_t I2C_TIMEOUT_MS = 50;   ///< Per-transaction timeout
constexpr int        I2C_RETRIES    = 3;    ///< Retry count on NAK/timeout

class I2CBus {
public:
    I2CBus() noexcept = default;
    ~I2CBus() noexcept;

    /// Initialise the bus. Returns ESP_OK on success.
    esp_err_t init(i2c_port_t port, int sda_pin, int scl_pin,
                   uint32_t speed_hz = 400000) noexcept;

    /// Write `len` bytes to device at `addr` starting at register `reg`.
    esp_err_t write_reg(uint8_t addr, uint8_t reg,
                        const uint8_t* data, size_t len) noexcept;

    /// Read `len` bytes from device at `addr` starting at register `reg`.
    esp_err_t read_reg(uint8_t addr, uint8_t reg,
                       uint8_t* buf, size_t len) noexcept;

    /// Write a single byte to a register.
    esp_err_t write_byte(uint8_t addr, uint8_t reg, uint8_t value) noexcept {
        return write_reg(addr, reg, &value, 1);
    }

    /// Read a single byte from a register.
    esp_err_t read_byte(uint8_t addr, uint8_t reg, uint8_t& out) noexcept {
        return read_reg(addr, reg, &out, 1);
    }

    /// Check if device ACKs at address.
    bool probe(uint8_t addr) noexcept;

    bool is_initialised() const noexcept { return bus_ != nullptr; }

private:
    i2c_master_bus_handle_t bus_  = nullptr;
    SemaphoreHandle_t       mutex_= nullptr;
};

} // namespace hal
