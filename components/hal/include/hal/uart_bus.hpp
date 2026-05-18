/**
 * @file uart_bus.hpp
 * @brief UART abstraction for ESP32-S3 (N-GS-01 NavIC GNSS).
 *
 * Provides non-blocking line-oriented NMEA/UBX receive with DMA buffering.
 */
#pragma once

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include <cstdint>

namespace hal {

constexpr size_t UART_RX_BUF = 1024;
constexpr size_t UART_TX_BUF = 256;
constexpr int    UART_BAUD   = 115200;  ///< N-GS-01 default baud

class UARTBus {
public:
    UARTBus() noexcept = default;
    ~UARTBus() noexcept;

    esp_err_t init(uart_port_t port, int tx_pin, int rx_pin,
                   int baud = UART_BAUD) noexcept;

    /// Non-blocking read up to `max_len` bytes. Returns bytes read.
    int read(uint8_t* buf, size_t max_len, TickType_t timeout_ms = 0) noexcept;

    /// Write `len` bytes. Returns bytes written.
    int write(const uint8_t* data, size_t len) noexcept;

    /// Read one complete NMEA sentence (terminated by '\n'). Returns length.
    /// Returns 0 if no complete sentence available within timeout.
    int read_line(char* buf, size_t max_len, TickType_t timeout_ms = 100) noexcept;

    void flush_rx() noexcept;

    bool is_initialised() const noexcept { return initialised_; }

private:
    uart_port_t port_        = UART_NUM_1;
    bool        initialised_ = false;
};

} // namespace hal
