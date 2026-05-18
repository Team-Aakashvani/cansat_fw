#include "hal/uart_bus.hpp"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "UARTBus";

namespace hal {

UARTBus::~UARTBus() noexcept {
    if (initialised_) { uart_driver_delete(port_); initialised_ = false; }
}

esp_err_t UARTBus::init(uart_port_t port, int tx_pin, int rx_pin, int baud) noexcept {
    port_ = port;
    uart_config_t cfg{};
    cfg.baud_rate  = baud;
    cfg.data_bits  = UART_DATA_8_BITS;
    cfg.parity     = UART_PARITY_DISABLE;
    cfg.stop_bits  = UART_STOP_BITS_1;
    cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    esp_err_t ret = uart_param_config(port, &cfg);
    if (ret != ESP_OK) return ret;
    ret = uart_set_pin(port, tx_pin, rx_pin, -1, -1);
    if (ret != ESP_OK) return ret;
    ret = uart_driver_install(port, UART_RX_BUF, UART_TX_BUF, 0, nullptr, 0);
    if (ret != ESP_OK) return ret;
    uart_enable_pattern_det_baud_intr(port, '\n', 1, 9, 0, 0);
    uart_pattern_queue_reset(port, 8);
    initialised_ = true;
    ESP_LOGI(TAG, "UART%d init (TX=%d RX=%d @%d)", (int)port, tx_pin, rx_pin, baud);
    return ESP_OK;
}

int UARTBus::read(uint8_t* buf, size_t max_len, TickType_t timeout_ms) noexcept {
    if (!initialised_) return 0;
    return uart_read_bytes(port_, buf, max_len, pdMS_TO_TICKS(timeout_ms));
}

int UARTBus::write(const uint8_t* data, size_t len) noexcept {
    if (!initialised_) return 0;
    return uart_write_bytes(port_, (const char*)data, len);
}

int UARTBus::read_line(char* buf, size_t max_len, TickType_t timeout_ms) noexcept {
    if (!initialised_) return 0;
    int pos = uart_pattern_pop_pos(port_);
    if (pos < 0) {
        // Poll for newline
        uint8_t tmp;
        int n = 0;
        TickType_t t0 = xTaskGetTickCount();
        while ((int)(xTaskGetTickCount() - t0) < (int)pdMS_TO_TICKS(timeout_ms)) {
            if (uart_read_bytes(port_, &tmp, 1, 1) == 1) {
                if (n < (int)max_len - 1) buf[n++] = (char)tmp;
                if (tmp == '\n') { buf[n] = '\0'; return n; }
            }
        }
        return 0;
    }
    int n = uart_read_bytes(port_, (uint8_t*)buf, pos + 1,
                            pdMS_TO_TICKS(timeout_ms));
    if (n > 0) buf[n] = '\0';
    return n;
}

void UARTBus::flush_rx() noexcept {
    if (initialised_) uart_flush_input(port_);
}

} // namespace hal
