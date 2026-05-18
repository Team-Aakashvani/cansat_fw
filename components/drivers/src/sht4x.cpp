#include "drivers/sht4x.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
static const char* TAG = "SHT4x";
namespace drivers {
esp_err_t SHT4x::init(hal::I2CBus& bus) noexcept {
    bus_ = &bus;
    // Soft reset
    const uint8_t rst = 0x94;
    bus_->write_reg(I2C_ADDR, rst, nullptr, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    ready_ = true;
    ESP_LOGI(TAG, "SHT4x ready");
    return ESP_OK;
}
HumidData SHT4x::read() noexcept {
    HumidData d{}; if (!ready_) return d;
    // Measure high-repeatability (0xFD)
    const uint8_t cmd = 0xFD;
    bus_->write_reg(I2C_ADDR, cmd, nullptr, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    uint8_t buf[6] = {};
    if (bus_->read_reg(I2C_ADDR, 0x00, buf, 6) != ESP_OK) return d;
    uint16_t t_raw = ((uint16_t)buf[0] << 8) | buf[1];
    uint16_t h_raw = ((uint16_t)buf[3] << 8) | buf[4];
    d.temperature_c = -45.0 + 175.0 * t_raw / 65535.0;
    d.humidity_pct  = -6.0  + 125.0 * h_raw / 65535.0;
    if (d.humidity_pct < 0)   d.humidity_pct = 0;
    if (d.humidity_pct > 100) d.humidity_pct = 100;
    d.valid = true;
    return d;
}
} // namespace drivers
