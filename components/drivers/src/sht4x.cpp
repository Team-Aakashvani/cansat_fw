#include "drivers/sht4x.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
static const char* TAG = "SHT4x";
namespace drivers {

static uint8_t crc8(const uint8_t* data, size_t len) noexcept {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x80) ? ((crc << 1) ^ 0x31) : (crc << 1);
        }
    }
    return crc;
}

esp_err_t SHT4x::init(hal::I2CBus& bus) noexcept {
    bus_ = &bus;
    // Soft reset (0x94)
    const uint8_t rst = 0x94;
    if (bus_->write_raw(I2C_ADDR, &rst, 1) != ESP_OK) {
        ESP_LOGW(TAG, "SHT4x not found at 0x%02X (ABSENT)", I2C_ADDR);
        ready_ = false;
        return ESP_ERR_NOT_FOUND;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    ready_ = true;
    ESP_LOGI(TAG, "SHT4x initialised at 0x%02X", I2C_ADDR);
    return ESP_OK;
}

HumidData SHT4x::read() noexcept {
    HumidData d{};
    if (!ready_ || !bus_) return d;

    // Measure high-repeatability (0xFD)
    const uint8_t cmd = 0xFD;
    if (bus_->write_raw(I2C_ADDR, &cmd, 1) != ESP_OK) return d;
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t buf[6] = {};
    if (bus_->read_raw(I2C_ADDR, buf, 6) != ESP_OK) return d;

    // Verify CRC of temperature and humidity packets
    if (crc8(buf, 2) != buf[2] || crc8(buf + 3, 2) != buf[5]) {
        ESP_LOGW(TAG, "CRC error on SHT4x read");
        return d;
    }

    uint16_t t_raw = ((uint16_t)buf[0] << 8) | buf[1];
    uint16_t h_raw = ((uint16_t)buf[3] << 8) | buf[4];
    d.temperature_c = -45.0 + 175.0 * (double)t_raw / 65535.0;
    d.humidity_pct  = -6.0  + 125.0 * (double)h_raw / 65535.0;
    if (d.humidity_pct < 0.0)   d.humidity_pct = 0.0;
    if (d.humidity_pct > 100.0) d.humidity_pct = 100.0;
    d.valid = true;
    return d;
}
} // namespace drivers
