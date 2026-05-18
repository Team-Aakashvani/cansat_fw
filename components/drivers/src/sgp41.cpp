#include "drivers/sgp41.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
static const char* TAG = "SGP41";
namespace drivers {
static uint8_t crc8(const uint8_t* data, size_t n) noexcept {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < n; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) crc = (crc & 0x80) ? ((crc << 1) ^ 0x31) : (crc << 1);
    }
    return crc;
}
esp_err_t SGP41::init(hal::I2CBus& bus) noexcept {
    bus_ = &bus;
    if (!bus_->probe(I2C_ADDR)) { ESP_LOGE(TAG, "SGP41 not found"); return ESP_ERR_NOT_FOUND; }
    // Self-test (0x280E)
    uint8_t cmd[2] = { 0x28, 0x0E };
    bus_->write_reg(I2C_ADDR, cmd[0], cmd + 1, 1);
    vTaskDelay(pdMS_TO_TICKS(320));
    ready_ = true;
    ESP_LOGI(TAG, "SGP41 ready");
    return ESP_OK;
}
AirQualityData SGP41::read() noexcept {
    AirQualityData d{}; if (!ready_) return d;
    // Execute conditioning (0x2641) — uses default humidity compensation (50%RH, 25°C)
    // Default conditioning: humidity = 0x8000, temp = 0x6666
    const uint8_t cond[5] = { 0x26, 0x19, 0x80, 0x00, crc8((const uint8_t*)"\x80\x00", 2) };
    // Send measure raw command 0x2619
    bus_->write_reg(I2C_ADDR, cond[0], cond + 1, 4);
    vTaskDelay(pdMS_TO_TICKS(55));
    uint8_t buf[6] = {};
    if (bus_->read_reg(I2C_ADDR, 0x00, buf, 6) != ESP_OK) return d;
    // Verify CRC
    if (crc8(buf, 2) != buf[2] || crc8(buf + 3, 2) != buf[5]) return d;
    uint16_t sraw_voc = ((uint16_t)buf[0] << 8) | buf[1];
    uint16_t sraw_nox = ((uint16_t)buf[3] << 8) | buf[4];
    // Simple index: scale 0-65535 → 1-500
    d.voc_index = (uint16_t)(1 + (sraw_voc * 499UL / 65535UL));
    d.nox_index = (uint16_t)(1 + (sraw_nox * 499UL / 65535UL));
    d.valid = true;
    return d;
}
} // namespace drivers
