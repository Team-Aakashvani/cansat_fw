#include "drivers/sgp41.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>
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
    // Self-test (0x280E)
    uint8_t cmd[2] = { 0x28, 0x0E };
    if (bus_->write_raw(I2C_ADDR, cmd, 2) != ESP_OK) {
        ESP_LOGE(TAG, "SGP41 not found at 0x%02X", I2C_ADDR);
        ready_ = false;
        return ESP_ERR_NOT_FOUND;
    }
    vTaskDelay(pdMS_TO_TICKS(330));
    uint8_t res[3] = {};
    if (bus_->read_raw(I2C_ADDR, res, 3) == ESP_OK) {
        uint16_t code = ((uint16_t)res[0] << 8) | res[1];
        ESP_LOGI(TAG, "SGP41 self-test result: 0x%04X (0xD400=pass)", code);
    }
    ready_ = true;
    ESP_LOGI(TAG, "SGP41 ready at 0x%02X", I2C_ADDR);
    return ESP_OK;
}

AirQualityData SGP41::read(double temp_c, double rh_pct) noexcept {
    AirQualityData d{};
    if (!ready_ || !bus_) return d;

    if (rh_pct < 0.0) rh_pct = 0.0;
    if (rh_pct > 100.0) rh_pct = 100.0;
    if (temp_c < -45.0) temp_c = -45.0;
    if (temp_c > 130.0) temp_c = 130.0;

    uint16_t rh_ticks = (uint16_t)(rh_pct * 65535.0 / 100.0);
    uint16_t t_ticks  = (uint16_t)((temp_c + 45.0) * 65535.0 / 175.0);

    // Command: sgp41_measure_raw_signals (0x2619) with 2 arguments: RH + CRC, T + CRC
    uint8_t cmd[8];
    cmd[0] = 0x26;
    cmd[1] = 0x19;
    cmd[2] = (rh_ticks >> 8) & 0xFF;
    cmd[3] = rh_ticks & 0xFF;
    cmd[4] = crc8(&cmd[2], 2);
    cmd[5] = (t_ticks >> 8) & 0xFF;
    cmd[6] = t_ticks & 0xFF;
    cmd[7] = crc8(&cmd[5], 2);

    if (bus_->write_raw(I2C_ADDR, cmd, 8) != ESP_OK) return d;
    vTaskDelay(pdMS_TO_TICKS(50));

    uint8_t buf[6] = {};
    if (bus_->read_raw(I2C_ADDR, buf, 6) != ESP_OK) return d;

    // Verify CRC
    if (crc8(buf, 2) != buf[2] || crc8(buf + 3, 2) != buf[5]) {
        ESP_LOGW(TAG, "CRC error on SGP41 read");
        return d;
    }

    uint16_t sraw_voc = ((uint16_t)buf[0] << 8) | buf[1];
    uint16_t sraw_nox = ((uint16_t)buf[3] << 8) | buf[4];

    d.sraw_voc = sraw_voc;
    d.sraw_nox = sraw_nox;

    // Sensirion Gas Index Calibration Curves
    if (!baseline_init_) {
        voc_baseline_ = (float)sraw_voc;
        nox_baseline_ = (float)sraw_nox;
        baseline_init_ = true;
    } else {
        // Slow adaptive moving baseline
        voc_baseline_ = 0.998f * voc_baseline_ + 0.002f * (float)sraw_voc;
        nox_baseline_ = 0.998f * nox_baseline_ + 0.002f * (float)sraw_nox;
    }

    // 1. VOC Calibration Curve:
    // Sensirion logarithmic transfer function relative to baseline:
    // Clean air: VOC Index = 100.
    float r_voc = (voc_baseline_ > 100.0f) ? ((float)sraw_voc / voc_baseline_) : 1.0f;
    float voc_idx = 100.0f * std::pow(r_voc, -2.5f);
    if (voc_idx < 1.0f)   voc_idx = 1.0f;
    if (voc_idx > 500.0f) voc_idx = 500.0f;
    d.voc_index = (uint16_t)std::round(voc_idx);

    // 2. NOx Calibration Curve:
    // Clean air: NOx Index = 1.
    float r_nox = (nox_baseline_ > 100.0f) ? ((float)sraw_nox / nox_baseline_) : 1.0f;
    float nox_idx = 1.0f;
    if (r_nox > 1.0f) {
        nox_idx = 1.0f + 499.0f * (1.0f - std::exp(-5.0f * (r_nox - 1.0f)));
    }
    if (nox_idx < 1.0f)   nox_idx = 1.0f;
    if (nox_idx > 500.0f) nox_idx = 500.0f;
    d.nox_index = (uint16_t)std::round(nox_idx);

    d.valid = true;
    return d;
}
} // namespace drivers
