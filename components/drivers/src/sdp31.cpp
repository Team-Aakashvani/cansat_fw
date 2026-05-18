#include "drivers/sdp31.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>
static const char* TAG = "SDP31";
namespace drivers {
esp_err_t SDP31::init(hal::I2CBus& bus) noexcept {
    bus_ = &bus;
    // Start continuous measurement, mass flow compensation
    const uint8_t cmd[2] = { 0x36, 0x15 };
    esp_err_t ret = bus_->write_reg(I2C_ADDR, cmd[0], cmd + 1, 1);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "SDP31 start failed"); return ret; }
    vTaskDelay(pdMS_TO_TICKS(20));
    // Read product number to get scale factor
    const uint8_t req[2] = { 0x36, 0x7C };
    bus_->write_reg(I2C_ADDR, req[0], req + 1, 1);
    uint8_t id_buf[18] = {};
    bus_->read_reg(I2C_ADDR, 0x00, id_buf, 18);
    // Scale from product ID word (bytes 12-13)
    uint16_t scale_raw = ((uint16_t)id_buf[12] << 8) | id_buf[13];
    if (scale_raw > 0) scale_pa_lsb_ = (double)scale_raw;
    ready_ = true;
    ESP_LOGI(TAG, "SDP31 ready (scale=%.0f Pa/LSB)", scale_pa_lsb_);
    return ESP_OK;
}
DiffPressData SDP31::read(double rho) noexcept {
    DiffPressData d{}; if (!ready_) return d;
    uint8_t buf[9] = {};
    if (bus_->read_reg(I2C_ADDR, 0x00, buf, 9) != ESP_OK) return d;
    int16_t p_raw = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t t_raw = (int16_t)((buf[3] << 8) | buf[4]);
    d.diff_pressure_pa = (double)p_raw / scale_pa_lsb_;
    d.temperature_c    = (double)t_raw / scale_t_lsb_;
    // Bernoulli: v = sqrt(2*|ΔP|/ρ) with sign from pressure
    const double sign = (d.diff_pressure_pa >= 0) ? 1.0 : -1.0;
    d.airspeed_mps     = sign * std::sqrt(2.0 * std::abs(d.diff_pressure_pa) / rho);
    d.valid = true;
    return d;
}
} // namespace drivers
