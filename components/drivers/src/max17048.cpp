#include "drivers/max17048.hpp"
#include "esp_log.h"
static const char* TAG = "MAX17048";
namespace drivers {
esp_err_t MAX17048::init(hal::I2CBus& bus) noexcept {
    bus_ = &bus;
    if (!bus_->probe(I2C_ADDR)) { ESP_LOGE(TAG, "Not found"); return ESP_ERR_NOT_FOUND; }
    ready_ = true;
    ESP_LOGI(TAG, "MAX17048 ready");
    return ESP_OK;
}
FuelData MAX17048::read() noexcept {
    FuelData d{}; if (!ready_) return d;
    uint8_t buf[2] = {};
    // VCELL: 78.125µV/LSB (12-bit, big-endian MSB first)
    bus_->read_reg(I2C_ADDR, REG_VCELL, buf, 2);
    uint16_t raw_v = ((uint16_t)buf[0] << 4) | (buf[1] >> 4);
    d.voltage_v = raw_v * 78.125e-6;
    // SOC: high byte = integer %, low byte = 1/256 %
    bus_->read_reg(I2C_ADDR, REG_SOC, buf, 2);
    d.soc_pct = buf[0] + buf[1] / 256.0;
    // CRATE: 0.208%/hr per LSB
    bus_->read_reg(I2C_ADDR, REG_CRATE, buf, 2);
    d.crate_pct_hr = (int16_t)((buf[0]<<8)|buf[1]) * 0.208;
    d.valid = true;
    return d;
}
} // namespace drivers
