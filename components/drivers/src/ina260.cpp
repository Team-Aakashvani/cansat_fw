#include "drivers/ina260.hpp"
#include "nav/config.hpp"
#include "esp_log.h"
#include <cmath>

static const char* TAG = "INA260";

namespace drivers {

int16_t INA260::read_reg16(uint8_t reg) noexcept {
    uint8_t buf[2] = {};
    bus_->read_reg(addr_, reg, buf, 2);
    return (int16_t)((buf[0] << 8) | buf[1]);
}

esp_err_t INA260::init(hal::I2CBus& bus, uint8_t addr) noexcept {
    bus_ = &bus; addr_ = addr;
    if (!bus_->probe(addr_)) {
        ESP_LOGE(TAG, "INA260 not found at 0x%02X", addr_);
        return ESP_ERR_NOT_FOUND;
    }
    // Config: 1024 avg, 1.1ms bus+shunt, continuous mode
    // CONFIG = 0x6327: AVG=1024, VBUSCT=1.1ms, ISHCT=1.1ms, mode=111
    const uint8_t cfg[2] = { 0x63, 0x27 };
    esp_err_t ret = bus_->write_reg(addr_, REG_CONFIG, cfg, 2);
    if (ret != ESP_OK) return ret;
    ready_ = true;
    ESP_LOGI(TAG, "INA260 ready");
    return ESP_OK;
}

PowerData INA260::read() noexcept {
    PowerData d{};
    if (!ready_) return d;
    // Current: 1.25mA/LSB
    d.current_a  = read_reg16(REG_CURRENT) * 1.25e-3;
    // Voltage: 1.25mV/LSB
    d.voltage_v  = (int16_t)read_reg16(REG_VOLTAGE) * 1.25e-3;
    // Power: 10mW/LSB
    d.power_w    = (int16_t)read_reg16(REG_POWER) * 10.0e-3;
    d.overcurrent = std::abs(d.current_a) > nav::POWER_CFG.current_limit_a;
    d.valid = true;
    return d;
}

} // namespace drivers
