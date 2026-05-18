#include "drivers/bmp585.hpp"
#include "nav/frames.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

static const char* TAG = "BMP585";

namespace drivers {

esp_err_t BMP585::init(hal::I2CBus& bus, uint8_t addr, double ground_alt_m) noexcept {
    bus_          = &bus;
    addr_         = addr;
    ground_alt_m_ = ground_alt_m;

    // Verify chip ID
    uint8_t id = 0;
    esp_err_t ret = bus_->read_byte(addr_, REG_CHIP_ID, id);
    if (ret != ESP_OK || id != CHIP_ID) {
        ESP_LOGE(TAG, "BMP585 not found (id=0x%02X, expected 0x%02X)", id, CHIP_ID);
        return (ret == ESP_OK) ? ESP_ERR_NOT_FOUND : ret;
    }

    // Soft reset
    bus_->write_byte(addr_, 0x7E, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(10));

    // IIR filter coefficient 3
    ret = bus_->write_byte(addr_, REG_IIR_CFG, 0x03);
    if (ret != ESP_OK) return ret;

    // OSR: pressure ×8, temperature ×1
    ret = bus_->write_byte(addr_, REG_OSR, 0x03);   // osr_p=011 (×8), osr_t=00 (×1)
    if (ret != ESP_OK) return ret;

    // ODR = 50Hz (code 0x0B for BMP585 → ~50.05Hz)
    ret = bus_->write_byte(addr_, REG_ODR, 0x0B);
    if (ret != ESP_OK) return ret;

    // Normal mode, both sensors enabled
    ret = bus_->write_byte(addr_, REG_PWR_CTRL, 0x33);  // press_en=1, temp_en=1, mode=11 (normal)
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(5));
    ready_ = true;
    ESP_LOGI(TAG, "BMP585 ready (addr=0x%02X)", addr_);
    return ESP_OK;
}

BaroData BMP585::read() noexcept {
    BaroData data{};
    data.valid = false;
    if (!ready_ || !bus_) return data;

    // Check data-ready
    uint8_t status = 0;
    if (bus_->read_byte(addr_, REG_STATUS, status) != ESP_OK) return data;
    if (!(status & 0x20)) return data;  // Bit 5 = drdy

    // Read pressure (3 bytes big-endian) and temperature (3 bytes)
    uint8_t raw[6];
    if (bus_->read_reg(addr_, REG_PRESS_MSB, raw, 6) != ESP_OK) return data;

    data.pressure_pa    = raw_to_pressure_pa(raw);
    data.temperature_c  = raw_to_temperature_c(raw + 3);
    data.altitude_agl_m = nav::isa_pressure_to_altitude(data.pressure_pa) - ground_alt_m_;
    data.valid          = true;
    return data;
}

// BMP585 pressure raw → Pa: 20-bit signed, LSB = 1/64 Pa
double BMP585::raw_to_pressure_pa(const uint8_t raw[3]) noexcept {
    int32_t p_raw = ((int32_t)raw[0] << 16) | ((int32_t)raw[1] << 8) | raw[2];
    // Sign-extend 24-bit to 32-bit (but pressure is 20-bit from MSB)
    if (p_raw & 0x800000) p_raw |= 0xFF000000;
    return (double)p_raw / 64.0;
}

// BMP585 temperature raw → °C: 20-bit signed, LSB = 1/65536 °C
double BMP585::raw_to_temperature_c(const uint8_t raw[3]) noexcept {
    int32_t t_raw = ((int32_t)raw[0] << 16) | ((int32_t)raw[1] << 8) | raw[2];
    if (t_raw & 0x800000) t_raw |= 0xFF000000;
    return (double)t_raw / 65536.0;
}

} // namespace drivers
