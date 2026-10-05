#include "drivers/bmp585.hpp"
#include "nav/frames.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>
#include <cmath>

static const char* TAG = "Baro";

namespace drivers {

esp_err_t BMP585::init(hal::I2CBus& bus, uint8_t addr, double ground_alt_m) noexcept {
    bus_          = &bus;
    ground_alt_m_ = ground_alt_m;

    // 1. Try BMP585 / BMP581 at given address or standard 0x46 / 0x47
    uint8_t probe_addrs[] = {addr, 0x46, 0x47, 0x76, 0x77};
    for (uint8_t a : probe_addrs) {
        if (a == 0) continue;
        uint8_t id = 0;
        if (bus_->read_byte(a, REG_CHIP_ID, id) == ESP_OK && (id == 0x50 || id == 0x51)) {
            addr_ = a;
            chip_type_ = ChipType::BMP585;

            // Step 1: Soft reset
            bus_->write_byte(addr_, REG_CMD, 0xB6);
            vTaskDelay(pdMS_TO_TICKS(15));

            // Step 2: Set ODR_CONFIG (0x37): deep_dis=1 (bit 7), ODR=50Hz (0x0F<<2), pwr_mode=NORMAL (0x01)
            // 0x80 | (0x0F << 2) | 0x01 = 0xBD
            bus_->write_byte(addr_, REG_ODR_CONFIG, 0xBD);
            vTaskDelay(pdMS_TO_TICKS(5));

            // Step 3: Set OSR_CONFIG (0x36): press_en=1 (bit 6 = 0x40), osr_p=8x (3<<3 = 0x18), osr_t=2x (0x01)
            // 0x40 | 0x18 | 0x01 = 0x59
            bus_->write_byte(addr_, REG_OSR_CONFIG, 0x59);
            vTaskDelay(pdMS_TO_TICKS(5));

            // Step 4: Configure DSP IIR filter for smooth, low-noise barometric pressure
            bus_->write_byte(addr_, REG_DSP_CONFIG, 0x03);
            bus_->write_byte(addr_, REG_DSP_IIR, 0x1B);  // coefficient 3 for T and P
            vTaskDelay(pdMS_TO_TICKS(10));

            ready_ = true;
            ESP_LOGI(TAG, "BMP585 detected (ID=0x%02X) and initialised at 0x%02X (50Hz ODR, 8x OSR, IIR=3)", id, addr_);
            return ESP_OK;
        }

        // 2. Try BMP280 / BME280 at 0xD0 register
        if (bus_->read_byte(a, 0xD0, id) == ESP_OK) {
            if (id == 0x58 || id == 0x60 || id == 0x56 || id == 0x57) {
                addr_ = a;
                chip_type_ = ChipType::BMP280;
                
                // Read calibration table (24 bytes from 0x88)
                uint8_t cal[24];
                if (bus_->read_reg(addr_, 0x88, cal, 24) == ESP_OK) {
                    bmp280_cal_.dig_T1 = (uint16_t)(cal[0] | (cal[1] << 8));
                    bmp280_cal_.dig_T2 = (int16_t)(cal[2] | (cal[3] << 8));
                    bmp280_cal_.dig_T3 = (int16_t)(cal[4] | (cal[5] << 8));
                    bmp280_cal_.dig_P1 = (uint16_t)(cal[6] | (cal[7] << 8));
                    bmp280_cal_.dig_P2 = (int16_t)(cal[8] | (cal[9] << 8));
                    bmp280_cal_.dig_P3 = (int16_t)(cal[10] | (cal[11] << 8));
                    bmp280_cal_.dig_P4 = (int16_t)(cal[12] | (cal[13] << 8));
                    bmp280_cal_.dig_P5 = (int16_t)(cal[14] | (cal[15] << 8));
                    bmp280_cal_.dig_P6 = (int16_t)(cal[16] | (cal[17] << 8));
                    bmp280_cal_.dig_P7 = (int16_t)(cal[18] | (cal[19] << 8));
                    bmp280_cal_.dig_P8 = (int16_t)(cal[20] | (cal[21] << 8));
                    bmp280_cal_.dig_P9 = (int16_t)(cal[22] | (cal[23] << 8));
                }

                // Configure BMP280: normal mode, osrs_t=2, osrs_p=5
                bus_->write_byte(addr_, 0xF4, 0x57);  // ctrl_meas
                bus_->write_byte(addr_, 0xF5, 0x10);  // config: t_sb=0.5ms, filter=16
                vTaskDelay(pdMS_TO_TICKS(10));
                ready_ = true;
                ESP_LOGI(TAG, "BMP280 detected and initialised at 0x%02X (ID=0x%02X)", addr_, id);
                return ESP_OK;
            }
        }
    }

    ESP_LOGE(TAG, "No barometer found (tried BMP585 0x46/0x47, BMP280 0x76/0x77)");
    return ESP_ERR_NOT_FOUND;
}

BaroData BMP585::read_bmp280() noexcept {
    BaroData data{};
    data.valid = false;
    uint8_t raw[6];
    if (bus_->read_reg(addr_, 0xF7, raw, 6) != ESP_OK) return data;

    int32_t adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
    int32_t adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);

    if (adc_P == 0 || adc_T == 0) return data;

    // Compensate temperature
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)bmp280_cal_.dig_T1 << 1))) * ((int32_t)bmp280_cal_.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)bmp280_cal_.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp280_cal_.dig_T1))) >> 12) * ((int32_t)bmp280_cal_.dig_T3)) >> 14;
    bmp280_cal_.t_fine = var1 + var2;
    data.temperature_c = (double)((bmp280_cal_.t_fine * 5 + 128) >> 8) / 100.0;

    // Compensate pressure
    int64_t p_var1 = ((int64_t)bmp280_cal_.t_fine) - 128000;
    int64_t p_var2 = p_var1 * p_var1 * (int64_t)bmp280_cal_.dig_P6;
    p_var2 = p_var2 + ((p_var1 * (int64_t)bmp280_cal_.dig_P5) << 17);
    p_var2 = p_var2 + (((int64_t)bmp280_cal_.dig_P4) << 35);
    p_var1 = ((p_var1 * p_var1 * (int64_t)bmp280_cal_.dig_P3) >> 8) + ((p_var1 * (int64_t)bmp280_cal_.dig_P2) << 12);
    p_var1 = (((((int64_t)1) << 47) + p_var1)) * ((int64_t)bmp280_cal_.dig_P1) >> 33;

    if (p_var1 == 0) return data;

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - p_var2) * 3125) / p_var1;
    p_var1 = (((int64_t)bmp280_cal_.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    p_var2 = (((int64_t)bmp280_cal_.dig_P8) * p) >> 19;
    p = ((p + p_var1 + p_var2) >> 8) + (((int64_t)bmp280_cal_.dig_P7) << 4);
    data.pressure_pa = (double)p / 256.0;

    // Hypsometric formula using calibrated sea-level reference pressure (QNH)
    double p0 = (sea_level_p0_pa_ > 80000.0 && sea_level_p0_pa_ < 120000.0) ? sea_level_p0_pa_ : 101325.0;
    double alt_msl = 44330.0 * (1.0 - std::pow(data.pressure_pa / p0, 0.190294957));
    data.altitude_agl_m = alt_msl - ground_alt_m_;
    data.valid = (data.pressure_pa > 30000.0 && data.pressure_pa < 120000.0);
    return data;
}

void BMP585::auto_calibrate_baseline(double known_elevation_m, int samples) noexcept {
    if (samples <= 0) samples = 20;
    double sum_p = 0.0;
    int valid_cnt = 0;

    for (int i = 0; i < samples; ++i) {
        BaroData d = read();
        if (d.valid) {
            sum_p += d.pressure_pa;
            valid_cnt++;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    if (valid_cnt > 0) {
        double avg_p = sum_p / valid_cnt;
        if (known_elevation_m > -500.0 && known_elevation_m < 9000.0) {
            // Calculate sea-level pressure (QNH) from known elevation
            sea_level_p0_pa_ = avg_p / std::pow(1.0 - (known_elevation_m / 44330.0), 5.255);
            ground_alt_m_ = 0.0; // AGL is 0 on ground
            ESP_LOGI(TAG, "Baro Auto-Calibrated: Local P=%.1f Pa, Known Alt=%.1fm -> QNH=%.1f Pa",
                     avg_p, known_elevation_m, sea_level_p0_pa_);
        } else {
            // Standard tare to zero AGL
            ground_alt_m_ = 44330.0 * (1.0 - std::pow(avg_p / sea_level_p0_pa_, 0.190294957));
            ESP_LOGI(TAG, "Baro Tare Zeroed: Base Alt=%.1fm, Local P=%.1f Pa", ground_alt_m_, avg_p);
        }
    }
}

BaroData BMP585::read() noexcept {
    if (sim_enabled_) {
        BaroData sim_data{};
        sim_data.pressure_pa = sim_pressure_pa_;
        sim_data.temperature_c = 25.0;
        // Same calibrated reference as live data, so simulated altitudes are AGL and the
        // mission logic sees exactly what it would see in flight
        const double p0 = (sea_level_p0_pa_ > 30000.0) ? sea_level_p0_pa_ : 101325.0;
        double alt_agl = 44330.0 * (1.0 - std::pow(sim_data.pressure_pa / p0, 0.190294957)) - ground_alt_m_;
        sim_data.altitude_agl_m = alt_agl;
        sim_data.valid = true;
        return sim_data;
    }

    if (!ready_ || !bus_) return BaroData{};
    if (chip_type_ == ChipType::BMP280) {
        return read_bmp280();
    }

    BaroData data{};
    data.valid = false;

    // Read 6 bytes: 3 bytes pressure (0x1D..0x1F), 3 bytes temperature (0x20..0x22)
    uint8_t raw[6];
    if (bus_->read_reg(addr_, 0x1D, raw, 6) != ESP_OK) return data;

    // BMP585: 0x1D..0x1F = TEMP_DATA (XLSB..MSB), 0x20..0x22 = PRESS_DATA (XLSB..MSB)
    int32_t t_raw  = ((int32_t)raw[2] << 16) | ((int32_t)raw[1] << 8) | raw[0];
    if (t_raw & 0x800000) t_raw |= 0xFF000000;
    uint32_t p_raw = ((uint32_t)raw[5] << 16) | ((uint32_t)raw[4] << 8) | raw[3];

    data.temperature_c = (double)t_raw / 65536.0;
    data.pressure_pa   = (double)p_raw / 64.0;

    if (data.pressure_pa < 30000.0 || data.pressure_pa > 125000.0) return data;

    double p0 = (sea_level_p0_pa_ > 80000.0 && sea_level_p0_pa_ < 120000.0) ? sea_level_p0_pa_ : 101325.0;
    double alt_msl = 44330.0 * (1.0 - std::pow(data.pressure_pa / p0, 0.190294957));
    data.altitude_agl_m = alt_msl - ground_alt_m_;
    data.valid          = true;
    return data;
}

// BMP585 pressure raw → Pa: 24-bit, LSB = 1/64 Pa
double BMP585::raw_to_pressure_pa(const uint8_t raw[3]) noexcept {
    uint32_t p_raw = ((uint32_t)raw[2] << 16) | ((uint32_t)raw[1] << 8) | raw[0];
    return (double)p_raw / 64.0;
}

// BMP585 temperature raw → °C: 24-bit signed, LSB = 1/65536 °C
double BMP585::raw_to_temperature_c(const uint8_t raw[3]) noexcept {
    int32_t t_raw = ((int32_t)raw[2] << 16) | ((int32_t)raw[1] << 8) | raw[0];
    if (t_raw & 0x800000) t_raw |= 0xFF000000;
    return (double)t_raw / 65536.0;
}

} // namespace drivers
