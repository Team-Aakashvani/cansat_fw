/**
 * @file bmp585.hpp
 * @brief BMP585 barometric pressure/temperature sensor driver (I2C).
 *
 * Provides calibrated pressure (Pa) and temperature (°C) at up to 200Hz ODR.
 * Converts pressure to altitude AGL using the ISA troposphere model.
 *
 * I2C address: 0x46 (SDO=GND) or 0x47 (SDO=VDD).
 *
 * Configuration:
 *   - OSR_P: 8× oversampling (best noise vs. rate for 50Hz)
 *   - OSR_T: 1× oversampling
 *   - IIR filter: coefficient 3
 *   - ODR: 50Hz (20ms period)
 *   - Power mode: Normal (continuous)
 *
 * @compliance BMP585 datasheet v1.4
 */
#pragma once

#include "app_hal/i2c_bus.hpp"
#include <cstdint>

namespace drivers {

struct BaroData {
    double pressure_pa;    ///< Calibrated pressure (Pa)
    double temperature_c;  ///< Temperature (°C)
    double altitude_agl_m; ///< ISA-derived altitude AGL (m)
    double timestamp_s;
    bool   valid;
};

class BMP585 {
public:
    static constexpr uint8_t I2C_ADDR_SDO_GND = 0x46;
    static constexpr uint8_t I2C_ADDR_SDO_VDD = 0x47;
    static constexpr uint8_t CHIP_ID           = 0x51;

    BMP585() noexcept = default;

    esp_err_t init(hal::I2CBus& bus, uint8_t addr = I2C_ADDR_SDO_GND,
                   double ground_alt_m = 0.0) noexcept;

    BaroData read() noexcept;

    /// Set pad altitude reference (call after averaging several readings at rest).
    void set_ground_altitude(double alt_m) noexcept { ground_alt_m_ = alt_m; }
    double get_ground_altitude() const noexcept { return ground_alt_m_; }

    void set_sea_level_pressure(double p0_pa) noexcept { sea_level_p0_pa_ = p0_pa; }
    double get_sea_level_pressure() const noexcept { return sea_level_p0_pa_; }

    /// Auto-calibrate baseline from live readings
    void auto_calibrate_baseline(double known_elevation_m = 0.0, int samples = 20) noexcept;

    void set_sim_mode(bool enable) noexcept { sim_enabled_ = enable; }
    bool is_sim_mode() const noexcept { return sim_enabled_; }
    void inject_sim_pressure(double pa) noexcept {
        sim_pressure_pa_ = pa;
        sim_data_ready_ = true;
    }

    bool is_ready() const noexcept { return ready_; }

    // BMP585 register map (Bosch Sensortec BMP581/BMP585)
    static constexpr uint8_t REG_CHIP_ID    = 0x01;
    static constexpr uint8_t REG_REV_ID     = 0x02;
    static constexpr uint8_t REG_TEMP_XLSB  = 0x1D;
    static constexpr uint8_t REG_PRESS_XLSB = 0x20;
    static constexpr uint8_t REG_INT_STATUS = 0x27;
    static constexpr uint8_t REG_STATUS     = 0x28;
    static constexpr uint8_t REG_DSP_CONFIG = 0x30;
    static constexpr uint8_t REG_DSP_IIR    = 0x31;
    static constexpr uint8_t REG_OSR_CONFIG = 0x36;
    static constexpr uint8_t REG_ODR_CONFIG = 0x37;
    static constexpr uint8_t REG_CMD        = 0x7E;

    enum class ChipType : uint8_t { UNKNOWN, BMP585, BMP280 };

    struct BMP280Calib {
        uint16_t dig_T1 = 0;
        int16_t  dig_T2 = 0;
        int16_t  dig_T3 = 0;
        uint16_t dig_P1 = 0;
        int16_t  dig_P2 = 0;
        int16_t  dig_P3 = 0;
        int16_t  dig_P4 = 0;
        int16_t  dig_P5 = 0;
        int16_t  dig_P6 = 0;
        int16_t  dig_P7 = 0;
        int16_t  dig_P8 = 0;
        int16_t  dig_P9 = 0;
        int32_t  t_fine  = 0;
    };

private:
    hal::I2CBus* bus_          = nullptr;
    uint8_t      addr_         = I2C_ADDR_SDO_GND;
    double       ground_alt_m_ = 0.0;
    double       sea_level_p0_pa_ = 101325.0;
    bool         ready_        = false;
    ChipType     chip_type_    = ChipType::UNKNOWN;
    BMP280Calib  bmp280_cal_{};
    bool         sim_enabled_  = false;
    double       sim_pressure_pa_ = 101325.0;
    bool         sim_data_ready_  = false;

    // Pressure and temperature conversion helpers
    static double raw_to_pressure_pa(const uint8_t raw[3]) noexcept;
    static double raw_to_temperature_c(const uint8_t raw[3]) noexcept;
    BaroData read_bmp280() noexcept;
};

} // namespace drivers
