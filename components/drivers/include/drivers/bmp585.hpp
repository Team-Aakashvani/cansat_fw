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

#include "hal/i2c_bus.hpp"
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

    bool is_ready() const noexcept { return ready_; }

    // BMP585 register map
    static constexpr uint8_t REG_CHIP_ID   = 0x01;
    static constexpr uint8_t REG_STATUS    = 0x27;
    static constexpr uint8_t REG_PRESS_MSB = 0x20;   // 3-byte big-endian
    static constexpr uint8_t REG_TEMP_MSB  = 0x23;   // 3-byte big-endian
    static constexpr uint8_t REG_PWR_CTRL  = 0x36;
    static constexpr uint8_t REG_OSR       = 0x37;
    static constexpr uint8_t REG_ODR       = 0x38;
    static constexpr uint8_t REG_IIR_CFG   = 0x31;

private:
    hal::I2CBus* bus_          = nullptr;
    uint8_t      addr_         = I2C_ADDR_SDO_GND;
    double       ground_alt_m_ = 0.0;
    bool         ready_        = false;

    // Pressure and temperature conversion helpers
    static double raw_to_pressure_pa(const uint8_t raw[3]) noexcept;
    static double raw_to_temperature_c(const uint8_t raw[3]) noexcept;
};

} // namespace drivers
