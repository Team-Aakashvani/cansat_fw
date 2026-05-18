/**
 * @file sdp31.hpp
 * @brief Sensirion SDP31 differential pressure sensor (I2C).
 * Used as pitot-style airspeed sensor. I2C address 0x21 (ADDR pin).
 * Output: differential pressure (Pa) + temperature (°C) at 200Hz max.
 * Configuration: Continuous measurement, mass flow, averaging off.
 */
#pragma once
#include "hal/i2c_bus.hpp"
#include <cstdint>
namespace drivers {
struct DiffPressData {
    double diff_pressure_pa;
    double temperature_c;
    double airspeed_mps;  ///< Computed from Bernoulli (ρ known from baro)
    bool   valid;
};
class SDP31 {
public:
    static constexpr uint8_t I2C_ADDR = 0x21;
    esp_err_t init(hal::I2CBus& bus) noexcept;
    DiffPressData read(double air_density_kgm3 = 1.225) noexcept;
    bool is_ready() const noexcept { return ready_; }
private:
    hal::I2CBus* bus_  = nullptr;
    bool         ready_= false;
    // Scale factors read from sensor (product ID dependent)
    double scale_pa_lsb_ = 60.0;  // Default SDP31 scale 60 Pa/LSB (from datasheet)
    double scale_t_lsb_  = 200.0; // 200 LSB/°C
};
} // namespace drivers
