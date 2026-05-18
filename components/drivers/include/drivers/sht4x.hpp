/**
 * @file sht4x.hpp
 * @brief Sensirion SHT40/SHT41 humidity/temperature sensor (I2C).
 * I2C address 0x44. 1Hz measurement rate, high-repeatability mode.
 * Provides: temperature (°C), relative humidity (%).
 * Used for cabin environment monitoring (competition optional field).
 */
#pragma once
#include "hal/i2c_bus.hpp"
#include <cstdint>
namespace drivers {
struct HumidData {
    double temperature_c;
    double humidity_pct;
    bool   valid;
};
class SHT4x {
public:
    static constexpr uint8_t I2C_ADDR = 0x44;
    esp_err_t init(hal::I2CBus& bus) noexcept;
    HumidData read() noexcept;
    bool is_ready() const noexcept { return ready_; }
private:
    hal::I2CBus* bus_  = nullptr;
    bool         ready_= false;
};
} // namespace drivers
