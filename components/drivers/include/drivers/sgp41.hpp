/**
 * @file sgp41.hpp
 * @brief Sensirion SGP41 VOC/NOx index sensor (I2C).
 * I2C address 0x59. Measures VOC and NOx index (1–500 range).
 * Used for air quality monitoring (competition optional/science payload).
 */
#pragma once
#include "hal/i2c_bus.hpp"
#include <cstdint>
namespace drivers {
struct AirQualityData {
    uint16_t voc_index;   ///< 1–500 (100 = typical clean air)
    uint16_t nox_index;   ///< 1–500
    bool     valid;
};
class SGP41 {
public:
    static constexpr uint8_t I2C_ADDR = 0x59;
    esp_err_t init(hal::I2CBus& bus) noexcept;
    AirQualityData read() noexcept;
    bool is_ready() const noexcept { return ready_; }
private:
    hal::I2CBus* bus_  = nullptr;
    bool         ready_= false;
    uint16_t     voc_idx_ = 0;
    uint16_t     nox_idx_ = 0;
    // Simplified VocalGasIndex algorithm state (Sensirion open-source)
    int32_t voc_state_  = 0;
    int32_t nox_state_  = 0;
};
} // namespace drivers
