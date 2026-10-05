/**
 * @file sgp41.hpp
 * @brief Sensirion SGP41 VOC/NOx index sensor (I2C).
 * I2C address 0x59. Measures VOC and NOx index (1–500 range).
 * Used for air quality monitoring (competition optional/science payload).
 */
#pragma once
#include "app_hal/i2c_bus.hpp"
#include <cstdint>
namespace drivers {
struct AirQualityData {
    uint16_t voc_index;   ///< 1–500 (100 = typical clean air)
    uint16_t nox_index;   ///< 1–500
    uint16_t sraw_voc;    ///< Raw ADC tick count
    uint16_t sraw_nox;    ///< Raw ADC tick count
    bool     valid;
};
class SGP41 {
public:
    static constexpr uint8_t I2C_ADDR = 0x59;
    esp_err_t init(hal::I2CBus& bus) noexcept;
    AirQualityData read(double temp_c = 25.0, double rh_pct = 50.0) noexcept;
    bool is_ready() const noexcept { return ready_; }
private:
    hal::I2CBus* bus_  = nullptr;
    bool         ready_= false;
    float        voc_baseline_ = 0.0f;
    float        nox_baseline_ = 0.0f;
    bool         baseline_init_ = false;
};
} // namespace drivers
