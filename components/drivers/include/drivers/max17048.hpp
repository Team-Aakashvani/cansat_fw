/**
 * @file max17048.hpp
 * @brief MAX17048 LiPo fuel gauge (I2C).
 * I2C address: 0x36
 * Provides SOC (%), voltage (V), and charge rate (%/hr).
 */
#pragma once
#include "hal/i2c_bus.hpp"
#include <cstdint>

namespace drivers {

struct FuelData {
    double soc_pct;       ///< State of charge (%)
    double voltage_v;     ///< Cell voltage (V)
    double crate_pct_hr;  ///< Charge/discharge rate (%/hr)
    bool   valid;
};

class MAX17048 {
public:
    static constexpr uint8_t I2C_ADDR = 0x36;
    esp_err_t init(hal::I2CBus& bus) noexcept;
    FuelData  read() noexcept;
    bool is_ready() const noexcept { return ready_; }

    static constexpr uint8_t REG_VCELL  = 0x02;
    static constexpr uint8_t REG_SOC    = 0x04;
    static constexpr uint8_t REG_CRATE  = 0x16;
    static constexpr uint8_t REG_ID     = 0x18;
    static constexpr uint8_t REG_VRESET = 0x19;

private:
    hal::I2CBus* bus_  = nullptr;
    bool         ready_= false;
};

} // namespace drivers
