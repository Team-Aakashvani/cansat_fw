/**
 * @file ina260.hpp
 * @brief INA260 precision current/voltage/power monitor (I2C).
 *
 * I2C address: 0x40 (A0=GND, A1=GND)
 * Measures: Bus voltage (V), Current (A), Power (W)
 *
 * Config: 1024× averaging, 1.1ms conversion → ~10Hz effective rate.
 *
 * @compliance CAN-7USAT §5.3 VOLTAGE telemetry field.
 */
#pragma once

#include "hal/i2c_bus.hpp"
#include <cstdint>

namespace drivers {

struct PowerData {
    double voltage_v;
    double current_a;
    double power_w;
    bool   valid;
    bool   overcurrent;  ///< > POWER_CFG.current_limit_a
};

class INA260 {
public:
    static constexpr uint8_t I2C_ADDR    = 0x40;
    static constexpr uint16_t MFID       = 0x5449;  ///< Texas Instruments
    static constexpr uint16_t DEVICE_ID  = 0x2270;

    esp_err_t init(hal::I2CBus& bus, uint8_t addr = I2C_ADDR) noexcept;
    PowerData read() noexcept;
    bool is_ready() const noexcept { return ready_; }

    static constexpr uint8_t REG_CONFIG   = 0x00;
    static constexpr uint8_t REG_CURRENT  = 0x01;
    static constexpr uint8_t REG_VOLTAGE  = 0x02;
    static constexpr uint8_t REG_POWER    = 0x03;
    static constexpr uint8_t REG_MASK_EN  = 0x06;
    static constexpr uint8_t REG_ALERT    = 0x07;
    static constexpr uint8_t REG_MFID     = 0xFE;
    static constexpr uint8_t REG_DEV_ID   = 0xFF;

private:
    hal::I2CBus* bus_  = nullptr;
    uint8_t      addr_ = I2C_ADDR;
    bool         ready_= false;

    int16_t read_reg16(uint8_t reg) noexcept;
};

} // namespace drivers
