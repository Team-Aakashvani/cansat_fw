/**
 * @file bno085.hpp
 * @brief BNO085 9-DOF IMU driver over I2C (ESP-IDF).
 *
 * The BNO085 is used in NDOF mode (accelerometer + gyroscope ARVR-stabilised).
 * We disable the on-board sensor fusion and read RAW accelerometer + gyroscope
 * for the flight computer's own ES-EKF — using the BNO085's superior
 * calibrated outputs rather than its internal Euler angles.
 *
 * I2C address: 0x4A (ADDR pin to GND) or 0x4B (ADDR to VDD).
 *
 * Output rate: 100Hz (max 400Hz; 100Hz sufficient per rules, saves power).
 *
 * Data available: BNO085 signals via INT pin (gpio pin configurable).
 *
 * @compliance BNO085 datasheet Rev1.2, SHTP protocol
 */
#pragma once

#include "hal/i2c_bus.hpp"
#include "nav/config.hpp"
#include <cstdint>

namespace drivers {

struct IMUData {
    double acc_x, acc_y, acc_z;   ///< m/s² (specific force, body frame)
    double gyr_x, gyr_y, gyr_z;   ///< rad/s
    double mag_x, mag_y, mag_z;   ///< µT
    double timestamp_s;
    bool   valid;
    bool   mag_valid;
    bool   saturated;
};

class BNO085 {
public:
    static constexpr uint8_t I2C_ADDR_LOW  = 0x4A;
    static constexpr uint8_t I2C_ADDR_HIGH = 0x4B;
    static constexpr uint8_t CHIP_ID       = 0xF8;  ///< Expected product ID byte

    BNO085() noexcept = default;

    /// Initialise: reset device, configure IMU reports at rate_hz.
    esp_err_t init(hal::I2CBus& bus, uint8_t addr = I2C_ADDR_LOW,
                   double rate_hz = 100.0) noexcept;

    /// Poll for new data (non-blocking). Returns valid IMUData if new sample ready.
    IMUData read() noexcept;

    /// Trigger soft reset.
    void reset() noexcept;

    bool is_ready() const noexcept { return ready_; }

private:
    hal::I2CBus* bus_   = nullptr;
    uint8_t      addr_  = I2C_ADDR_LOW;
    bool         ready_ = false;

    // SHTP/SH2 packet handling
    esp_err_t shtp_write(uint8_t channel, const uint8_t* payload, size_t len) noexcept;
    int        shtp_read(uint8_t* buf, size_t max_len) noexcept;
    esp_err_t enable_report(uint8_t report_id, uint32_t interval_us) noexcept;
    esp_err_t parse_input_report(const uint8_t* buf, size_t len, IMUData& out) noexcept;

    // Scale factors from BNO085 datasheet
    static constexpr double Q4_SCALE  = 1.0 / (1 << 4);   // Q-point 4 → float
    static constexpr double Q8_SCALE  = 1.0 / (1 << 8);   // Q-point 8 → float
    static constexpr double Q9_SCALE  = 1.0 / (1 << 9);
    static constexpr double Q14_SCALE = 1.0 / (1 << 14);

    // BNO085 SHTP channels
    static constexpr uint8_t CHANNEL_SHTP_CMD  = 0;
    static constexpr uint8_t CHANNEL_EXE       = 1;
    static constexpr uint8_t CHANNEL_CONTROL   = 2;
    static constexpr uint8_t CHANNEL_INPUT      = 3;

    // Report IDs
    static constexpr uint8_t REPORT_ACCELEROMETER   = 0x01;
    static constexpr uint8_t REPORT_GYROSCOPE_CALIB = 0x02;
    static constexpr uint8_t REPORT_MAGNETOMETER    = 0x03;
    static constexpr uint8_t REPORT_RAW_ACCEL       = 0x14;
    static constexpr uint8_t REPORT_RAW_GYRO        = 0x15;

    uint8_t seq_[8] = {};  // SHTP sequence numbers per channel
};

} // namespace drivers
