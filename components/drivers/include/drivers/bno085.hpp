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

#include "app_hal/i2c_bus.hpp"
#include "nav/config.hpp"
#include "drivers/imu_attitude.hpp"
#include <cstdint>

namespace drivers {

struct IMUData {
    double acc_x, acc_y, acc_z;   ///< m/s² (specific force, body frame)
    double gyr_x, gyr_y, gyr_z;   ///< rad/s
    double mag_x, mag_y, mag_z;   ///< µT
    double euler_pitch_deg = 0.0; ///< Fused orientation Pitch (deg)
    double euler_roll_deg  = 0.0; ///< Fused orientation Roll (deg)
    double euler_yaw_deg   = 0.0; ///< Fused orientation Yaw/Heading (deg)
    double quat_w = 1.0;          ///< Relative vehicle quaternion
    double quat_x = 0.0;
    double quat_y = 0.0;
    double quat_z = 0.0;
    double qw_world = 1.0;        ///< Vehicle -> fusion-world quaternion (magnetic heading kept)
    double qx_world = 0.0;
    double qy_world = 0.0;
    double qz_world = 0.0;
    MountClass mount = MountClass::UNKNOWN; ///< Auto-identified mount orientation
    double timestamp_s;
    bool   valid;
    bool   mag_valid;
    bool   euler_valid;
    bool   quat_valid;
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

    void reset() noexcept;
    void calibrate_gyro_bias(int samples = 80) noexcept;

    bool is_ready() const noexcept { return ready_; }
    void request_attitude_tare() noexcept { attitude_ref_.request_tare(); }
    MountClass current_mount() const noexcept { return attitude_ref_.mount(); }
    AttitudeReference& attitude_ref() noexcept { return attitude_ref_; }

    enum class IMUType : uint8_t { UNKNOWN, BNO085, BNO055, MPU6050 };

private:
    IMUData read_mpu6050() noexcept;
    IMUData read_bno055() noexcept;
    hal::I2CBus* bus_       = nullptr;
    uint8_t      addr_      = I2C_ADDR_LOW;
    bool         ready_     = false;
    IMUType      imu_type_  = IMUType::UNKNOWN;
    double       gyro_bias_x_ = 0.0;
    double       gyro_bias_y_ = 0.0;
    double       gyro_bias_z_ = 0.0;
    uint64_t     last_read_us_ = 0;
    AttitudeReference attitude_ref_{};

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
