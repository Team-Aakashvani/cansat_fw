/**
 * @file bno085.cpp
 * @brief BNO085 IMU driver — I2C + SHTP protocol implementation.
 *
 * Implements the Sensor Hub Transport Protocol (SHTP) over I2C to configure
 * the BNO085 for raw calibrated accelerometer + gyroscope output at 100Hz.
 */
#include "drivers/bno085.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <cstring>
#include <cmath>

static const char* TAG = "BNO085";

namespace drivers {

// SHTP header: 4 bytes [length_lsb, length_msb, channel, seq_num]
static constexpr size_t SHTP_HDR = 4;
static constexpr size_t SHTP_MAX = 128;

esp_err_t BNO085::init(hal::I2CBus& bus, uint8_t addr, double rate_hz) noexcept {
    bus_  = &bus;

    // 1. Try BNO055 at 0x28 / 0x29 (Bosch BNO055 Chip ID reg 0x00 == 0xA0)
    uint8_t bno055_addrs[] = {addr, 0x28, 0x29};
    for (uint8_t a : bno055_addrs) {
        if (a != 0x28 && a != 0x29) continue;
        uint8_t id = 0;
        if (bus_->read_byte(a, 0x00, id) == ESP_OK && id == 0xA0) {
            addr_ = a;
            // Normal power mode
            bus_->write_byte(addr_, 0x3E, 0x00);
            vTaskDelay(pdMS_TO_TICKS(10));
            // NDOF operating mode (9-DOF fusion)
            bus_->write_byte(addr_, 0x3D, 0x0C);
            vTaskDelay(pdMS_TO_TICKS(20));
            ready_ = true;
            imu_type_ = IMUType::BNO055;
            ESP_LOGI(TAG, "BNO055 detected at 0x%02X in 9-DOF fusion mode!", addr_);
            return ESP_OK;
        }
    }

    // 2. Try MPU6050 / MPU6500 / MPU9250 at 0x68 / 0x69
    uint8_t mpu_addrs[] = {0x68, 0x69};
    for (uint8_t a : mpu_addrs) {
        uint8_t who = 0;
        if (bus_->read_byte(a, 0x75, who) == ESP_OK && who != 0x00 && who != 0xFF) {
            addr_ = a;
            // Wake up MPU6050 (PWR_MGMT_1 = 0x00)
            bus_->write_byte(addr_, 0x6B, 0x00);
            vTaskDelay(pdMS_TO_TICKS(10));
            // Sample rate divider: 1kHz / (1 + 7) = 125Hz
            bus_->write_byte(addr_, 0x19, 0x07);
            // DLPF config: 42Hz filter
            bus_->write_byte(addr_, 0x1A, 0x03);
            // Gyro full scale: ±2000 deg/s
            bus_->write_byte(addr_, 0x1B, 0x18);
            // Accel full scale: ±8g
            bus_->write_byte(addr_, 0x1C, 0x10);
            ready_ = true;
            imu_type_ = IMUType::MPU6050;
            ESP_LOGI(TAG, "MPU6050/6500 detected at 0x%02X (WHO_AM_I=0x%02X) — Calibrating gyro bias...", addr_, who);
            calibrate_gyro_bias(60);
            ESP_LOGI(TAG, "MPU6050 initialised (bias: gx=%.4f, gy=%.4f, gz=%.4f rad/s)",
                     gyro_bias_x_, gyro_bias_y_, gyro_bias_z_);
            return ESP_OK;
        }
    }

    // 3. Try BNO085 at 0x4A / 0x4B (native Hillcrest SHTP)
    uint8_t bno085_addrs[] = {0x4A, 0x4B};
    for (uint8_t a : bno085_addrs) {
        if (bus_->probe(a)) {
            addr_ = a;
            uint8_t reset_cmd[1] = { 0x01 };
            if (shtp_write(CHANNEL_EXE, reset_cmd, 1) == ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(150));
                uint32_t interval_us = (uint32_t)(1.0e6 / rate_hz);
                enable_report(REPORT_ACCELEROMETER, interval_us);
                enable_report(REPORT_GYROSCOPE_CALIB, interval_us);
                enable_report(REPORT_MAGNETOMETER, interval_us);
                ready_ = true;
                imu_type_ = IMUType::BNO085;
                ESP_LOGI(TAG, "BNO085 detected and initialised at 0x%02X @ %.0f Hz", addr_, rate_hz);
                return ESP_OK;
            }
        }
    }

    ESP_LOGE(TAG, "No IMU found (tried BNO055 0x28/0x29, MPU6050 0x68/0x69, BNO085 0x4A/0x4B)");
    return ESP_ERR_NOT_FOUND;
}

void BNO085::calibrate_gyro_bias(int samples) noexcept {
    if (!ready_ || !bus_ || samples <= 0) return;
    double sum_gx = 0.0, sum_gy = 0.0, sum_gz = 0.0;
    int valid_cnt = 0;
    constexpr double GYRO_SCALE = (2000.0 * 3.141592653589793) / (180.0 * 32768.0);

    for (int i = 0; i < samples; ++i) {
        uint8_t buf[6];
        if (bus_->read_reg(addr_, 0x43, buf, 6) == ESP_OK) {
            int16_t raw_gx = (int16_t)((buf[0] << 8) | buf[1]);
            int16_t raw_gy = (int16_t)((buf[2] << 8) | buf[3]);
            int16_t raw_gz = (int16_t)((buf[4] << 8) | buf[5]);
            sum_gx += (double)raw_gx * GYRO_SCALE;
            sum_gy += (double)raw_gy * GYRO_SCALE;
            sum_gz += (double)raw_gz * GYRO_SCALE;
            valid_cnt++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (valid_cnt > 0) {
        gyro_bias_x_ = sum_gx / valid_cnt;
        gyro_bias_y_ = sum_gy / valid_cnt;
        gyro_bias_z_ = sum_gz / valid_cnt;
    }
}

IMUData BNO085::read_mpu6050() noexcept {
    IMUData data{};
    data.valid = false;
    uint8_t buf[14];
    if (bus_->read_reg(addr_, 0x3B, buf, 14) != ESP_OK) return data;

    int16_t raw_ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t raw_ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t raw_az = (int16_t)((buf[4] << 8) | buf[5]);

    int16_t raw_gx = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t raw_gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t raw_gz = (int16_t)((buf[12] << 8) | buf[13]);

    // Convert to m/s^2 (at ±8g, 4096 LSB/g)
    constexpr double ACCEL_SCALE = (8.0 * 9.80665) / 32768.0;
    data.acc_x = (double)raw_ax * ACCEL_SCALE;
    data.acc_y = (double)raw_ay * ACCEL_SCALE;
    data.acc_z = (double)raw_az * ACCEL_SCALE;

    // Convert to rad/s (at ±2000 dps, 16.4 LSB/(deg/s)) with calibrated zero-bias subtracted
    constexpr double GYRO_SCALE = (2000.0 * 3.141592653589793) / (180.0 * 32768.0);
    data.gyr_x = (double)raw_gx * GYRO_SCALE - gyro_bias_x_;
    data.gyr_y = (double)raw_gy * GYRO_SCALE - gyro_bias_y_;
    data.gyr_z = (double)raw_gz * GYRO_SCALE - gyro_bias_z_;

    data.mag_x = 0.0; data.mag_y = 0.0; data.mag_z = 0.0;
    data.mag_valid = false;
    data.saturated = false;
    data.valid = true;
    return data;
}

IMUData BNO085::read_bno055() noexcept {
    IMUData data{};
    data.valid = false;
    uint8_t buf[32];
    // Read Accel (0x08..0x0D), Mag (0x0E..0x13), Gyro (0x14..0x19), Euler (0x1A..0x1F), Quat (0x20..0x27)
    if (bus_->read_reg(addr_, 0x08, buf, 32) != ESP_OK) return data;

    int16_t ax = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t ay = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t az = (int16_t)((buf[5] << 8) | buf[4]);

    int16_t mx = (int16_t)((buf[7] << 8) | buf[6]);
    int16_t my = (int16_t)((buf[9] << 8) | buf[8]);
    int16_t mz = (int16_t)((buf[11] << 8) | buf[10]);

    int16_t gx = (int16_t)((buf[13] << 8) | buf[12]);
    int16_t gy = (int16_t)((buf[15] << 8) | buf[14]);
    int16_t gz = (int16_t)((buf[17] << 8) | buf[16]);

    // BNO055 raw sensor specific force: 1 m/s² = 100 LSB
    double raw_acc[3] = { (double)ax / 100.0, (double)ay / 100.0, (double)az / 100.0 };

    // BNO055 raw sensor angular velocity: 1 rad/s = 16 LSB * (180/pi)
    constexpr double GYRO_SCALE = (3.141592653589793 / 180.0) / 16.0;
    double raw_gyr[3] = { (double)gx * GYRO_SCALE, (double)gy * GYRO_SCALE, (double)gz * GYRO_SCALE };

    // BNO055 raw magnetic field: 1 µT = 16 LSB
    data.mag_x = (double)mx / 16.0;
    data.mag_y = (double)my / 16.0;
    data.mag_z = (double)mz / 16.0;
    data.mag_valid = true;

    // Read 9-DOF fused quaternion (registers 0x20..0x27): 1 unit = 2^14 LSB = 16384 LSB
    int16_t qw = (int16_t)((buf[25] << 8) | buf[24]);
    int16_t qx = (int16_t)((buf[27] << 8) | buf[26]);
    int16_t qy = (int16_t)((buf[29] << 8) | buf[28]);
    int16_t qz = (int16_t)((buf[31] << 8) | buf[30]);

    constexpr double QUAT_SCALE = 1.0 / 16384.0;
    Quat q_sensor = {
        (double)qw * QUAT_SCALE,
        (double)qx * QUAT_SCALE,
        (double)qy * QUAT_SCALE,
        (double)qz * QUAT_SCALE
    };

    uint64_t now_us = (uint64_t)esp_timer_get_time();
    double dt_s = (last_read_us_ > 0) ? (double)(now_us - last_read_us_) * 1e-6 : 0.01;
    last_read_us_ = now_us;
    if (dt_s <= 0.0 || dt_s > 0.5) dt_s = 0.01;
    data.timestamp_s = (double)now_us * 1e-6;   // MCU clock at the sample instant

    // Feed 9-DOF fused quaternion into AttitudeReference: auto-detects parallel vs perpendicular mount
    AttitudeOutput att = attitude_ref_.update(q_sensor, raw_acc, raw_gyr, dt_s);

    // Transform specific force and angular velocity from sensor frame to vehicle body frame
    double veh_acc[3];
    double veh_gyr[3];
    attitude_ref_.sensor_to_vehicle(raw_acc, veh_acc);
    attitude_ref_.sensor_to_vehicle(raw_gyr, veh_gyr);

    data.acc_x = veh_acc[0];
    data.acc_y = veh_acc[1];
    data.acc_z = veh_acc[2];

    data.gyr_x = veh_gyr[0];
    data.gyr_y = veh_gyr[1];
    data.gyr_z = veh_gyr[2];

    if (att.referenced) {
        data.euler_pitch_deg = att.tilt_x_deg;
        data.euler_roll_deg  = att.tilt_y_deg;
        data.euler_yaw_deg   = att.rot_z_deg;
        data.quat_w          = att.q_rel.w;
        data.quat_x          = att.q_rel.x;
        data.quat_y          = att.q_rel.y;
        data.quat_z          = att.q_rel.z;
        data.mount           = att.mount;
        data.euler_valid     = true;
        data.quat_valid      = true;
    } else {
        // Not referenced yet (board moving / fusion converging): report level, not stale angles
        data.euler_pitch_deg = 0.0;
        data.euler_roll_deg  = 0.0;
        data.euler_yaw_deg   = 0.0;
        data.euler_valid     = true;
        data.quat_valid      = false;
    }

    data.valid = true;
    return data;
}

IMUData BNO085::read() noexcept {
    if (!ready_ || !bus_) return IMUData{};
    if (imu_type_ == IMUType::MPU6050) {
        return read_mpu6050();
    }
    if (imu_type_ == IMUType::BNO055) {
        return read_bno055();
    }

    IMUData data{};
    data.valid = false;

    uint8_t buf[SHTP_MAX];
    int n = shtp_read(buf, SHTP_MAX);
    if (n < (int)SHTP_HDR) return data;

    parse_input_report(buf + SHTP_HDR, n - SHTP_HDR, data);
    return data;
}

void BNO085::reset() noexcept {
    if (!bus_) return;
    uint8_t cmd[1] = {0x01};
    shtp_write(CHANNEL_EXE, cmd, 1);
    vTaskDelay(pdMS_TO_TICKS(300));
    ready_ = false;
}

// ---------------------------------------------------------------------------
// SHTP Layer
// ---------------------------------------------------------------------------

esp_err_t BNO085::shtp_write(uint8_t channel, const uint8_t* payload, size_t len) noexcept {
    uint8_t buf[SHTP_MAX];
    uint16_t total = (uint16_t)(len + SHTP_HDR);
    buf[0] = (uint8_t)(total & 0xFF);
    buf[1] = (uint8_t)(total >> 8);
    buf[2] = channel;
    buf[3] = seq_[channel]++;
    memcpy(buf + SHTP_HDR, payload, len);
    return bus_->write_reg(addr_, 0x00, buf, total);
}

int BNO085::shtp_read(uint8_t* buf, size_t max_len) noexcept {
    // Read 4-byte header first to determine packet length
    uint8_t hdr[4];
    if (bus_->read_reg(addr_, 0x00, hdr, 4) != ESP_OK) return -1;
    uint16_t len = (uint16_t)(hdr[0] | ((hdr[1] & 0x7F) << 8));
    if (len < 4 || len > max_len) return -1;
    buf[0]=hdr[0]; buf[1]=hdr[1]; buf[2]=hdr[2]; buf[3]=hdr[3];
    if (len > 4) {
        if (bus_->read_reg(addr_, 0x00, buf + 4, len - 4) != ESP_OK) return -1;
    }
    return (int)len;
}

esp_err_t BNO085::enable_report(uint8_t report_id, uint32_t interval_us) noexcept {
    // Set Feature Command: report 0xFD
    uint8_t cmd[17] = {};
    cmd[0]  = 0xFD;                          // Set Feature Command
    cmd[1]  = report_id;                     // Feature Report ID
    cmd[2]  = 0x00;                          // Flags (no wake-up, no change sensitivity)
    cmd[3]  = 0x00;                          // Change sensitivity (LSB)
    cmd[4]  = 0x00;                          // Change sensitivity (MSB)
    cmd[5]  = (uint8_t)(interval_us & 0xFF); // Report interval µs [0]
    cmd[6]  = (uint8_t)((interval_us >> 8)  & 0xFF);
    cmd[7]  = (uint8_t)((interval_us >> 16) & 0xFF);
    cmd[8]  = (uint8_t)((interval_us >> 24) & 0xFF);
    // Batch interval (0 = no batching)
    cmd[9]  = 0; cmd[10] = 0; cmd[11] = 0; cmd[12] = 0;
    // Sensor-specific config (0)
    cmd[13] = 0; cmd[14] = 0; cmd[15] = 0; cmd[16] = 0;
    return shtp_write(CHANNEL_CONTROL, cmd, 17);
}

esp_err_t BNO085::parse_input_report(const uint8_t* buf, size_t len,
                                      IMUData& out) noexcept {
    // BNO085 input report layout (after SHTP header):
    //   byte 0: Report ID
    //   byte 1: Sequence number
    //   byte 2: Status (accuracy, etc.)
    //   byte 3: Delay (ms)
    //   bytes 4-5: X (int16)  [Q-point varies per report]
    //   bytes 6-7: Y (int16)
    //   bytes 8-9: Z (int16)
    if (len < 10) return ESP_ERR_INVALID_SIZE;

    const uint8_t report_id = buf[0];
    const int16_t x = (int16_t)((buf[5] << 8) | buf[4]);
    const int16_t y = (int16_t)((buf[7] << 8) | buf[6]);
    const int16_t z = (int16_t)((buf[9] << 8) | buf[8]);

    if (report_id == REPORT_ACCELEROMETER) {
        // Q-point 8 → m/s²
        out.acc_x = x * Q8_SCALE;
        out.acc_y = y * Q8_SCALE;
        out.acc_z = z * Q8_SCALE;
        out.valid = true;
        // Saturation check (±156.96 m/s² = ±16g)
        const double sat_lim = nav::IMU_CFG.accel_saturation_mps2 * 0.99;
        out.saturated = (std::abs(out.acc_x) >= sat_lim ||
                         std::abs(out.acc_y) >= sat_lim ||
                         std::abs(out.acc_z) >= sat_lim);
    } else if (report_id == REPORT_GYROSCOPE_CALIB) {
        // Q-point 9 → rad/s
        out.gyr_x = x * Q9_SCALE;
        out.gyr_y = y * Q9_SCALE;
        out.gyr_z = z * Q9_SCALE;
        out.valid = true;
        const double sat_lim = nav::IMU_CFG.gyro_saturation_radps * 0.99;
        out.saturated = (std::abs(out.gyr_x) >= sat_lim ||
                         std::abs(out.gyr_y) >= sat_lim ||
                         std::abs(out.gyr_z) >= sat_lim);
    } else if (report_id == REPORT_MAGNETOMETER) {
        // Q-point 4 → µT
        out.mag_x = x * Q4_SCALE;
        out.mag_y = y * Q4_SCALE;
        out.mag_z = z * Q4_SCALE;
        out.mag_valid = true;
    }
    return ESP_OK;
}

} // namespace drivers
