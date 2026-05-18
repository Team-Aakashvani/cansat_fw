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
#include <cstring>
#include <cmath>

static const char* TAG = "BNO085";

namespace drivers {

// SHTP header: 4 bytes [length_lsb, length_msb, channel, seq_num]
static constexpr size_t SHTP_HDR = 4;
static constexpr size_t SHTP_MAX = 128;

esp_err_t BNO085::init(hal::I2CBus& bus, uint8_t addr, double rate_hz) noexcept {
    bus_  = &bus;
    addr_ = addr;

    // Wait for BNO085 to finish boot
    vTaskDelay(pdMS_TO_TICKS(200));

    // Probe
    if (!bus_->probe(addr_)) {
        ESP_LOGE(TAG, "BNO085 not found at 0x%02X", addr_);
        return ESP_ERR_NOT_FOUND;
    }

    // Soft reset via SHTP executable channel
    uint8_t reset_cmd[1] = { 0x01 };
    esp_err_t ret = shtp_write(CHANNEL_EXE, reset_cmd, 1);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(300));

    // Calculate report interval in microseconds
    uint32_t interval_us = (uint32_t)(1.0e6 / rate_hz);

    // Enable calibrated accelerometer
    ret = enable_report(REPORT_ACCELEROMETER, interval_us);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Accel report failed"); return ret; }

    // Enable calibrated gyroscope
    ret = enable_report(REPORT_GYROSCOPE_CALIB, interval_us);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Gyro report failed"); return ret; }

    // Enable calibrated magnetometer
    ret = enable_report(REPORT_MAGNETOMETER, interval_us);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Mag report failed"); return ret; }

    ready_ = true;
    ESP_LOGI(TAG, "BNO085 ready @ %.0f Hz", rate_hz);
    return ESP_OK;
}

IMUData BNO085::read() noexcept {
    IMUData data{};
    data.valid = false;
    if (!ready_ || !bus_) return data;

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
