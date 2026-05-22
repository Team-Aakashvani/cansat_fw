/**
 * @file built_in_test.cpp
 * @brief Power-On Self-Test implementation.
 */
#include "bit/built_in_test.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>
#include <cstring>
#include <cstdio>

static const char* TAG = "BIT";

namespace bit {

// ---------------------------------------------------------------------------
// Public: run
// ---------------------------------------------------------------------------
BITResult BuiltInTest::run(hal::I2CBus&   i2c,
                            hal::SPIBus&   spi,
                            hal::UARTBus&  gnss_uart,
                            hal::UARTBus&  xbee_uart,
                            logging::SDLogger*     sd,
                            config_mgr::NVSConfig& cfg) noexcept {
    ESP_LOGI(TAG, "--- BIT START ---");
    uint32_t flags = 0;

    flags |= test_imu   (i2c);
    flags |= test_baro  (i2c);
    flags |= test_power (i2c);
    flags |= test_gnss  (gnss_uart);
    flags |= test_xbee  (xbee_uart);
    flags |= test_sd    (sd);
    flags |= test_nvs   (cfg);

    BITResult r{flags};
    print_result(r);
    return r;
}

// ---------------------------------------------------------------------------
// Internal tests
// ---------------------------------------------------------------------------
uint32_t BuiltInTest::test_imu(hal::I2CBus& i2c) noexcept {
    uint32_t flags = 0;
    drivers::BNO085 imu;
    if (imu.init(i2c) != ESP_OK) {  // default addr 0x4A
        ESP_LOGE(TAG, "[BIT] IMU: ABSENT");
        return BIT_IMU_ABSENT;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    drivers::IMUData d = imu.read();
    if (!d.valid) {
        ESP_LOGE(TAG, "[BIT] IMU: read failed");
        flags |= BIT_IMU_ABSENT;
        return flags;
    }
    // Sanity: |a| ≈ g ± 2 m/s²
    double amag = std::sqrt((double)d.acc_x*(double)d.acc_x +
                            (double)d.acc_y*(double)d.acc_y +
                            (double)d.acc_z*(double)d.acc_z);
    if (amag < 7.8 || amag > 11.8) {
        ESP_LOGW(TAG, "[BIT] IMU sanity: |a|=%.2f m/s² (expected ~9.81)", amag);
        flags |= BIT_IMU_SANITY;
    } else {
        ESP_LOGI(TAG, "[BIT] IMU: PASS (|a|=%.2f m/s²)", amag);
    }
    return flags;
}

uint32_t BuiltInTest::test_baro(hal::I2CBus& i2c) noexcept {
    uint32_t flags = 0;
    drivers::BMP585 baro;
    if (baro.init(i2c) != ESP_OK) {  // default addr 0x46
        ESP_LOGE(TAG, "[BIT] BARO: ABSENT");
        return BIT_BARO_ABSENT;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    drivers::BaroData d = baro.read();
    if (!d.valid || d.pressure_pa < 70000.0 || d.pressure_pa > 110000.0) {
        ESP_LOGW(TAG, "[BIT] BARO sanity: P=%.1f Pa", (double)(d.valid ? d.pressure_pa : 0));
        flags |= BIT_BARO_SANITY;
    } else {
        ESP_LOGI(TAG, "[BIT] BARO: PASS (P=%.1f Pa T=%.1f°C alt=%.1fm)",
                 (double)d.pressure_pa, (double)d.temperature_c, (double)d.altitude_agl_m);
    }
    return flags;
}

uint32_t BuiltInTest::test_power(hal::I2CBus& i2c) noexcept {
    drivers::INA260 ina;
    if (ina.init(i2c) != ESP_OK) {  // default addr 0x40
        ESP_LOGE(TAG, "[BIT] INA260: ABSENT");
        return BIT_POWER_ABSENT;
    }
    drivers::PowerData d = ina.read();
    if (!d.valid) return BIT_POWER_ABSENT;

    uint32_t flags = 0;
    if (d.voltage_v < 3.0f) {
        ESP_LOGE(TAG, "[BIT] Voltage critically low: %.2fV", (double)d.voltage_v);
        flags |= BIT_VOLTAGE_LOW;
    } else {
        ESP_LOGI(TAG, "[BIT] Power: PASS (V=%.2fV I=%.3fA P=%.2fW)",
                 (double)d.voltage_v, (double)d.current_a, (double)d.power_w);
    }
    return flags;
}

uint32_t BuiltInTest::test_gnss(hal::UARTBus& uart) noexcept {
    // Wait up to 2s for any NMEA sentence
    char line[128];
    for (int attempts = 0; attempts < 20; ++attempts) {
        if (uart.read_line(line, sizeof(line), 100) > 0) {
            if (line[0] == '$') {
                ESP_LOGI(TAG, "[BIT] GNSS: PASS (got NMEA)");
                return 0;
            }
        }
    }
    ESP_LOGW(TAG, "[BIT] GNSS: no NMEA sentence in 2s");
    return BIT_GNSS_NO_NMEA;
}

uint32_t BuiltInTest::test_xbee(hal::UARTBus& xbee_uart) noexcept {
    // Basic connectivity check for XBee
    if (!xbee_uart.is_initialised()) {
        ESP_LOGE(TAG, "[BIT] XBee: NOT INITIALISED");
        return BIT_LORA_ABSENT; // Keep flag for compatibility
    }
    ESP_LOGI(TAG, "[BIT] XBee: PASS");
    return 0;
}

uint32_t BuiltInTest::test_sd(logging::SDLogger* sd) noexcept {
    if (!sd) return BIT_SD_FAIL;
    if (!sd->is_mounted()) {
        ESP_LOGW(TAG, "[BIT] SD: not mounted");
        return BIT_SD_FAIL;
    }
    bool ok = sd->write_line("BIT_TEST,0,0,0,0,0,0,00:00:00,0,0,0,0,0,0,0,0\n");
    if (!ok) {
        ESP_LOGW(TAG, "[BIT] SD: write_line failed");
        return BIT_SD_FAIL;
    }
    sd->flush();
    ESP_LOGI(TAG, "[BIT] SD: PASS");
    return 0;
}

uint32_t BuiltInTest::test_nvs(config_mgr::NVSConfig& cfg) noexcept {
    uint16_t tid = cfg.get_team_id();
    if (tid == 0) {
        ESP_LOGW(TAG, "[BIT] NVS: team_id=0 (default)");
    } else {
        ESP_LOGI(TAG, "[BIT] NVS: PASS (team_id=%u)", (unsigned)tid);
    }
    return 0;
}

// ---------------------------------------------------------------------------
// print_result
// ---------------------------------------------------------------------------
void BuiltInTest::print_result(const BITResult& r) noexcept {
    if (r.pass()) {
        ESP_LOGI(TAG, "=== BIT PASS (flags=0x%08X) ===", (unsigned)r.flags);
    } else {
        ESP_LOGE(TAG, "=== BIT FAIL (flags=0x%08X) ===", (unsigned)r.flags);
        if (r.flags & BIT_IMU_ABSENT)   ESP_LOGE(TAG, "  FAIL: IMU absent");
        if (r.flags & BIT_BARO_ABSENT)  ESP_LOGE(TAG, "  FAIL: Baro absent");
        if (r.flags & BIT_POWER_ABSENT) ESP_LOGE(TAG, "  FAIL: Power monitor absent");
        if (r.flags & BIT_LORA_ABSENT)  ESP_LOGE(TAG, "  FAIL: LoRa absent");
    }
    if (r.flags & BIT_GNSS_NO_NMEA) ESP_LOGW(TAG, "  WARN: GNSS no NMEA");
    if (r.flags & BIT_SD_FAIL)      ESP_LOGW(TAG, "  WARN: SD card fail");
    if (r.flags & BIT_IMU_SANITY)   ESP_LOGW(TAG, "  WARN: IMU specific-force out of range");
    if (r.flags & BIT_BARO_SANITY)  ESP_LOGW(TAG, "  WARN: Baro pressure out of range");
    if (r.flags & BIT_VOLTAGE_LOW)  ESP_LOGW(TAG, "  WARN: Battery voltage low");
}

} // namespace bit
