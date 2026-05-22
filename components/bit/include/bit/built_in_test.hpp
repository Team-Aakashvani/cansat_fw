/**
 * @file built_in_test.hpp
 * @brief Power-On Self-Test (POST) / Built-In Test (BIT) sequence.
 *
 * Executed once at boot before entering the main flight loop. Tests:
 *   1. I2C bus scan — detects BNO085, BMP585, INA260, MAX17048, SDP31, SHT4x, SGP41
 *   2. BNO085 — verify CHIP_ID, read one IMU sample
 *   3. BMP585 — verify CHIP_ID, read one baro sample
 *   4. INA260  — verify connectivity, voltage in range [3.0, 4.5]V
 *   5. N-GS-01 — UART RX test (look for NMEA sentence within 2s)
 *   6. SX1278  — read version register (expect 0x12)
 *   7. SD card  — mount and write one test record
 *   8. NVS      — open and close config namespace
 *   9. IMU data sanity — specific force ~9.81 m/s² ±2
 *  10. Baro data sanity — pressure in [70000, 110000] Pa
 *
 * Result is encoded as a bitmask (BITResult). All-zero = pass.
 * On hard failure (critical sensor absent) the system halts and beeps.
 * On soft failure (non-critical sensor absent) it logs and continues.
 *
 * @compliance CAN-7USAT §7.1 (system self-test requirement)
 */
#pragma once

#include "hal/i2c_bus.hpp"
#include "hal/spi_bus.hpp"
#include "hal/uart_bus.hpp"
#include "drivers/bno085.hpp"
#include "drivers/bmp585.hpp"
#include "drivers/ngps01.hpp"
#include "comms/xbee_link.hpp"
#include "drivers/ina260.hpp"
#include "drivers/max17048.hpp"
#include "logging/sd_logger.hpp"
#include "config_mgr/nvs_config.hpp"
#include "esp_err.h"
#include <cstdint>

namespace bit {

/// Bitmask of BIT failures. 0 = pass.
enum BITFlags : uint32_t {
    BIT_OK             = 0,
    BIT_IMU_ABSENT     = (1u << 0),
    BIT_BARO_ABSENT    = (1u << 1),
    BIT_POWER_ABSENT   = (1u << 2),
    BIT_GNSS_NO_NMEA   = (1u << 3),
    BIT_LORA_ABSENT    = (1u << 4),
    BIT_SD_FAIL        = (1u << 5),
    BIT_NVS_FAIL       = (1u << 6),
    BIT_IMU_SANITY     = (1u << 7),
    BIT_BARO_SANITY    = (1u << 8),
    BIT_VOLTAGE_LOW    = (1u << 9),
};

/// OR of BITFlags that must be zero to proceed to flight. Soft failures may be non-zero.
static constexpr uint32_t BIT_CRITICAL_MASK =
    BIT_IMU_ABSENT | BIT_BARO_ABSENT | BIT_POWER_ABSENT | BIT_LORA_ABSENT;

struct BITResult {
    uint32_t flags;        ///< OR of BITFlags
    bool     pass() const noexcept { return (flags & BIT_CRITICAL_MASK) == 0; }
};

class BuiltInTest {
public:
    BuiltInTest() noexcept = default;

    /**
     * @brief Run the full BIT sequence.
     *
     * @param i2c       Initialised I2C bus.
     * @param spi       Initialised SPI bus.
     * @param gnss_uart Initialised UART bus for N-GS-01.
     * @param xbee_uart Initialised UART bus for XBee.
     * @param sd        SD logger (must be init'd before calling this; may be null).
     * @param cfg       NVS config (must be init'd before calling this).
     * @return BITResult with pass/fail flags.
     */
    BITResult run(hal::I2CBus&    i2c,
                  hal::SPIBus&    spi,
                  hal::UARTBus&   gnss_uart,
                  hal::UARTBus&   xbee_uart,
                  logging::SDLogger*      sd,
                  config_mgr::NVSConfig&  cfg) noexcept;

    /// Print BIT results to ESP_LOGI.
    static void print_result(const BITResult& r) noexcept;

private:
    uint32_t test_imu   (hal::I2CBus& i2c) noexcept;
    uint32_t test_baro  (hal::I2CBus& i2c) noexcept;
    uint32_t test_power (hal::I2CBus& i2c) noexcept;
    uint32_t test_gnss  (hal::UARTBus& gnss_uart) noexcept;
    uint32_t test_xbee  (hal::UARTBus& xbee_uart) noexcept;
    uint32_t test_sd    (logging::SDLogger* sd) noexcept;
    uint32_t test_nvs   (config_mgr::NVSConfig& cfg) noexcept;
};

} // namespace bit
