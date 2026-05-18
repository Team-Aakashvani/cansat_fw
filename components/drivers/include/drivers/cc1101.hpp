/**
 * @file cc1101.hpp
 * @brief CC1101 sub-1GHz RF transceiver driver (SPI) — RSSI scanner mode.
 *
 * Used exclusively as an RF raster scanner for the CAN-7USAT payload.
 * Shares SPI2_HOST with the SX1278 LoRa transceiver via spi_bus_add_device().
 *
 * The CC1101 sweeps 300–928 MHz and reads RSSI per channel to build
 * an RF interference map. It does NOT transmit — RX-only scanner.
 *
 * SPI bus arbitration: the caller MUST wrap set_frequency() inside
 * spi_device_acquire_bus() / spi_device_release_bus() to prevent
 * the LoRa driver from interleaving mid-PLL-strobe.
 */
#pragma once

#include "hal/spi_bus.hpp"
#include "driver/spi_master.h"
#include <cstdint>

namespace drivers {

class CC1101 {
public:
    CC1101() noexcept = default;

    /// Initialise — registers as a second device on the existing SPI bus.
    /// Does NOT call spi_bus_initialize() — bus is already up.
    esp_err_t init(hal::SPIBus& spi, int cs_pin) noexcept;

    /// Tune to a new frequency. MUST be called inside acquire/release block.
    /// PLL sequence: SIDLE → write FREQ2/1/0 → SCAL → SRX
    void set_frequency(uint32_t freq_hz) noexcept;

    /// Read current RSSI in dBm (valid only after set_frequency + ~2ms settle).
    int8_t read_rssi_dbm() noexcept;

    /// Get the raw SPI device handle (needed for acquire/release bus calls).
    spi_device_handle_t device_handle() const noexcept { return dev_; }

    bool is_ready() const noexcept { return ready_; }

private:
    hal::SPIBus*        spi_   = nullptr;
    spi_device_handle_t dev_   = nullptr;
    bool                ready_ = false;

    // CC1101 register addresses
    static constexpr uint8_t REG_FREQ2       = 0x0D;
    static constexpr uint8_t REG_FREQ1       = 0x0E;
    static constexpr uint8_t REG_FREQ0       = 0x0F;
    static constexpr uint8_t REG_RSSI        = 0x34;  // Status register (burst read)

    // CC1101 command strobes
    static constexpr uint8_t STROBE_SIDLE    = 0x36;
    static constexpr uint8_t STROBE_SCAL     = 0x33;
    static constexpr uint8_t STROBE_SRX      = 0x34;

    void send_strobe(uint8_t strobe) noexcept;
    void write_reg(uint8_t reg, uint8_t val) noexcept;
    uint8_t read_status_reg(uint8_t reg) noexcept;
};

} // namespace drivers
