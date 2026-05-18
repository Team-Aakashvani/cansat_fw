/**
 * @file cc1101.cpp
 * @brief CC1101 RF scanner driver implementation.
 *
 * FILE LOCATION: components/drivers/src/cc1101.cpp
 *
 * Remember to add "cc1101.cpp" to the SRCS list in
 * components/drivers/CMakeLists.txt
 */

#include "drivers/cc1101.hpp"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "CC1101";

namespace drivers {

esp_err_t CC1101::init(hal::SPIBus& spi, int cs_pin) noexcept
{
    spi_ = &spi;

    // Register the CC1101 as a second device on the existing SPI bus.
    // The SX1278 is already device #1. The CC1101 gets its own CS pin
    // so both chips can share the same MOSI/MISO/SCK wires.
    // Clock speed: 6 MHz (CC1101 max is ~6.5 MHz for single-byte access)
    esp_err_t ret = spi.add_device(cs_pin, 6000000, dev_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add CC1101 SPI device: %s", esp_err_to_name(ret));
        return ret;
    }

    // Quick sanity check: read a register and see if we get a plausible value
    // (In production you'd read the VERSION register, but for now just mark ready)
    ready_ = true;
    ESP_LOGI(TAG, "CC1101 initialised on CS pin %d", cs_pin);
    return ESP_OK;
}

void CC1101::set_frequency(uint32_t freq_hz) noexcept
{
    if (!ready_) return;

    // CC1101 frequency register formula:
    //   FREQ = (desired_freq_hz * 2^16) / 26_000_000
    uint32_t freq_word = (uint32_t)(((uint64_t)freq_hz << 16) / 26000000ULL);

    uint8_t freq2 = (freq_word >> 16) & 0xFF;
    uint8_t freq1 = (freq_word >>  8) & 0xFF;
    uint8_t freq0 =  freq_word        & 0xFF;

    // PLL sequence (MANDATORY — skipping causes bogus RSSI):
    // This whole sequence MUST happen atomically on the SPI bus.
    spi_device_acquire_bus(dev_, portMAX_DELAY);

    send_strobe(STROBE_SIDLE);
    write_reg(REG_FREQ2, freq2);
    write_reg(REG_FREQ1, freq1);
    write_reg(REG_FREQ0, freq0);
    send_strobe(STROBE_SCAL);
    send_strobe(STROBE_SRX);

    spi_device_release_bus(dev_);
}

int8_t CC1101::read_rssi_dbm() noexcept
{
    if (!ready_) return -128;

    // Read the raw RSSI status register
    uint8_t raw = read_status_reg(REG_RSSI);

    // CC1101 RSSI formula (from datasheet):
    //   If raw >= 128: rssi_dbm = (raw - 256) / 2 - 74
    //   If raw <  128: rssi_dbm = raw / 2 - 74
    int8_t rssi_dbm;
    if (raw >= 128) {
        rssi_dbm = (int8_t)(((int)raw - 256) / 2 - 74);
    } else {
        rssi_dbm = (int8_t)((int)raw / 2 - 74);
    }

    return rssi_dbm;
}

// ---- Private helper functions ----

void CC1101::send_strobe(uint8_t strobe) noexcept
{
    // A strobe is a single-byte SPI command with no data payload.
    // Writing just the command byte triggers the action.
    uint8_t cmd = strobe;
    spi_->write_reg(dev_, cmd, nullptr, 0);
}

void CC1101::write_reg(uint8_t reg, uint8_t val) noexcept
{
    spi_->write_reg(dev_, reg, &val, 1);
}

uint8_t CC1101::read_status_reg(uint8_t reg) noexcept
{
    // CC1101 status registers are read with bit 7=1, bit 6=1 (burst + read)
    // So the address byte is: reg | 0xC0
    uint8_t val = 0;
    spi_->read_reg(dev_, reg | 0xC0, &val, 1);
    return val;
}

} // namespace drivers
