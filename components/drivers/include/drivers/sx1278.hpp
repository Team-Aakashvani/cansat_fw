/**
 * @file sx1278.hpp
 * @brief SX1278 LoRa transceiver driver (SPI).
 *
 * Implements the CAN-7USAT radio link for:
 *   - Downlink: 1Hz telemetry CSV packets
 *   - Uplink:   Command reception (CX, ST commands per rules §6.3)
 *
 * Configuration per CAN-7USAT guidelines §4.2:
 *   - Frequency:      433MHz (ISM)
 *   - Bandwidth:      125kHz
 *   - Spreading Factor: SF10
 *   - Coding Rate:    4/5
 *   - TX Power:       17dBm (50mW)
 *   - Preamble:       8 symbols
 *   - Sync Word:      0x12 (matches NETID / PANID)
 *   - CRC:            enabled
 *   - Mode:           Explicit header
 *
 * @compliance CAN-7USAT §4.2 (Communication Requirements)
 */
#pragma once

#include "hal/spi_bus.hpp"
#include "driver/gpio.h"
#include <cstdint>
#include <cstddef>
#include <functional>

namespace drivers {

struct LoRaRxPacket {
    uint8_t  data[256];
    uint8_t  len;
    int8_t   rssi_dbm;
    float    snr_db;
    bool     crc_ok;
};

/// Callback invoked when a packet is received (called from ISR context).
using LoRaRxCallback = std::function<void(const LoRaRxPacket&)>;

class SX1278 {
public:
    SX1278() noexcept = default;

    esp_err_t init(hal::SPIBus& spi, int cs_pin, int rst_pin, int irq_pin) noexcept;

    /// Transmit `len` bytes (blocking until TX done or timeout).
    esp_err_t transmit(const uint8_t* data, uint8_t len) noexcept;

    /// Transmit a null-terminated string.
    esp_err_t transmit_str(const char* str) noexcept;

    /// Put in continuous receive mode. On packet RX, calls `cb`.
    esp_err_t start_rx(LoRaRxCallback cb) noexcept;

    /// Stop receiving (go to standby).
    void stop_rx() noexcept;

    /// Manually read last received packet (polling mode) with timeout.
    esp_err_t read_packet(LoRaRxPacket& pkt, uint32_t timeout_ms = 0) noexcept;

    /// RSSI of last received packet.
    int8_t last_rssi() const noexcept { return last_rssi_; }

    bool is_ready() const noexcept { return ready_; }

    // SX1278 Register map
    static constexpr uint8_t REG_FIFO          = 0x00;
    static constexpr uint8_t REG_OP_MODE       = 0x01;
    static constexpr uint8_t REG_FRF_MSB       = 0x06;
    static constexpr uint8_t REG_FRF_MID       = 0x07;
    static constexpr uint8_t REG_FRF_LSB       = 0x08;
    static constexpr uint8_t REG_PA_CONFIG      = 0x09;
    static constexpr uint8_t REG_LNA           = 0x0C;
    static constexpr uint8_t REG_FIFO_ADDR_PTR = 0x0D;
    static constexpr uint8_t REG_FIFO_TX_BASE  = 0x0E;
    static constexpr uint8_t REG_FIFO_RX_BASE  = 0x0F;
    static constexpr uint8_t REG_FIFO_RX_CURR  = 0x10;
    static constexpr uint8_t REG_IRQ_FLAGS_MASK= 0x11;
    static constexpr uint8_t REG_IRQ_FLAGS     = 0x12;
    static constexpr uint8_t REG_RX_NB_BYTES   = 0x13;
    static constexpr uint8_t REG_PKT_RSSI      = 0x1A;
    static constexpr uint8_t REG_PKT_SNR       = 0x19;
    static constexpr uint8_t REG_MODEM_CONFIG1 = 0x1D;
    static constexpr uint8_t REG_MODEM_CONFIG2 = 0x1E;
    static constexpr uint8_t REG_PREAMBLE_MSB  = 0x20;
    static constexpr uint8_t REG_PREAMBLE_LSB  = 0x21;
    static constexpr uint8_t REG_PAYLOAD_LEN   = 0x22;
    static constexpr uint8_t REG_MODEM_CONFIG3 = 0x26;
    static constexpr uint8_t REG_SYNC_WORD     = 0x39;
    static constexpr uint8_t REG_DIO_MAP1      = 0x40;
    static constexpr uint8_t REG_VERSION       = 0x42;
    static constexpr uint8_t REG_PA_DAC        = 0x4D;

    // Op modes
    static constexpr uint8_t MODE_LORA_SLEEP    = 0x80;
    static constexpr uint8_t MODE_LORA_STDBY    = 0x81;
    static constexpr uint8_t MODE_LORA_TX       = 0x83;
    static constexpr uint8_t MODE_LORA_RXCONT   = 0x85;
    static constexpr uint8_t MODE_LORA_RXSINGLE = 0x86;

private:
    hal::SPIBus*     spi_     = nullptr;
    spi_device_handle_t dev_  = nullptr;
    int              rst_pin_ = -1;
    int              irq_pin_ = -1;
    bool             ready_   = false;
    int8_t           last_rssi_ = 0;
    LoRaRxCallback   rx_cb_;

    void write_reg(uint8_t reg, uint8_t val) noexcept;
    uint8_t read_reg(uint8_t reg) noexcept;
    void set_frequency(uint32_t freq_hz) noexcept;
    void configure_modem() noexcept;
    static void IRAM_ATTR irq_handler(void* arg) noexcept;
};

} // namespace drivers
