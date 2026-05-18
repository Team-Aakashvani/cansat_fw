/**
 * @file lora_link.hpp
 * @brief LoRa RF link — 1Hz telemetry downlink + uplink command receive.
 *
 * Wraps the SX1278 driver with a thread-safe TX queue and an RX callback.
 * Operates at 433MHz, SF10, BW125kHz, CR4/5 per CAN-7USAT §6.2.
 *
 * TX packet format (downlink):
 *   <TEAM_ID>,<CSV_PAYLOAD>\n    (plain ASCII, max 256 bytes)
 *
 * RX packet format (uplink commands):
 *   <TEAM_ID>,<CMD>[,<ARG>]\n
 *   e.g. "1234,CX,ON"  or  "1234,ST,01:30:00"
 *
 * This class is NOT responsible for building the CSV payload; that is done by
 * TelemetryEncoder. Call enqueue_packet() with an already-encoded CSV string.
 *
 * Thread safety: enqueue_packet() and set_rx_callback() may be called from any
 * task. The internal TX/RX work is driven by calling spin() from a dedicated
 * LoRa task at ~1Hz.
 *
 * @compliance CAN-7USAT 2026 §6.2 (RF link spec)
 */
#pragma once

#include "drivers/sx1278.hpp"
#include "nav/config.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <cstdint>
#include <cstddef>
#include <functional>

namespace comms {

/// Uplink command types decoded from received packets.
enum class CommandType : uint8_t {
    UNKNOWN = 0,
    CX,       ///< Enable/disable telemetry: CX,ON | CX,OFF
    ST,       ///< Set mission time: ST,HH:MM:SS
    SIM,      ///< Simulation mode: SIM,ENABLE | SIM,DISABLE | SIM,ACTIVATE
    SIMP,     ///< Simulated pressure value: SIMP,<pa>
    SIMG,     ///< Simulated GNSS value: SIMG,<lat>,<lon>,<alt>,<ve>,<vn>,<vu>
    SIMI,     ///< Simulated IMU value: SIMI,<ax>,<ay>,<az>,<gx>,<gy>,<gz>
    CAL,      ///< Calibrate ground altitude: CAL
    ABORT,    ///< Emergency mission abort
    CHUTE,    ///< Manual parachute deployment
    RTL,      ///< Return to Launch (controlled descent)
    MAP,      ///< RF mapping toggle
    OTA,      ///< OTA update: OTA,START | OTA,CHUNK,<hex> | OTA,FINISH | OTA,ABORT
};

struct UplinkCommand {
    CommandType type;
    char        arg[32];  ///< Null-terminated argument string (may be empty)
    int8_t      rssi_dbm;
    float       snr_db;
};

/// Callback invoked from the LoRa task when a valid uplink command is received.
using RxCallback = std::function<void(const UplinkCommand&)>;

class LoRaLink {
public:
    static constexpr size_t TX_BUF_LEN   = 256;
    static constexpr size_t RX_BUF_LEN   = 128;
    static constexpr int    TX_QUEUE_LEN  = 4;

    LoRaLink() noexcept = default;

    /**
     * @brief Initialise LoRa link.
     * @param spi  Initialised SPI bus (shared with SX1278 driver).
     * @param cs_pin   Chip-select GPIO.
     * @param rst_pin  Reset GPIO.
     * @param irq_pin  DIO0 IRQ GPIO.
     * @return ESP_OK on success.
     */
    esp_err_t init(hal::SPIBus& spi, int cs_pin, int rst_pin, int irq_pin) noexcept;

    /**
     * @brief Enqueue a CSV telemetry string for transmission.
     *
     * Thread-safe. If the queue is full the oldest entry is dropped.
     * @param csv  Null-terminated CSV string (output of TelemetryEncoder).
     * @param len  Length excluding null terminator.
     * @return true if enqueued, false if dropped.
     */
    bool enqueue_packet(const char* csv, size_t len) noexcept;

    /**
     * @brief Register uplink command callback.
     * Thread-safe — replaces previous callback atomically.
     */
    void set_rx_callback(RxCallback cb) noexcept;

    /**
     * @brief Drive TX/RX — call from a dedicated FreeRTOS task.
     * @return true if a packet was received.
     */
    bool spin() noexcept;

    /// Get raw data of last received packet (valid only if spin() returned true).
    const uint8_t* last_rx_data() const noexcept { return last_rx_pkt_.data; }
    size_t         last_rx_len()  const noexcept { return (size_t)last_rx_pkt_.len; }

    /**
     * @brief Set promiscuous mode (GCS bridge).
     * If true, spin() will not attempt to parse uplink commands and will 
     * return true for any valid CRC packet.
     */
    void set_promiscuous(bool enable) noexcept { promiscuous_ = enable; }

    /// Return false if the radio failed to initialise.
    bool is_ready() const noexcept { return ready_; }

    /// Statistics (read-only from other tasks).
    uint32_t tx_count()    const noexcept { return tx_count_; }
    uint32_t rx_count()    const noexcept { return rx_count_; }
    uint32_t tx_errors()   const noexcept { return tx_errors_; }

private:
    struct TxEntry {
        char buf[TX_BUF_LEN];
        size_t len;
    };

    drivers::SX1278  radio_{};
    drivers::LoRaRxPacket last_rx_pkt_{};
    QueueHandle_t    tx_queue_{nullptr};
    SemaphoreHandle_t cb_mutex_{nullptr};
    RxCallback       rx_cb_{};

    bool     ready_       = false;
    bool     promiscuous_ = false;
    uint32_t tx_count_    = 0;
    uint32_t rx_count_    = 0;
    uint32_t tx_errors_ = 0;

    /// Parse a raw received buffer into an UplinkCommand.
    /// Returns false if team ID mismatch or parse error.
    bool parse_uplink(const uint8_t* buf, size_t len, UplinkCommand& out) const noexcept;
};

} // namespace comms
