/**
 * @file xbee_link.hpp
 * @brief XBee RF link — 1Hz telemetry downlink + uplink command receive.
 *
 * Wraps the UART driver for communication with XBee Pro S3B/Series 2.
 * Supports Transparent (AT) mode for simplicity and compliance.
 */
#pragma once

#include "hal/uart_bus.hpp"
#include "comms/comms_types.hpp"
#include "nav/config.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <cstdint>
#include <cstddef>

namespace comms {

class XBeeLink {
public:
    static constexpr size_t TX_BUF_LEN   = 256;
    static constexpr size_t RX_BUF_LEN   = 256;
    static constexpr int    TX_QUEUE_LEN  = 4;

    XBeeLink() noexcept = default;

    /**
     * @brief Initialise XBee link.
     * @param uart  Initialised UART bus.
     * @return ESP_OK on success.
     */
    esp_err_t init(hal::UARTBus& uart) noexcept;

    /**
     * @brief Enqueue a CSV telemetry string for transmission.
     *
     * Thread-safe. If the queue is full the oldest entry is dropped.
     * @param csv  Null-terminated CSV string.
     * @param len  Length excluding null terminator.
     * @return true if enqueued, false if dropped.
     */
    bool enqueue_packet(const char* csv, size_t len) noexcept;

    /**
     * @brief Register uplink command callback.
     */
    void set_rx_callback(RxCallback cb) noexcept;

    /**
     * @brief Drive TX/RX — call from a dedicated task at ~10-100Hz for low latency RX.
     * @return true if a packet was received.
     */
    bool spin() noexcept;

    /// Get raw data of last received packet.
    const uint8_t* last_rx_data() const noexcept { return rx_buf_; }
    size_t         last_rx_len()  const noexcept { return last_rx_len_; }

    void set_promiscuous(bool enable) noexcept { promiscuous_ = enable; }

    bool is_ready() const noexcept { return ready_; }

    uint32_t tx_count()    const noexcept { return tx_count_; }
    uint32_t rx_count()    const noexcept { return rx_count_; }
    uint32_t tx_errors()   const noexcept { return tx_errors_; }

private:
    struct TxEntry {
        char buf[TX_BUF_LEN];
        size_t len;
    };

    hal::UARTBus*    uart_{nullptr};
    QueueHandle_t    tx_queue_{nullptr};
    SemaphoreHandle_t cb_mutex_{nullptr};
    RxCallback       rx_cb_{};

    uint8_t  rx_buf_[RX_BUF_LEN];
    size_t   rx_ptr_ = 0;
    size_t   last_rx_len_ = 0;

    bool     ready_       = false;
    bool     promiscuous_ = false;
    uint32_t tx_count_    = 0;
    uint32_t rx_count_    = 0;
    uint32_t tx_errors_ = 0;

    /// Parse a raw received buffer into an UplinkCommand.
    bool parse_uplink(const uint8_t* buf, size_t len, UplinkCommand& out) const noexcept;
};

} // namespace comms
