/**
 * @file p4_link.hpp
 * @brief Reliable UART link to the ESP32-P4 Media Coprocessor.
 */
#pragma once

#include "hal/uart_bus.hpp"
#include <atomic>
#include <cstdint>

namespace p4_link {

struct P4Status {
    bool    connected;
    bool    recording;
    float   sd_free_gb;
    uint8_t fps;
    uint32_t last_heartbeat_ms;
};

class P4Link {
public:
    P4Link() noexcept = default;

    /**
     * @brief Initialize the link.
     * @param uart Initialized UART bus to P4
     */
    void init(hal::UARTBus& uart) noexcept;

    /// Send a command to P4 (with XOR checksum)
    void send_command(const char* cmd) noexcept;

    /// Drive reception and monitoring (call periodically)
    void spin() noexcept;

    /// Get current status of the P4 subsystem
    P4Status get_status() const noexcept;

private:
    hal::UARTBus* uart_ = nullptr;
    P4Status      status_{};
    
    static uint8_t calc_checksum(const char* s, size_t len) noexcept;
    void parse_heartbeat(const char* line) noexcept;
};

} // namespace p4_link
