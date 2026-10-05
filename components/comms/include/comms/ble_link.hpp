/**
 * @file ble_link.hpp
 * @brief Bluetooth LE health / command link (Nordic UART Service profile).
 *
 * Text lines in both directions, same format as the USB console:
 *   CanSat -> ground : notifications on TX  6E400003-B5A3-F393-E0A9-E50E24DCCA9E
 *   ground -> CanSat : writes to RX         6E400002-B5A3-F393-E0A9-E50E24DCCA9E
 *   service                                 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 * Works with the dock and with generic phone apps (nRF Connect, Serial Bluetooth Terminal).
 *
 * Intended for the pad and for recovery: main.cpp switches it off once a real flight
 * starts (2.4 GHz shared with the XBee) and back on after landing.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

namespace comms {

class BleLink {
public:
    using LineHandler = std::function<void(const char* line)>;

    /// Start the NimBLE host and advertise as `name`. Received lines go to `on_line`
    /// (called from the BLE host task: keep it short, queue real work).
    bool init(const char* name, LineHandler on_line) noexcept;

    /// Advertise / accept connections (true) or drop the link and go silent (false).
    void set_enabled(bool en) noexcept;
    bool enabled() const noexcept;

    /// True when a client is connected and subscribed to notifications.
    bool connected() const noexcept;

    /// Send one text line (a trailing '\n' is added). Split into MTU-sized notifications.
    /// Non-blocking: returns false (and drops) if not connected or out of buffers.
    bool send_line(const char* line) noexcept;
};

} // namespace comms
