/**
 * @file dshot.hpp
 * @brief 4-channel DShot300 ESC output on the ESP32-S3 RMT peripheral (BLHeli_S).
 *
 * One frame per channel per call to write(). The caller (100 Hz control task) is the
 * frame clock: if it ever stalls, frames stop and the ESC's own signal-loss failsafe
 * stops the motors (dead-man behaviour). No looping transmissions are used for that reason.
 *
 * DShot frame: 11-bit value (0 = stop, 1-47 = commands, 48-2047 = throttle),
 * 1 telemetry bit, 4-bit CRC; 16 bits MSB first. DShot300: 3.333 us per bit,
 * '1' = 75 % high, '0' = 37.5 % high.
 */
#pragma once

#include "driver/rmt_tx.h"
#include <cstdint>

namespace drivers {

class DShot {
public:
    static constexpr int N = 4;

    /// Configure 4 RMT TX channels. Nothing is transmitted until write() is called,
    /// so an ESC powered on the bench stays disarmed (no signal).
    esp_err_t init(const int gpio[N]) noexcept;

    /// Send one frame per motor. value: 0 = stop, 48..2047 = throttle.
    void write(const uint16_t value[N]) noexcept;

    /// Normalised throttle (0..1) -> DShot value (48..2047); <= 0 -> 0 (stop)
    static uint16_t from_throttle(float t) noexcept;

    bool ready() const noexcept { return ready_; }

private:
    rmt_channel_handle_t  ch_[N]  = {};
    rmt_encoder_handle_t  enc_    = nullptr;
    rmt_symbol_word_t     buf_[N][2][16] = {};   ///< Double-buffered so a queued frame is never overwritten
    uint8_t               flip_   = 0;
    bool                  ready_  = false;
};

} // namespace drivers
