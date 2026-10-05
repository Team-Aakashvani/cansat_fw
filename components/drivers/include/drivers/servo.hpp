/**
 * @file servo.hpp
 * @brief Two 50 Hz PWM servo outputs (drone-arm latches) on LEDC.
 *
 * ESP32-S3 LEDC timers are 14-bit maximum (the old MotorMixer asked for 16-bit,
 * failed, and silently left every motor and servo channel unconfigured).
 * Uses LEDC timer 2 / channels 6-7 so it cannot collide with any legacy code.
 */
#pragma once

#include "esp_err.h"
#include <cstdint>

namespace drivers {

class ServoPair {
public:
    /// Configure both servos and immediately drive them to initial_us.
    esp_err_t init(int gpio_a, int gpio_b, uint32_t initial_us) noexcept;

    /// Set both servos to the same pulse width (500..2500 us).
    void set_us(uint32_t us) noexcept;

    uint32_t current_us() const noexcept { return us_; }
    bool ready() const noexcept { return ready_; }

private:
    bool     ready_ = false;
    uint32_t us_    = 0;
};

} // namespace drivers
