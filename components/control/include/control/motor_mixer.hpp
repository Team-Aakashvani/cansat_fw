/**
 * @file motor_mixer.hpp
 * @brief Quadrotor cross (+) motor mixer + PWM output via LEDC.
 *
 * Motor layout (viewed from above, CW/CCW convention):
 *   Motor 0 (Front-Left)  : CW  (GPIO PINS.motor[0])
 *   Motor 1 (Front-Right) : CCW (GPIO PINS.motor[1])
 *   Motor 2 (Rear-Right)  : CW  (GPIO PINS.motor[2])
 *   Motor 3 (Rear-Left)   : CCW (GPIO PINS.motor[3])
 *
 * Mixing matrix (+config):
 *   M0 = throttle + pitch + roll + yaw
 *   M1 = throttle + pitch - roll - yaw
 *   M2 = throttle - pitch - roll + yaw
 *   M3 = throttle - pitch + roll - yaw
 *
 * Outputs are ESC PWM signals in microseconds [1000, 2000].
 * LEDC at 50Hz (ESC standard). 16-bit resolution.
 *
 * Also controls the release servo (LEDC channel 4).
 */
#pragma once
#include "nav/config.hpp"
#include "driver/ledc.h"
#include <cstdint>

namespace control {

class MotorMixer {
public:
    static constexpr int N_MOTORS = 4;
    static constexpr uint32_t LEDC_FREQ_HZ  = 50;
    static constexpr uint32_t LEDC_RES_BITS = 16;
    static constexpr uint32_t LEDC_MAX_DUTY = (1 << LEDC_RES_BITS) - 1;
    static constexpr uint32_t SERVO_CH      = 4;  // LEDC channel for servo

    MotorMixer() noexcept = default;

    /// Initialise LEDC channels for all motors and servo.
    esp_err_t init() noexcept;

    /// Arm motors (send 1000µs for 2 seconds to ESC).
    void arm() noexcept;

    /// Disarm: set all motors to idle.
    void disarm() noexcept;

    /// Set individual motor PWM in microseconds [1000, 2000].
    void set_motor_us(int motor_idx, uint32_t us) noexcept;

    /// Mix and set all motors from command inputs.
    /// throttle ∈ [0,1], pitch/roll/yaw ∈ [-1,1]
    void mix_and_set(double throttle, double pitch, double roll, double yaw,
                     double battery_factor = 1.0) noexcept;

    /// Release servo: deploy parachute/mechanism.
    void servo_release() noexcept;

    /// Return servo to home position.
    void servo_home() noexcept;

    /**
     * @brief Set the servo to a specific angle (0 to 180 degrees).
     * Used for directional antenna sweeping.
     */
    void set_servo_angle(double degrees) noexcept;

    bool is_armed() const noexcept { return armed_; }

private:
    bool     armed_    = false;
    uint32_t motor_us_[N_MOTORS] = {};

    uint32_t us_to_duty(uint32_t us) const noexcept;
    void     apply_motor(int idx) noexcept;
};

} // namespace control
