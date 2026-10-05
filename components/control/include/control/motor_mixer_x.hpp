#pragma once
#include <algorithm>
#include <cmath>

namespace control {

/**
 * @brief Standalone Motor Mixer for Quadcopter 'X' configuration.
 * Decoupled from hardware/RTOS.
 */
class MotorMixerX {
public:
    struct MotorOutputs {
        float m1; // Front-Left
        float m2; // Front-Right
        float m3; // Rear-Right
        float m4; // Rear-Left
    };

    MotorMixerX() = default;

    /**
     * @brief Mix torque corrections and throttle to motor outputs.
     * @param throttle Collective throttle [0, 1]
     * @param roll Torque correction for roll [-1, 1]
     * @param pitch Torque correction for pitch [-1, 1]
     * @param yaw Torque correction for yaw [-1, 1]
     * @return Motor outputs in microseconds [1000, 2000].
     */
    MotorOutputs mix(float throttle, float roll, float pitch, float yaw) {
        if (!std::isfinite(throttle)) throttle = 0.10f;
        if (!std::isfinite(roll))     roll = 0.0f;
        if (!std::isfinite(pitch))    pitch = 0.0f;
        if (!std::isfinite(yaw))      yaw = 0.0f;

        // Clamp throttle safely to [0.05, 0.40] (max 40% for bench stability test)
        throttle = std::clamp(throttle, 0.05f, 0.40f);

        // Standard Quad-X mixing with differential torques
        float fl = throttle + roll + pitch + yaw;
        float fr = throttle - roll + pitch - yaw;
        float rr = throttle - roll - pitch + yaw;
        float rl = throttle + roll - pitch - yaw;

        MotorOutputs out;
        out.m1 = apply_slew(0, scale_to_pwm(fl));
        out.m2 = apply_slew(1, scale_to_pwm(fr));
        out.m3 = apply_slew(2, scale_to_pwm(rr));
        out.m4 = apply_slew(3, scale_to_pwm(rl));
        
        return out;
    }

    void reset_slew() {
        for (int i = 0; i < 4; ++i) prev_pwm_[i] = 1000.0f;
    }

private:
    float prev_pwm_[4] = {1000.0f, 1000.0f, 1000.0f, 1000.0f};

    float scale_to_pwm(float value) {
        if (!std::isfinite(value)) return 1000.0f;
        // Map [0, 1] to safe [1000, 1450] us (max 45% throttle to prevent brownouts)
        float pwm = 1000.0f + value * 1000.0f;
        return std::clamp(pwm, 1000.0f, 1450.0f);
    }

    float apply_slew(int idx, float target_pwm) {
        if (idx < 0 || idx >= 4) return 1000.0f;
        if (!std::isfinite(target_pwm)) target_pwm = 1000.0f;
        if (!std::isfinite(prev_pwm_[idx])) prev_pwm_[idx] = 1000.0f;

        const float max_step = 25.0f; // max 25us delta per 10ms cycle (smooth soft-start ramp)
        float diff = target_pwm - prev_pwm_[idx];
        if (diff > max_step) target_pwm = prev_pwm_[idx] + max_step;
        else if (diff < -max_step) target_pwm = prev_pwm_[idx] - max_step;
        prev_pwm_[idx] = target_pwm;
        return target_pwm;
    }
};

} // namespace control
