#pragma once
#include <algorithm>

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
        // Standard Quad-X mixing:
        // FL = T + R + P + Y
        // FR = T - R + P - Y
        // RR = T - R - P + Y
        // RL = T + R - P - Y
        
        float fl = throttle + roll + pitch + yaw;
        float fr = throttle - roll + pitch - yaw;
        float rr = throttle - roll - pitch + yaw;
        float rl = throttle + roll - pitch - yaw;

        MotorOutputs out;
        out.m1 = scale_to_pwm(fl);
        out.m2 = scale_to_pwm(fr);
        out.m3 = scale_to_pwm(rr);
        out.m4 = scale_to_pwm(rl);
        
        return out;
    }

private:
    float scale_to_pwm(float value) {
        // Map [0, 1] (or beyond) to [1000, 2000]
        float pwm = 1000.0f + value * 1000.0f;
        return std::max(1000.0f, std::min(2000.0f, pwm));
    }
};

} // namespace control
