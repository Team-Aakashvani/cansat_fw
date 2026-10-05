#pragma once
#include <cmath>
#include <algorithm>

namespace control {

/**
 * @brief Simple PID controller class.
 * Decoupled from any hardware or RTOS.
 */
struct PIDGains {
    float kp;
    float ki;
    float kd;
    float i_limit;
};

class PID {
public:
    PID() = default;

    void set_gains(const PIDGains& gains) {
        gains_ = gains;
    }

    void reset() {
        integral_ = 0.0f;
        prev_error_ = 0.0f;
        first_run_ = true;
    }

    float update(float setpoint, float measurement, float dt) {
        if (dt <= 0.0f || !std::isfinite(setpoint) || !std::isfinite(measurement)) {
            return 0.0f;
        }

        float error = setpoint - measurement;
        if (!std::isfinite(error)) return 0.0f;
        
        // Integral with anti-windup limit
        integral_ += error * dt;
        const float limit = (gains_.i_limit > 0.0f) ? gains_.i_limit : 0.5f;
        integral_ = std::clamp(integral_, -limit, limit);

        // Derivative (with first-run spike suppression)
        float derivative = 0.0f;
        if (!first_run_ && std::isfinite(prev_error_)) {
            derivative = (error - prev_error_) / dt;
            if (!std::isfinite(derivative)) derivative = 0.0f;
        }
        first_run_ = false;
        prev_error_ = error;

        float output = (gains_.kp * error) + (gains_.ki * integral_) + (gains_.kd * derivative);
        if (!std::isfinite(output)) {
            reset();
            return 0.0f;
        }
        return output;
    }

private:
    PIDGains gains_{0.0f, 0.0f, 0.0f, 0.0f};
    float integral_   = 0.0f;
    float prev_error_ = 0.0f;
    bool  first_run_  = true;
};

/**
 * @brief Cascaded PID controller for attitude and rate control.
 */
class CascadedPID {
public:
    struct Vector3 { float x, y, z; };

    CascadedPID() = default;

    void set_angle_gains(const PIDGains& roll, const PIDGains& pitch) {
        roll_angle_.set_gains(roll);
        pitch_angle_.set_gains(pitch);
    }

    void set_rate_gains(const PIDGains& roll, const PIDGains& pitch, const PIDGains& yaw) {
        roll_rate_.set_gains(roll);
        pitch_rate_.set_gains(pitch);
        yaw_rate_.set_gains(yaw);
    }

    void reset() {
        roll_angle_.reset();
        pitch_angle_.reset();
        roll_rate_.reset();
        pitch_rate_.reset();
        yaw_rate_.reset();
    }

    /**
     * @brief Update the cascaded PID controller.
     * @param target_euler Target attitude (roll, pitch in rad, yaw is target rate in rad/s)
     * @param current_euler Current attitude (roll, pitch, yaw in rad)
     * @param current_rates Current angular rates (rad/s)
     * @param dt Timestep in seconds
     * @return Torque corrections for roll, pitch, and yaw axes.
     */
    Vector3 update(const Vector3& target_euler, const Vector3& current_euler, 
                   const Vector3& current_rates, float dt) {
        if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;

        // Comprehensive guard against any non-finite orientation or gyro inputs
        if (!std::isfinite(current_euler.x) || !std::isfinite(current_euler.y) || !std::isfinite(current_euler.z) ||
            !std::isfinite(current_rates.x) || !std::isfinite(current_rates.y) || !std::isfinite(current_rates.z)) {
            reset();
            return {0.0f, 0.0f, 0.0f};
        }

        // Wrap angle error to [-pi, +pi] and clamp to +/- 30 degrees (0.52 rad)
        auto wrap_clamp_error = [](float sp, float meas) -> float {
            if (!std::isfinite(sp) || !std::isfinite(meas)) return 0.0f;
            float err = sp - meas;
            while (err > 3.14159265f)  err -= 6.2831853f;
            while (err < -3.14159265f) err += 6.2831853f;
            return std::clamp(err, -0.5235f, 0.5235f);
        };

        float err_roll  = wrap_clamp_error(target_euler.x, current_euler.x);
        float err_pitch = wrap_clamp_error(target_euler.y, current_euler.y);

        // Outer loop: Angle error -> Rate Setpoint (clamped to +/- 2.0 rad/s)
        float roll_rate_sp  = std::clamp(roll_angle_.update(err_roll, 0.0f, dt), -2.0f, 2.0f);
        float pitch_rate_sp = std::clamp(pitch_angle_.update(err_pitch, 0.0f, dt), -2.0f, 2.0f);
        
        // Inner loop: Rate Error -> Differential Torque (clamped to +/- 0.15)
        Vector3 torque;
        torque.x = std::clamp(roll_rate_.update(roll_rate_sp, current_rates.x, dt), -0.15f, 0.15f);
        torque.y = std::clamp(pitch_rate_.update(pitch_rate_sp, current_rates.y, dt), -0.15f, 0.15f);
        torque.z = std::clamp(yaw_rate_.update(target_euler.z, current_rates.z, dt), -0.05f, 0.05f);
        
        if (!std::isfinite(torque.x)) torque.x = 0.0f;
        if (!std::isfinite(torque.y)) torque.y = 0.0f;
        if (!std::isfinite(torque.z)) torque.z = 0.0f;
        return torque;
    }

private:
    PID roll_angle_, pitch_angle_;
    PID roll_rate_, pitch_rate_, yaw_rate_;
};

} // namespace control
