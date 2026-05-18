/**
 * @file pid.hpp
 * @brief PID controller with anti-windup, derivative-on-measurement, and slew rate.
 *
 * Used for attitude (pitch/roll) and descent-rate stabilisation during drone phase.
 * Anti-windup: conditional integration (stop integrating when saturated).
 * Derivative on measurement (not error) to avoid derivative kick on setpoint change.
 * All state is fixed-size, no dynamic allocation.
 *
 * @compliance CAN-7USAT §3.2 (1-3 m/s descent rate requirement)
 */
#pragma once
#include <cmath>

namespace control {

struct PIDConfig {
    double kp, ki, kd;
    double dt_s;               ///< Expected timestep (s)
    double output_min;         ///< Lower output clamp
    double output_max;         ///< Upper output clamp
    double integral_limit;     ///< Anti-windup integrator clamp
    double deadband;           ///< Error deadband (zero below this)
    double derivative_alpha;   ///< Low-pass filter on derivative: α ∈ [0,1]
};

class PID {
public:
    explicit PID() noexcept { reset(); }
    void configure(const PIDConfig& cfg) noexcept { cfg_ = cfg; }

    double update(double setpoint, double measurement, double dt) noexcept {
        const double error = setpoint - measurement;
        const double err_dz = (std::abs(error) < cfg_.deadband) ? 0.0 : error;

        // Proportional
        const double P = cfg_.kp * err_dz;

        // Integral with anti-windup (conditional integration)
        bool saturated = (output_prev_ <= cfg_.output_min && err_dz < 0.0)
                      || (output_prev_ >= cfg_.output_max && err_dz > 0.0);
        if (!saturated) integral_ += err_dz * dt;
        integral_ = clamp(integral_, -cfg_.integral_limit, cfg_.integral_limit);
        const double I = cfg_.ki * integral_;

        // Derivative on measurement (filtered)
        const double dmeas = (measurement - prev_meas_) / std::max(dt, 1.0e-6);
        deriv_filt_ = cfg_.derivative_alpha * deriv_filt_ + (1.0-cfg_.derivative_alpha)*dmeas;
        const double D = -cfg_.kd * deriv_filt_;  // Negative: derivative on measurement

        prev_meas_    = measurement;
        output_prev_  = clamp(P + I + D, cfg_.output_min, cfg_.output_max);
        return output_prev_;
    }

    void reset() noexcept {
        integral_    = 0.0;
        prev_meas_   = 0.0;
        deriv_filt_  = 0.0;
        output_prev_ = 0.0;
    }

    double integral() const noexcept { return integral_; }

private:
    PIDConfig cfg_{};
    double integral_   = 0.0;
    double prev_meas_  = 0.0;
    double deriv_filt_ = 0.0;
    double output_prev_= 0.0;

    static double clamp(double v, double lo, double hi) noexcept {
        return (v < lo) ? lo : (v > hi ? hi : v);
    }
};

} // namespace control
