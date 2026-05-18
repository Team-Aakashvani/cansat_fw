/**
 * @file health.hpp
 * @brief Per-sensor health monitor H_i ∈ [H_floor, 1].
 *
 * Faithful C++ port of health.py.
 *
 * Health is computed from five sub-scores (pessimistic min, then EMA):
 *   1. NIS consistency: exp(−(NIS−dof)² / 2·dof)
 *   2. Stuck-value: exp(−consec_stuck / 10)
 *   3. Driver fault: exp(−fault_counter / 5)
 *   4. Dropout: exp(−(gap/T_expected − 3))  when gap > 3·T
 *   5. Composite: min(all) → EMA with τ = health_smoothing_tau_s
 *
 * R_eff = R₀ / max(H, H_floor) — as used in the EKF update.
 * H_floor prevents complete trust collapse (graceful degradation).
 *
 * All state is pre-allocated (ring buffer as static array).
 * No dynamic allocation.
 */
#pragma once

#include "config.hpp"
#include <cmath>
#include <cstring>

namespace nav {

struct SensorHealthState {
    // Ring buffer for NIS windowed average
    static constexpr int WINDOW = 50;
    double nis_buf[WINDOW];
    double z_last[MAX_MEAS_DIM];
    double t_last;
    int    nis_idx;
    int    n_filled;
    int    stuck_counter;
    int    fault_counter;
    double last_health;
    double expected_period_s;

    void reset(double expected_rate_hz) noexcept {
        for (int i = 0; i < WINDOW; ++i) nis_buf[i] = 0.0;
        for (int i = 0; i < MAX_MEAS_DIM; ++i) z_last[i] = 0.0;
        t_last          = -1.0;
        nis_idx         = 0;
        n_filled        = 0;
        stuck_counter   = 0;
        fault_counter   = 0;
        last_health     = 1.0;
        expected_period_s = (expected_rate_hz > 0.0) ? (1.0 / expected_rate_hz) : 1.0;
    }
};

class SensorHealthMonitor {
public:
    SensorHealthState state[N_SENSORS];

    SensorHealthMonitor() noexcept {
        state[SENSOR_IMU].reset(IMU_CFG.rate_hz);
        state[SENSOR_BARO].reset(BARO_CFG.rate_hz);
        state[SENSOR_GNSS].reset(GNSS_CFG.rate_hz);
        state[SENSOR_MAG].reset(25.0);   // BNO085 mag at 25Hz
    }

    double health(int sensor_id) const noexcept {
        if (sensor_id < 0 || sensor_id >= N_SENSORS) return 1.0;
        return state[sensor_id].last_health;
    }

    /// Update health from one measurement's diagnostics.
    /// @param mahal_sq  Mahalanobis² from EKF update
    /// @param z_meas    Raw measurement vector
    /// @param meas_dim  Length of z_meas
    /// @param t_s       Current timestamp (s)
    /// @param driver_fault  True if hardware driver reported anomaly
    double update(int sensor_id, double mahal_sq,
                  const double z_meas[], int meas_dim,
                  double t_s, bool driver_fault) noexcept {
        if (sensor_id < 0 || sensor_id >= N_SENSORS) return 1.0;
        SensorHealthState& hs = state[sensor_id];

        // 1. NIS sub-score
        hs.nis_buf[hs.nis_idx] = mahal_sq;
        hs.nis_idx = (hs.nis_idx + 1) % SensorHealthState::WINDOW;
        if (hs.n_filled < SensorHealthState::WINDOW) hs.n_filled++;
        double mean_nis = 0.0;
        for (int i = 0; i < hs.n_filled; ++i) mean_nis += hs.nis_buf[i];
        mean_nis /= hs.n_filled;
        const double dof = (double)meas_dim;
        const double var_scale = 2.0 * dof;
        const double s_nis = std::exp(-((mean_nis - dof) * (mean_nis - dof)) / (2.0 * var_scale));

        // 2. Stuck-value detection
        bool identical = true;
        for (int i = 0; i < meas_dim; ++i)
            if (std::abs(z_meas[i] - hs.z_last[i]) > 1.0e-9) { identical = false; break; }
        if (identical) hs.stuck_counter++;
        else           hs.stuck_counter = 0;
        for (int i = 0; i < meas_dim && i < MAX_MEAS_DIM; ++i) hs.z_last[i] = z_meas[i];
        const double s_stuck = std::exp(-(double)hs.stuck_counter / 10.0);

        // 3. Driver fault
        if (driver_fault) hs.fault_counter = std::min(hs.fault_counter + 1, 20);
        else              hs.fault_counter = std::max(hs.fault_counter - 1, 0);
        const double s_fault = std::exp(-(double)hs.fault_counter / 5.0);

        // 4. Dropout detection
        double s_drop = 1.0;
        if (hs.t_last > 0.0) {
            const double gap = t_s - hs.t_last;
            if (gap > 3.0 * hs.expected_period_s)
                s_drop = std::exp(-(gap / hs.expected_period_s - 3.0));
        }
        hs.t_last = t_s;

        // Composite (pessimistic min)
        double raw = s_nis;
        if (s_stuck < raw) raw = s_stuck;
        if (s_fault < raw) raw = s_fault;
        if (s_drop  < raw) raw = s_drop;

        // EMA
        const double T_exp = hs.expected_period_s;
        const double tau   = ESTIMATOR_CFG.health_smoothing_tau_s;
        double alpha = T_exp / (tau + T_exp);
        if (alpha < 0.01) alpha = 0.01;
        if (alpha > 1.0)  alpha = 1.0;
        double smoothed = (1.0 - alpha) * hs.last_health + alpha * raw;
        const double floor = ESTIMATOR_CFG.health_floor;
        if (smoothed < floor) smoothed = floor;
        if (smoothed > 1.0)  smoothed = 1.0;
        hs.last_health = smoothed;
        return smoothed;
    }

    void snapshot(double h_out[N_SENSORS]) const noexcept {
        for (int i = 0; i < N_SENSORS; ++i) h_out[i] = state[i].last_health;
    }
};

} // namespace nav
