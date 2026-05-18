/**
 * @file adaptive.hpp
 * @brief Online adaptive measurement-noise covariance (AdaptiveR).
 *
 * Faithful C++ port of adaptive.py (Mehra 1972 / Mohamed & Schwarz 1999).
 *
 * EMA estimate of innovation covariance:
 *   Ĉ_ν,k = (1−α)·Ĉ_ν,k−1 + α·ν_k·ν_kᵀ
 *   R_adapted = clamp(Ĉ_ν − H·P·Hᵀ, R_min, R_max)
 *
 * Per-sensor state is a (MAX_MEAS_DIM × MAX_MEAS_DIM) covariance matrix.
 * No heap allocation.
 */
#pragma once

#include "config.hpp"
#include <cmath>
#include <cstring>
#include <algorithm>

namespace nav {

struct AdaptState {
    double R0[MAX_MEAS_DIM][MAX_MEAS_DIM];
    double C[MAX_MEAS_DIM][MAX_MEAS_DIM];   ///< Running EMA of innovation cov
    int    meas_dim = 1;
    int    n = 0;

    void init(const double r0_diag[], int M) noexcept {
        meas_dim = M;
        n = 0;
        for (int i = 0; i < MAX_MEAS_DIM; ++i)
            for (int j = 0; j < MAX_MEAS_DIM; ++j) {
                R0[i][j] = (i == j && i < M) ? r0_diag[i] : 0.0;
                C[i][j]  = R0[i][j];
            }
    }
};

class AdaptiveR {
public:
    AdaptState state[N_SENSORS];

    AdaptiveR() noexcept {
        constexpr double baro_diag[1] = { BARO_CFG.sigma_h_floor_m * BARO_CFG.sigma_h_floor_m };
        state[SENSOR_BARO].init(baro_diag, 1);

        constexpr double gnss_diag[6] = {
            GNSS_CFG.horizontal_pos_std_m * GNSS_CFG.horizontal_pos_std_m,
            GNSS_CFG.horizontal_pos_std_m * GNSS_CFG.horizontal_pos_std_m,
            GNSS_CFG.vertical_pos_std_m   * GNSS_CFG.vertical_pos_std_m,
            GNSS_CFG.horizontal_vel_std_mps * GNSS_CFG.horizontal_vel_std_mps,
            GNSS_CFG.horizontal_vel_std_mps * GNSS_CFG.horizontal_vel_std_mps,
            GNSS_CFG.horizontal_vel_std_mps * GNSS_CFG.horizontal_vel_std_mps,
        };
        state[SENSOR_GNSS].init(gnss_diag, 6);

        constexpr double mag_diag[3] = { 1.0, 1.0, 1.0 };
        state[SENSOR_MAG].init(mag_diag, 3);

        // IMU is propagation-only (not updated via AdaptiveR)
        state[SENSOR_IMU].init(nullptr, 6);
    }

    /// Observe one accepted innovation vector — update EMA.
    void observe(int sensor_id, const double innov[], int M) noexcept {
        if (sensor_id < 0 || sensor_id >= N_SENSORS) return;
        AdaptState& st = state[sensor_id];
        const double alpha = ESTIMATOR_CFG.adaptive_alpha;
        // Ĉ ← (1−α)·Ĉ + α·ν·νᵀ
        for (int i = 0; i < M; ++i)
            for (int j = 0; j < M; ++j)
                st.C[i][j] = (1.0-alpha)*st.C[i][j] + alpha*innov[i]*innov[j];
        // Force symmetry
        for (int i = 0; i < M; ++i)
            for (int j = i+1; j < M; ++j) {
                double avg = 0.5*(st.C[i][j]+st.C[j][i]);
                st.C[i][j] = st.C[j][i] = avg;
            }
        st.n++;
    }

    /// Get current clamped adaptive R diagonal for sensor_id.
    /// Fills r_out[M] with the clamped diagonal variances.
    void current_diag(int sensor_id, double r_out[], int M) const noexcept {
        if (sensor_id < 0 || sensor_id >= N_SENSORS) {
            for (int i = 0; i < M; ++i) r_out[i] = 1.0;
            return;
        }
        const AdaptState& st = state[sensor_id];
        const double Rmin = ESTIMATOR_CFG.adaptive_R_min_factor;
        const double Rmax = ESTIMATOR_CFG.adaptive_R_max_factor;
        for (int i = 0; i < M; ++i) {
            const double lo = st.R0[i][i] * Rmin;
            const double hi = st.R0[i][i] * Rmax;
            double val = st.C[i][i];
            if (val < lo) val = lo;
            if (val > hi) val = hi;
            r_out[i] = val;
        }
    }
};

} // namespace nav
