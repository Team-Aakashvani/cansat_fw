/**
 * @file fdir.hpp
 * @brief Fault Detection, Isolation, and Recovery (FDIR) monitor.
 *
 * Faithful C++ port of fdir.py.
 *
 * Multi-layer FDIR stack:
 *   Layer 1 — Per-channel innovation z-test:  |ν_i|/√S_ii > z_α (99%)
 *   Layer 2 — SPRT persistence: accumulates log-likelihood ratio over a window;
 *             fires when > log_threshold.
 *   Layer 3 — Analytical redundancy: cross-checks altitude rate across baro/GNSS/INS.
 *   Layer 4 — Recovery: quarantined sensors get R inflated to ceiling; health
 *             monitor restores trust when samples pass all layers again.
 *
 * All state is fixed-size, no dynamic allocation.
 */
#pragma once

#include "config.hpp"
#include <cmath>
#include <cstring>

namespace nav {

// ---------------------------------------------------------------------------
// Z-score thresholds
// ---------------------------------------------------------------------------
constexpr double Z_99  = 2.5758;   ///< P(|z| > 2.576) = 0.01  (99% confidence)
constexpr double Z_999 = 3.2905;   ///< P(|z| > 3.290) = 0.001 (99.9%)

// ---------------------------------------------------------------------------
// Per-channel FDIR state
// ---------------------------------------------------------------------------
struct ChannelStat {
    double sprt_logL    = 0.0;
    int    consec_bad   = 0;
    double last_z       = 0.0;
};

// ---------------------------------------------------------------------------
// FDIRReport — per-cycle summary
// ---------------------------------------------------------------------------
struct FDIRReport {
    int    sensor_id;
    double t_s;
    bool   per_channel_flags[MAX_MEAS_DIM];
    bool   persistent_fault;
    double sprt_logL;
    bool   quarantined;
};

// ---------------------------------------------------------------------------
// FDIRMonitor — one instance per sensor
// ---------------------------------------------------------------------------
struct SensorFDIR {
    ChannelStat ch[MAX_MEAS_DIM];
    bool        quarantined = false;
    int         meas_dim    = 1;

    void reset() noexcept {
        for (int i = 0; i < MAX_MEAS_DIM; ++i) ch[i] = ChannelStat{};
        quarantined = false;
    }
};

class FDIRMonitor {
public:
    static constexpr double Z_ALPHA     = Z_99;
    static constexpr double SPRT_THR    = 4.6;   ///< ln(100) ≈ 4.6
    static constexpr int    SPRT_WIN    = 10;
    static constexpr int    PERSIST_REQ = 5;

    SensorFDIR sensors[N_SENSORS];

    FDIRMonitor() noexcept {
        for (int i = 0; i < N_SENSORS; ++i) sensors[i].reset();
    }

    bool is_quarantined(int sensor_id) const noexcept {
        if (sensor_id < 0 || sensor_id >= N_SENSORS) return false;
        return sensors[sensor_id].quarantined;
    }

    void force_release(int sensor_id) noexcept {
        if (sensor_id >= 0 && sensor_id < N_SENSORS)
            sensors[sensor_id].quarantined = false;
    }

    // -----------------------------------------------------------------------
    // evaluate: run all layers on one measurement update's diagnostics.
    // normalised_residuals[m]: ν_i / √S_ii
    // Returns FDIRReport.
    // -----------------------------------------------------------------------
    FDIRReport evaluate(int sensor_id, double t_s,
                        const double norm_res[], int M) noexcept {
        FDIRReport rep{};
        rep.sensor_id = sensor_id;
        rep.t_s = t_s;
        if (sensor_id < 0 || sensor_id >= N_SENSORS) return rep;

        SensorFDIR& sf = sensors[sensor_id];
        sf.meas_dim = M;

        // Layer 1: per-channel z-test
        bool any_flag = false;
        for (int i = 0; i < M; ++i) {
            const double z = norm_res[i];
            sf.ch[i].last_z = z;
            bool flagged = std::abs(z) > Z_ALPHA;
            rep.per_channel_flags[i] = flagged;
            if (flagged) { sf.ch[i].consec_bad++; any_flag = true; }
            else         { if (sf.ch[i].consec_bad > 0) sf.ch[i].consec_bad--; }
        }

        // Layer 2: SPRT
        double log_lr = 0.0;
        for (int i = 0; i < M; ++i) {
            const double z_abs = std::abs(sf.ch[i].last_z);
            if (z_abs > Z_ALPHA)
                log_lr += 0.5 * (z_abs*z_abs - Z_ALPHA*Z_ALPHA);
            else
                log_lr -= 0.25 * (Z_ALPHA*Z_ALPHA - z_abs*z_abs);
        }
        const double decay = 1.0 - 1.0 / SPRT_WIN;
        sf.ch[0].sprt_logL = decay * sf.ch[0].sprt_logL + log_lr;
        rep.sprt_logL = sf.ch[0].sprt_logL;
        rep.persistent_fault = (sf.ch[0].sprt_logL > SPRT_THR);

        // Layer 4: quarantine decision
        int consec_max = 0;
        for (int i = 0; i < M; ++i)
            if (sf.ch[i].consec_bad > consec_max)
                consec_max = sf.ch[i].consec_bad;

        bool quarantine = rep.persistent_fault || (consec_max >= PERSIST_REQ);

        // Recovery: exit quarantine only if all channels clean AND SPRT decayed
        if (sf.quarantined && consec_max == 0
                && sf.ch[0].sprt_logL < 0.5 * SPRT_THR)
            quarantine = false;

        sf.quarantined = quarantine;
        rep.quarantined = quarantine;
        return rep;
    }

    // -----------------------------------------------------------------------
    // Analytical redundancy: cross-check vertical-velocity bearing signals.
    // Returns: suspect[SENSOR_BARO], suspect[SENSOR_GNSS], suspect[SENSOR_IMU]
    // -----------------------------------------------------------------------
    void analytical_redundancy(double baro_dot_mps,
                                double ins_vel_z_mps,
                                double gnss_vel_z_mps,
                                bool   gnss_valid,
                                double tol_mps,
                                bool   suspect_out[3]) noexcept {
        double vals[3] = { baro_dot_mps, ins_vel_z_mps, gnss_vel_z_mps };
        int n = gnss_valid ? 3 : 2;

        // Median of available signals
        double sorted[3];
        for (int i = 0; i < n; ++i) sorted[i] = vals[i];
        // Simple insertion sort for n≤3
        for (int i = 1; i < n; ++i) {
            double key = sorted[i]; int j = i-1;
            while (j >= 0 && sorted[j] > key) { sorted[j+1] = sorted[j]; j--; }
            sorted[j+1] = key;
        }
        const double median = sorted[n / 2];

        for (int i = 0; i < 3; ++i)
            suspect_out[i] = std::abs(vals[i] - median) > tol_mps;
    }
};

} // namespace nav
