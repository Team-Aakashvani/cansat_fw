/**
 * @file flight_computer.hpp
 * @brief Top-level integrated avionics — FlightComputer.
 *
 * C++ port of flight_computer.py.
 *
 * Data flow (matches Python flight_computer.py exactly):
 *   IMU  → IMMFilter.predict()  (high-rate, ~100Hz)
 *   Baro/GNSS/Mag → EKF update per model → IMMFilter.update()
 *                → SensorHealthMonitor.update()
 *                → FDIRMonitor.evaluate()
 *                → AdaptiveR.observe()
 *                → BayesianSupervisor.step()
 *                → FlightComputerOutput
 *
 * Thread safety: Not re-entrant. The caller (RT task) must ensure
 * that ingest_imu() and ingest_aiding() are not called concurrently.
 * Use a FreeRTOS mutex if shared across tasks.
 */
#pragma once

#include "imm.hpp"
#include "fdir.hpp"
#include "health.hpp"
#include "adaptive.hpp"
#include "supervisor.hpp"
#include "measurements.hpp"

namespace nav {

// ---------------------------------------------------------------------------
// FlightComputerOutput — one telemetry tick
// ---------------------------------------------------------------------------
struct FlightComputerOutput {
    double         t_s;
    IMMOutput      imm;
    SupervisorOutput sup;
    double         health[N_SENSORS];
    bool           fdir_quarantine[N_SENSORS];
    bool           fault_alarm;
    double         baro_alt_m;
    double         gnss_alt_m;
    double         voltage_v;
    double         current_a;
    double         accel_mag_mps2;
    double         gyro_mag_radps;
    double         alt_est_m;      ///< Altitude used by mission logic (baro, or protected estimate if held)
    bool           baro_held;      ///< Baro currently rejected as statistically inconsistent
    bool           gnss_held;      ///< GNSS currently rejected as statistically inconsistent
};

// ---------------------------------------------------------------------------
// FlightComputer
// ---------------------------------------------------------------------------
class FlightComputer {
public:
    IMMFilter           imm;
    SensorHealthMonitor health_mon;
    FDIRMonitor         fdir;
    AdaptiveR           adapt_r;
    BayesianSupervisor  supervisor;

    FlightComputerOutput last_output;
    bool                 output_valid = false;

    // -----------------------------------------------------------------------
    // Initialise from config (called once at boot after NVS is loaded)
    // -----------------------------------------------------------------------
    void init(const NavState& initial_nav) noexcept {
        imm.set_initial_nav(initial_nav);
        supervisor.reset();
        output_valid = false;
        last_imu_t   = -1.0;
        last_sup_t   = -1.0;
        fault_alarm  = false;
        baro_dot_buf_idx = 0;
        baro_dot_n       = 0;
        baro_last_alt    = 0.0;
        baro_last_t      = -1.0;
        for (int i = 0; i < N_SENSORS; ++i) gate_[i].reset();
    }

    // -----------------------------------------------------------------------
    // High-rate IMU ingestion (no output — purely propagates the IMM)
    // -----------------------------------------------------------------------
    void ingest_imu(double t_s,
                    double acc_x, double acc_y, double acc_z,
                    double gyr_x, double gyr_y, double gyr_z) noexcept {
        if (last_imu_t < 0.0) { last_imu_t = t_s; return; }
        const double dt = t_s - last_imu_t;
        last_imu_t = t_s;
        if (dt <= 0.0 || dt > 0.5) return;

        Vec<3> f_b, w_b;
        f_b(0) = acc_x; f_b(1) = acc_y; f_b(2) = acc_z;
        w_b(0) = gyr_x; w_b(1) = gyr_y; w_b(2) = gyr_z;

        // Feed specific force and timestamp to supervisor (dynamic launch detector)
        supervisor.set_imu_specific_force(f_b, t_s);

        // Get current VS-IMM gate
        double vs_gate[N_REGIMES][N_REGIMES];
        supervisor.build_vs_gate(vs_gate);

        // Propagate all IMM models
        imm.predict(f_b, w_b, dt, vs_gate);
    }

    // -----------------------------------------------------------------------
    // Barometer update (1-D altitude AGL)
    //
    // Residual-monitoring pipeline (no hard-coded failure cases):
    //   1. PROBE   ν = z − H·x̂⁻ and NIS = νᵀS⁻¹ν on the most likely model,
    //              without touching the state.
    //   2. MONITOR health + FDIR (z-test, SPRT persistence) always see ν, NIS.
    //   3. DECIDE  consistent              -> fuse into every IMM model
    //              inconsistent / quarantined -> reject: last valid state is kept,
    //                                         prediction coasts on the IMU
    //              rejected for a long time but noise nominal -> re-sync.
    // -----------------------------------------------------------------------
    const FlightComputerOutput& ingest_baro(double t_s, double alt_agl_m) noexcept {
        // Update baro rate estimate (for analytical redundancy)
        update_baro_dot(alt_agl_m, t_s);

        // Build R (adaptive if enabled)
        double R_baro_diag[1];
        if (ESTIMATOR_CFG.adaptive_enabled)
            adapt_r.current_diag(SENSOR_BARO, R_baro_diag, 1);
        else
            R_baro_diag[0] = BARO_CFG.sigma_h_floor_m * BARO_CFG.sigma_h_floor_m;

        const double H_pre = effective_health(SENSOR_BARO);
        const double z[1] = { alt_agl_m };
        double R[1][1] = {{ R_baro_diag[0] }};

        // 1. Probe (no commit)
        const int b = best_model_idx();
        auto pr = imm.models[b].measurement_update<1>(
            t_s, z, h_baro, H_baro, R, H_pre, -1.0, /*commit=*/false);

        // 2. Monitor
        health_mon.update(SENSOR_BARO, pr.mahalanobis_sq, z, 1, t_s, false);
        fdir.evaluate(SENSOR_BARO, t_s, pr.normalised_residual, 1);

        // 3. Decide
        const double r_norm[1] = { pr.innovation[0] / std::sqrt(std::max(R_baro_diag[0], 1e-9)) };
        const GateDecision d = gate_decide(SENSOR_BARO, t_s, pr.mahalanobis_sq,
                                           ESTIMATOR_CFG.baro_gate_chi2, r_norm, 1);
        if (d == GateDecision::RESYNC) {
            const double nu2 = pr.innovation[0] * pr.innovation[0];
            for (int i = 0; i < N_REGIMES; ++i) {
                imm.models[i].P(EIDX_P_0 + 2, EIDX_P_0 + 2) += 2.0 * nu2 + R_baro_diag[0];
                imm.models[i].P(EIDX_V_0 + 2, EIDX_V_0 + 2) += 4.0;
            }
        }
        if (d != GateDecision::REJECT) {
            ModelUpdateRecord records[N_REGIMES];
            for (int i = 0; i < N_REGIMES; ++i) {
                auto ur = imm.models[i].measurement_update<1>(
                    t_s, z, h_baro, H_baro, R, H_pre, -1.0);
                records[i].accepted       = ur.accepted;
                records[i].mahalanobis_sq = ur.mahalanobis_sq;
                records[i].meas_dim       = 1;
                records[i].innovation_cov[0][0] = ur.innovation_cov[0][0];
            }
            imm.update(records);
            double innov_arr[1] = { pr.innovation[0] };
            adapt_r.observe(SENSOR_BARO, innov_arr, 1);   // outliers never inflate R
        }
        return emit_output(t_s, alt_agl_m, 0.0);
    }

    // -----------------------------------------------------------------------
    // GNSS/NavIC update (6-D: position + velocity in ENU) — same pipeline
    // -----------------------------------------------------------------------
    const FlightComputerOutput& ingest_gnss(double t_s,
                                            double px, double py, double pz,
                                            double vx, double vy, double vz,
                                            bool gagan_active = false) noexcept {
        double R_gnss_diag[6];
        if (ESTIMATOR_CFG.adaptive_enabled)
            adapt_r.current_diag(SENSOR_GNSS, R_gnss_diag, 6);
        else {
            // Tighten measurement covariance when GAGAN DGPS corrections are active
            const double h_std = gagan_active ? 1.0 : GNSS_CFG.horizontal_pos_std_m;
            const double v_std = gagan_active ? 2.0 : GNSS_CFG.vertical_pos_std_m;
            R_gnss_diag[0] = R_gnss_diag[1] = h_std * h_std;
            R_gnss_diag[2] = v_std * v_std;
            R_gnss_diag[3] = R_gnss_diag[4] = R_gnss_diag[5] =
                GNSS_CFG.horizontal_vel_std_mps * GNSS_CFG.horizontal_vel_std_mps;
        }

        const double H_pre = effective_health(SENSOR_GNSS);
        const double z[6] = { px, py, pz, vx, vy, vz };
        double R[6][6] = {};
        for (int i = 0; i < 6; ++i) R[i][i] = R_gnss_diag[i];

        // 1. Probe
        const int b = best_model_idx();
        auto pr = imm.models[b].measurement_update<6>(
            t_s, z, h_gnss, H_gnss, R, H_pre, -1.0, /*commit=*/false);

        // 2. Monitor
        health_mon.update(SENSOR_GNSS, pr.mahalanobis_sq, z, 6, t_s, false);
        fdir.evaluate(SENSOR_GNSS, t_s, pr.normalised_residual, 6);

        // 3. Decide
        double r_norm[6];
        for (int k = 0; k < 6; ++k)
            r_norm[k] = pr.innovation[k] / std::sqrt(std::max(R_gnss_diag[k], 1e-9));
        const GateDecision d = gate_decide(SENSOR_GNSS, t_s, pr.mahalanobis_sq,
                                           ESTIMATOR_CFG.gnss_gate_chi2, r_norm, 6);
        if (d == GateDecision::RESYNC) {
            for (int i = 0; i < N_REGIMES; ++i) {
                for (int k = 0; k < 3; ++k) {
                    imm.models[i].P(EIDX_P_0 + k, EIDX_P_0 + k) +=
                        2.0 * pr.innovation[k] * pr.innovation[k] + R_gnss_diag[k];
                    imm.models[i].P(EIDX_V_0 + k, EIDX_V_0 + k) +=
                        2.0 * pr.innovation[3+k] * pr.innovation[3+k] + R_gnss_diag[3+k];
                }
            }
        }
        if (d != GateDecision::REJECT) {
            ModelUpdateRecord records[N_REGIMES];
            for (int i = 0; i < N_REGIMES; ++i) {
                auto ur = imm.models[i].measurement_update<6>(
                    t_s, z, h_gnss, H_gnss, R, H_pre, -1.0);
                records[i].accepted       = ur.accepted;
                records[i].mahalanobis_sq = ur.mahalanobis_sq;
                records[i].meas_dim       = 6;
                for (int r2 = 0; r2 < 6; ++r2)
                    for (int c2 = 0; c2 < 6; ++c2)
                        records[i].innovation_cov[r2][c2] = ur.innovation_cov[r2][c2];
            }
            imm.update(records);
            double innov[6];
            for (int k = 0; k < 6; ++k) innov[k] = pr.innovation[k];
            adapt_r.observe(SENSOR_GNSS, innov, 6);
        }
        return emit_output(t_s, baro_last_alt, pz);
    }

    // -----------------------------------------------------------------------
    // Magnetometer update (3-D body-frame field vector)
    // -----------------------------------------------------------------------
    void ingest_mag(double t_s, double bx, double by, double bz) noexcept {
        const double z[3] = { bx, by, bz };
        double R[3][3] = {};
        R[0][0] = R[1][1] = R[2][2] = 1.0;  // Will be updated by adaptive layer

        const double H_pre = effective_health(SENSOR_MAG);
        ModelUpdateRecord records[N_REGIMES];
        for (int i = 0; i < N_REGIMES; ++i) {
            auto ur = imm.models[i].measurement_update<3>(
                t_s, z, h_mag, H_mag, R, H_pre, -1.0);
            records[i].accepted       = ur.accepted;
            records[i].mahalanobis_sq = ur.mahalanobis_sq;
            records[i].meas_dim       = 3;
            for (int r2 = 0; r2 < 3; ++r2)
                for (int c2 = 0; c2 < 3; ++c2)
                    records[i].innovation_cov[r2][c2] = ur.innovation_cov[r2][c2];
            if (i == best_model_idx()) {
                health_mon.update(SENSOR_MAG, ur.mahalanobis_sq, z, 3, t_s, false);
                fdir.evaluate(SENSOR_MAG, t_s, ur.normalised_residual, 3);
            }
        }
        imm.update(records);
    }

private:
    double  last_imu_t = -1.0;
    double  last_sup_t = -1.0;
    bool    fault_alarm = false;

    // Barometric rate (for analytical redundancy)
    double baro_dot_buf[8];
    int    baro_dot_buf_idx = 0;
    int    baro_dot_n       = 0;
    double baro_last_alt    = 0.0;
    double baro_last_t      = -1.0;

    // ---- Innovation gate state (one per sensor) --------------------------
    enum class GateDecision : uint8_t { FUSE, REJECT, RESYNC };
    struct GateState {
        bool     rejecting   = false;
        double   t_first     = 0.0;
        bool     have_prev   = false;
        double   prev_r[MAX_MEAS_DIM] = {};
        double   diff_sq_sum = 0.0;
        int      n_diff      = 0;
        uint32_t n_rejected  = 0;
        void reset() noexcept { *this = GateState{}; }
    };
    GateState gate_[N_SENSORS];

    static const char* sensor_name(int s) noexcept {
        switch (s) { case SENSOR_IMU: return "IMU"; case SENSOR_BARO: return "BARO";
                     case SENSOR_GNSS: return "GNSS"; case SENSOR_MAG: return "MAG"; }
        return "?";
    }

    /// Decide whether a probed measurement may touch the state.
    /// @param nis    νᵀS⁻¹ν from the probe
    /// @param r_norm innovation normalised by √R (for the noise-level test)
    GateDecision gate_decide(int s, double t_s, double nis, double gate_chi2,
                             const double r_norm[], int M) noexcept {
        GateState& g = gate_[s];
        const bool inconsistent = (nis > gate_chi2) || fdir.is_quarantined(s);

        if (!inconsistent) {
            if (g.rejecting)
                ESP_LOGW("FDIR", "%s consistent again after %u rejected samples -> fusing",
                         sensor_name(s), (unsigned)g.n_rejected);
            g.reset();
            return GateDecision::FUSE;
        }

        if (!g.rejecting) {
            g.rejecting = true;
            g.t_first   = t_s;
            ESP_LOGW("FDIR", "%s ANOMALY: NIS=%.1f (gate %.1f)%s -> holding last valid state",
                     sensor_name(s), nis, gate_chi2,
                     fdir.is_quarantined(s) ? " [quarantined]" : "");
        }
        g.n_rejected++;

        // Sample-to-sample noise of the rejected stream. A healthy sensor
        // against a diverged prediction shows a smooth (white, ~√2) difference;
        // a faulty sensor (spikes, dropouts, erratic jumps) does not.
        if (g.have_prev) {
            for (int k = 0; k < M; ++k) {
                const double d = r_norm[k] - g.prev_r[k];
                g.diff_sq_sum += d * d;
                g.n_diff++;
            }
        }
        for (int k = 0; k < M; ++k) g.prev_r[k] = r_norm[k];
        g.have_prev = true;

        if ((t_s - g.t_first) >= ESTIMATOR_CFG.resync_after_s && g.n_diff >= 10) {
            const double rms = std::sqrt(g.diff_sq_sum / g.n_diff);
            if (rms < ESTIMATOR_CFG.resync_noise_max) {
                ESP_LOGW("FDIR", "%s rejected %.1fs but noise nominal (rms=%.2f) -> "
                         "prediction diverged, RE-SYNC", sensor_name(s), t_s - g.t_first, rms);
                fdir.sensors[s].reset();
                g.reset();
                return GateDecision::RESYNC;
            }
            // Erratic: restart the observation window, keep rejecting
            g.t_first = t_s; g.diff_sq_sum = 0.0; g.n_diff = 0;
        }
        return GateDecision::REJECT;
    }

    int best_model_idx() const noexcept {
        int best = 0;
        for (int i = 1; i < N_REGIMES; ++i)
            if (imm.mu[i] > imm.mu[best]) best = i;
        return best;
    }

    double effective_health(int sensor_id) const noexcept {
        double h = health_mon.health(sensor_id);
        if (fdir.is_quarantined(sensor_id))
            h = ESTIMATOR_CFG.health_floor;
        return h;
    }

    void update_baro_dot(double alt, double t) noexcept {
        if (baro_last_t > 0.0) {
            const double dt = t - baro_last_t;
            if (dt > 0.0 && dt < 1.0) {
                baro_dot_buf[baro_dot_buf_idx] = (alt - baro_last_alt) / dt;
                baro_dot_buf_idx = (baro_dot_buf_idx + 1) % 8;
                if (baro_dot_n < 8) baro_dot_n++;
            }
        }
        baro_last_alt = alt;
        baro_last_t   = t;
    }

    const FlightComputerOutput& emit_output(double t_s, double baro_alt, double gnss_alt) noexcept {
        const IMMOutput& fused = imm.fuse();
        const double vel_z   = fused.nav.v(2);

        // While the baro is held as anomalous, the mission logic must not act
        // on it: feed the supervisor the protected (IMU-coasted) estimate.
        const bool   baro_held = gate_[SENSOR_BARO].rejecting;
        const double alt_m     = baro_held ? fused.nav.p(2) : baro_alt;

        double dt_sup = 0.0;
        if (last_sup_t >= 0.0) dt_sup = t_s - last_sup_t;
        last_sup_t = t_s;

        double h_snap[N_SENSORS];
        health_mon.snapshot(h_snap);

        // Analytical redundancy check and effective vertical speed
        double baro_dot = 0.0;
        for (int i = 0; i < baro_dot_n; ++i) baro_dot += baro_dot_buf[i];
        if (baro_dot_n > 0) baro_dot /= baro_dot_n;

        const double vel_eff = (baro_held || std::abs(vel_z) > 0.3) ? vel_z : baro_dot;

        const SupervisorOutput sup_out = supervisor.step(
            dt_sup, fused, alt_m, vel_eff, h_snap, t_s);

        bool ar_suspect[3] = {};
        fdir.analytical_redundancy(
            baro_dot, vel_z, fused.nav.v(2), true,
            ESTIMATOR_CFG.analytical_redundancy_tol_mps, ar_suspect);

        fault_alarm = false;
        for (int i = 0; i < N_SENSORS; ++i)
            if (fdir.is_quarantined(i)) { fault_alarm = true; break; }

        last_output.t_s           = t_s;
        last_output.imm           = fused;
        last_output.sup           = sup_out;
        for (int i = 0; i < N_SENSORS; ++i) last_output.health[i] = h_snap[i];
        for (int i = 0; i < N_SENSORS; ++i)
            last_output.fdir_quarantine[i] = fdir.is_quarantined(i);
        last_output.fault_alarm   = fault_alarm;
        last_output.baro_alt_m    = baro_alt;
        last_output.gnss_alt_m    = gnss_alt;
        last_output.alt_est_m     = alt_m;
        last_output.baro_held     = baro_held;
        last_output.gnss_held     = gate_[SENSOR_GNSS].rejecting;
        output_valid              = true;
        return last_output;
    }
};

} // namespace nav
