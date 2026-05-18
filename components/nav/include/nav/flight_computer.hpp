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

        // Feed specific force to supervisor (launch detection Channel A/D)
        supervisor.set_imu_specific_force(f_b);

        // Get current VS-IMM gate
        double vs_gate[N_REGIMES][N_REGIMES];
        supervisor.build_vs_gate(vs_gate);

        // Propagate all IMM models
        imm.predict(f_b, w_b, dt, vs_gate);
    }

    // -----------------------------------------------------------------------
    // Barometer update (1-D altitude AGL)
    // -----------------------------------------------------------------------
    FlightComputerOutput ingest_baro(double t_s, double alt_agl_m) noexcept {
        // Update baro rate estimate (for analytical redundancy)
        update_baro_dot(alt_agl_m, t_s);

        // Build R (adaptive if enabled)
        double R_baro_diag[1];
        if (ESTIMATOR_CFG.adaptive_enabled)
            adapt_r.current_diag(SENSOR_BARO, R_baro_diag, 1);
        else
            R_baro_diag[0] = BARO_CFG.sigma_h_floor_m * BARO_CFG.sigma_h_floor_m;

        const double H_pre = effective_health(SENSOR_BARO);
        ModelUpdateRecord records[N_REGIMES];
        double innov_arr[1];

        // Apply update to each IMM model
        for (int i = 0; i < N_REGIMES; ++i) {
            const double z[1] = { alt_agl_m };
            double R[1][1] = {{ R_baro_diag[0] }};
            auto ur = imm.models[i].measurement_update<1>(
                t_s, z, h_baro, H_baro, R, H_pre, -1.0);
            records[i].accepted       = ur.accepted;
            records[i].mahalanobis_sq = ur.mahalanobis_sq;
            records[i].meas_dim       = 1;
            records[i].innovation_cov[0][0] = ur.innovation_cov[0][0];
            if (i == best_model_idx()) {
                innov_arr[0] = ur.innovation[0];
                health_mon.update(SENSOR_BARO, ur.mahalanobis_sq,
                                  z, 1, t_s, false);
                fdir.evaluate(SENSOR_BARO, t_s, ur.normalised_residual, 1);
                adapt_r.observe(SENSOR_BARO, innov_arr, 1);
            }
        }
        imm.update(records);
        return emit_output(t_s, alt_agl_m, 0.0);
    }

    // -----------------------------------------------------------------------
    // GNSS/NavIC update (6-D: position + velocity in ENU)
    // -----------------------------------------------------------------------
    FlightComputerOutput ingest_gnss(double t_s,
                                      double px, double py, double pz,
                                      double vx, double vy, double vz) noexcept {
        double R_gnss_diag[6];
        if (ESTIMATOR_CFG.adaptive_enabled)
            adapt_r.current_diag(SENSOR_GNSS, R_gnss_diag, 6);
        else {
            R_gnss_diag[0] = R_gnss_diag[1] = GNSS_CFG.horizontal_pos_std_m * GNSS_CFG.horizontal_pos_std_m;
            R_gnss_diag[2] = GNSS_CFG.vertical_pos_std_m * GNSS_CFG.vertical_pos_std_m;
            R_gnss_diag[3] = R_gnss_diag[4] = R_gnss_diag[5] =
                GNSS_CFG.horizontal_vel_std_mps * GNSS_CFG.horizontal_vel_std_mps;
        }

        const double H_pre = effective_health(SENSOR_GNSS);
        const double z[6] = { px, py, pz, vx, vy, vz };
        double R[6][6] = {};
        for (int i = 0; i < 6; ++i) R[i][i] = R_gnss_diag[i];

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
            if (i == best_model_idx()) {
                double innov[6];
                for (int k = 0; k < 6; ++k) innov[k] = ur.innovation[k];
                health_mon.update(SENSOR_GNSS, ur.mahalanobis_sq, z, 6, t_s, false);
                fdir.evaluate(SENSOR_GNSS, t_s, ur.normalised_residual, 6);
                adapt_r.observe(SENSOR_GNSS, innov, 6);
            }
        }
        imm.update(records);
        return emit_output(t_s, 0.0, pz);
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

    FlightComputerOutput emit_output(double t_s, double baro_alt, double gnss_alt) noexcept {
        const IMMOutput fused = imm.fuse();
        const double alt_m   = fused.nav.p(2);
        const double vel_z   = fused.nav.v(2);

        double dt_sup = 0.0;
        if (last_sup_t >= 0.0) dt_sup = t_s - last_sup_t;
        last_sup_t = t_s;

        double h_snap[N_SENSORS];
        health_mon.snapshot(h_snap);

        const SupervisorOutput sup_out = supervisor.step(
            dt_sup, fused, alt_m, vel_z, h_snap, t_s);

        // Analytical redundancy check
        double baro_dot = 0.0;
        for (int i = 0; i < baro_dot_n; ++i) baro_dot += baro_dot_buf[i];
        if (baro_dot_n > 0) baro_dot /= baro_dot_n;
        bool ar_suspect[3] = {};
        fdir.analytical_redundancy(
            baro_dot, vel_z, fused.nav.v(2), true,
            ESTIMATOR_CFG.analytical_redundancy_tol_mps, ar_suspect);

        fault_alarm = false;
        for (int i = 0; i < N_SENSORS; ++i)
            if (fdir.is_quarantined(i)) { fault_alarm = true; break; }

        FlightComputerOutput out{};
        out.t_s           = t_s;
        out.imm           = fused;
        out.sup           = sup_out;
        for (int i = 0; i < N_SENSORS; ++i) out.health[i] = h_snap[i];
        for (int i = 0; i < N_SENSORS; ++i)
            out.fdir_quarantine[i] = fdir.is_quarantined(i);
        out.fault_alarm   = fault_alarm;
        out.baro_alt_m    = baro_alt;
        out.gnss_alt_m    = gnss_alt;
        last_output       = out;
        output_valid      = true;
        return out;
    }
};

} // namespace nav
