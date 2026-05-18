/**
 * @file supervisor.hpp
 * @brief Bayesian supervisory controller + independent launch detector.
 *
 * Faithful C++ port of supervisor.py (v2.1).
 *
 * CAN-7USAT mission phases (v2.1 architecture):
 *   0  BOOT           → Pre-power-on (maps to PRE_FLIGHT on pad)
 *   1  TEST_MODE      → Ground test
 *   2  LAUNCH_PAD     → Armed on pad, waiting
 *   3  ASCENT         → Boost + Ballistic phases combined (BOOST→BALLISTIC)
 *   4  ROCKET_DEPLOY  → Secondary deployment triggered
 *   5  DESCENT        → Parachute descent
 *   6  AEROBREAK_RELEASE → Drone activation
 *   7  IMPACT         → Landed
 *
 * The supervisor's internal phase matches the Python enum:
 *   PRE_FLIGHT / BOOST / BALLISTIC / PARACHUTE / DRONE_HOVER / LANDED
 *
 * Corrections implemented (doctrine v2.1):
 *   1. Independent multi-channel launch detection (2-of-3 majority vote).
 *   2. Irreversible mission latches (False→True only).
 *   3. PRE_FLIGHT unreachable after launch.
 *   4. Soft VS-IMM gating (floor instead of zero for physically-implausible transitions).
 *   5. Hysteresis on all physical interlocks (Schmitt triggers).
 *   6. Separate PRE_FLIGHT/LANDED semantics via supervisor latches.
 *
 * @compliance CAN-7USAT §5.1 (Deployment interlocks), §6.2 (Safety assurance)
 */
#pragma once

#include "imm.hpp"
#include "config.hpp"
#include <cmath>

namespace nav {

// ===========================================================================
// Mission phase enumeration — matches CAN-7USAT §3 states
// ===========================================================================
enum class Phase : uint8_t {
    PRE_FLIGHT  = 0,
    BOOST       = 1,
    BALLISTIC   = 2,
    PARACHUTE   = 3,
    DRONE_HOVER = 4,
    LANDED      = 5,
};

// Map supervisor phase → IMM regime row index
constexpr int phase_to_regime(Phase p) noexcept {
    switch (p) {
        case Phase::PRE_FLIGHT:  return REGIME_LANDED;    // kinematically identical
        case Phase::BOOST:       return REGIME_BOOST;
        case Phase::BALLISTIC:   return REGIME_BALLISTIC;
        case Phase::PARACHUTE:   return REGIME_PARACHUTE;
        case Phase::DRONE_HOVER: return REGIME_DRONE_HOVER;
        case Phase::LANDED:      return REGIME_LANDED;
    }
    return REGIME_LANDED;
}

// CAN-7USAT software state index (for telemetry SOFTWARE_STATE field)
constexpr uint8_t phase_to_state_code(Phase p) noexcept {
    switch (p) {
        case Phase::PRE_FLIGHT:  return 2;  // LAUNCH_PAD
        case Phase::BOOST:       return 3;  // ASCENT
        case Phase::BALLISTIC:   return 3;  // ASCENT (continued)
        case Phase::PARACHUTE:   return 4;  // ROCKET_DEPLOY → DESCENT
        case Phase::DRONE_HOVER: return 6;  // AEROBREAK_RELEASE
        case Phase::LANDED:      return 7;  // IMPACT
    }
    return 0;
}

// ===========================================================================
// MissionLatches — irreversible one-shot state booleans (Correction 2)
// Stored in RTC Fast Memory for reset-survivability (tagged with RTC_DATA_ATTR
// at usage site in main.cpp).
// ===========================================================================
struct MissionLatches {
    bool   flight_started        = false;
    bool   parachute_deployed    = false;
    bool   drones_active         = false;
    bool   landed_after_flight   = false;
    double t_flight_started      = 0.0;
    double t_chute               = 0.0;
    double t_drone               = 0.0;
    double t_landed              = 0.0;

    bool latch_flight_started(double t) noexcept {
        if (flight_started) return false;
        flight_started = true; t_flight_started = t; return true;
    }
    bool latch_parachute(double t) noexcept {
        if (parachute_deployed) return false;
        parachute_deployed = true; t_chute = t; return true;
    }
    bool latch_drones(double t) noexcept {
        if (drones_active) return false;
        drones_active = true; t_drone = t; return true;
    }
    bool latch_landed(double t) noexcept {
        if (!flight_started) return false;
        if (landed_after_flight) return false;
        landed_after_flight = true; t_landed = t; return true;
    }
};

// ===========================================================================
// LaunchDetector — independent multi-channel (Correction 1)
// ===========================================================================
class LaunchDetector {
public:
    double tA = 0.0;  ///< Specific-force excess persistence (s)
    double tB = 0.0;  ///< Altitude rise persistence (s)
    double tC = 0.0;  ///< Vertical-velocity persistence (s)

    LaunchDetector() noexcept { reset(); }

    void reset() noexcept {
        baseline_alt_m  = 0.0;
        baseline_f_mps2 = G0_MPS2;
        baseline_set    = false;
        t_seen          = 0.0;
        cal_done        = false;
        tA = tB = tC    = 0.0;
    }

    /// Returns true if launch should be declared this tick.
    bool update(double dt, double f_mag_mps2,
                bool   f_valid,
                double alt_m,
                double vel_z_mps) noexcept {
        if (dt < 0.0) dt = 0.0;
        t_seen += dt;
        const SupervisorConfig& S = SUPERVISOR_CFG;

        // ---- Pad calibration -------------------------------------------
        if (!cal_done) {
            if (!baseline_set) {
                baseline_alt_m  = alt_m;
                baseline_f_mps2 = f_valid ? f_mag_mps2 : G0_MPS2;
                baseline_set    = true;
            } else {
                const double tau = S.launch_baseline_s;
                const double a   = dt / (tau + dt);
                baseline_alt_m  = (1.0-a)*baseline_alt_m  + a*alt_m;
                if (f_valid)
                    baseline_f_mps2 = (1.0-a)*baseline_f_mps2 + a*f_mag_mps2;
            }
            if (t_seen >= S.launch_baseline_s) cal_done = true;
            return false;
        }

        // ---- Channel A: specific-force excess ---------------------------
        if (f_valid) {
            const double excess = f_mag_mps2 - baseline_f_mps2;
            if (excess >= S.launch_specific_force_excess_mps2) tA += dt;
            else tA = (tA > dt) ? (tA - dt) : 0.0;
            // Channel D: hard override (no-doubt path)
            if (f_mag_mps2 >= S.launch_hard_specific_force_mps2) return true;
        }

        // ---- Channel B: altitude rise -----------------------------------
        if ((alt_m - baseline_alt_m) >= S.launch_altitude_rise_m) tB += dt;
        else tB = (tB > dt) ? (tB - dt) : 0.0;

        // ---- Channel C: vertical-velocity magnitude ---------------------
        if (std::abs(vel_z_mps) >= S.launch_vertical_vel_mps) tC += dt;
        else tC = (tC > dt) ? (tC - dt) : 0.0;

        // ---- 2-of-3 majority vote ---------------------------------------
        const double T = S.launch_persist_s;
        int votes = (tA >= T ? 1 : 0) + (tB >= T ? 1 : 0) + (tC >= T ? 1 : 0);
        return votes >= 2;
    }

private:
    double baseline_alt_m;
    double baseline_f_mps2;
    bool   baseline_set;
    double t_seen;
    bool   cal_done;
};

// ===========================================================================
// SupervisorOutput — result of one supervisor tick
// ===========================================================================
struct SupervisorOutput {
    Phase   phase;
    bool    parachute_deployed;
    bool    drones_active;
    bool    landed;
    double  p_deploy_chute;
    double  p_deploy_drone;
    double  p_landed;
    double  vs_gate[N_REGIMES][N_REGIMES];
    bool    flight_started;
    bool    launch_detected_this_tick;
    uint8_t state_code;             ///< For CAN-7USAT telemetry SOFTWARE_STATE
};

// ===========================================================================
// BayesianSupervisor
// ===========================================================================
class BayesianSupervisor {
public:
    MissionLatches latches;
    Phase          phase = Phase::PRE_FLIGHT;

    BayesianSupervisor() noexcept {
        reset();
    }

    void reset() noexcept {
        latches            = MissionLatches{};
        phase              = Phase::PRE_FLIGHT;
        t_chute            = 0.0;
        t_drone            = 0.0;
        t_landed           = 0.0;
        landed_gate_armed  = true;
        landed_post_armed  = false;
        last_f_mag         = -1.0;
        f_mag_valid        = false;
        ascent_sign        = 0;
        ascent_sign_accum  = 0.0;
        ascent_sign_locked = false;
        launch_det.reset();
    }

    /// Force emergency abort state
    void emergency_abort() noexcept {
        latches.landed_after_flight = true;
        phase = Phase::LANDED;
    }

    /// Feed raw IMU specific force — consumed by LaunchDetector (Correction 1).
    void set_imu_specific_force(const Vec<3>& f_b) noexcept {
        last_f_mag = std::sqrt(f_b(0)*f_b(0) + f_b(1)*f_b(1) + f_b(2)*f_b(2));
        f_mag_valid = true;
    }

    /// Build VS-IMM gate matrix (Correction 4 — soft floor).
    void build_vs_gate(double gate[N_REGIMES][N_REGIMES]) const noexcept {
        const double soft = SUPERVISOR_CFG.vs_gate_soft_floor;
        for (int i = 0; i < N_REGIMES; ++i)
            for (int j = 0; j < N_REGIMES; ++j)
                gate[i][j] = 1.0;

        // SOFT: DroneHover column attenuated until drones latched
        if (!latches.drones_active)
            for (int i = 0; i < N_REGIMES; ++i) gate[i][REGIME_DRONE_HOVER] = soft;

        // SOFT: Boost+Ballistic columns attenuated after parachute deployed
        if (latches.parachute_deployed) {
            for (int i = 0; i < N_REGIMES; ++i) {
                gate[i][REGIME_BOOST]     = soft;
                gate[i][REGIME_BALLISTIC] = soft;
            }
        }

        // SOFT: Parachute column attenuated once drones latched
        if (latches.drones_active)
            for (int i = 0; i < N_REGIMES; ++i) gate[i][REGIME_PARACHUTE] = soft;

        // HARD: Post-landing — landed row cannot step back to flight phases
        if (latches.landed_after_flight) {
            gate[REGIME_LANDED][REGIME_BOOST]       = 0.0;
            gate[REGIME_LANDED][REGIME_BALLISTIC]   = 0.0;
            gate[REGIME_LANDED][REGIME_PARACHUTE]   = 0.0;
            gate[REGIME_LANDED][REGIME_DRONE_HOVER] = 0.0;
        }
    }

    // -----------------------------------------------------------------------
    // Main supervisor tick — call once per aiding measurement
    // -----------------------------------------------------------------------
    SupervisorOutput step(double dt, const IMMOutput& imm,
                          double alt_m, double vel_z_mps,
                          const double health[N_SENSORS],
                          double t_s) noexcept {
        if (dt < 0.0 || dt > 1.0) dt = 0.0;  // guard

        const double* mu = imm.mu;
        const SupervisorConfig& S = SUPERVISOR_CFG;

        // ---- Independent launch detection (Correction 1) ---------------
        bool launch_now = launch_det.update(dt, last_f_mag, f_mag_valid,
                                            alt_m, vel_z_mps);
        bool latched_this_tick = false;
        if (launch_now && !latches.flight_started) {
            latches.latch_flight_started(t_s);
            latched_this_tick = true;
            set_phase(Phase::BOOST);
        }

        // ---- Sign-convention auto-detection (Correction 1 follow-on) --
        if (phase == Phase::BOOST && latches.flight_started
                && !ascent_sign_locked
                && std::abs(vel_z_mps) > S.ascent_sign_vel_min_mps) {
            ascent_sign_accum += (vel_z_mps > 0.0) ? 1.0 : -1.0;
        }
        if (latches.flight_started && !ascent_sign_locked
                && phase >= Phase::BALLISTIC
                && std::abs(ascent_sign_accum) >= S.ascent_sign_votes_required) {
            ascent_sign = (ascent_sign_accum > 0.0) ? 1 : -1;
            ascent_sign_locked = true;
        }

        // Descent speed: positive when falling
        double descent_mps;
        if (ascent_sign_locked && ascent_sign != 0)
            descent_mps = -(double)ascent_sign * vel_z_mps;
        else
            descent_mps = std::abs(vel_z_mps);

        // ---- BOOST → BALLISTIC -----------------------------------------
        if (phase == Phase::BOOST) {
            bool f_collapsed = f_mag_valid &&
                std::abs(last_f_mag - G0_MPS2) < S.boost_burnout_fmag_band_mps2;
            if ((mu[REGIME_BALLISTIC] > S.boost_to_ballistic_mu_on
                 && std::abs(vel_z_mps) > S.boost_tipover_vel_mps)
                || f_collapsed)
                set_phase(Phase::BALLISTIC);
        }

        // ---- BALLISTIC → PARACHUTE (probabilistic, debounced) ----------
        if (phase == Phase::BALLISTIC) {
            const double p_chute = chute_posterior(mu, alt_m, descent_mps, health);
            if (p_chute > S.deploy_posterior_threshold) t_chute += dt;
            else if (p_chute < S.deploy_posterior_off_thr)
                t_chute = (t_chute > dt) ? t_chute - dt : 0.0;
            if (t_chute >= S.deploy_confirm_s) {
                latches.latch_parachute(t_s);
                set_phase(Phase::PARACHUTE);
            }
        }

        // ---- PARACHUTE → DRONE_HOVER -----------------------------------
        if (phase == Phase::PARACHUTE) {
            const double p_drone = drone_posterior(mu, alt_m, descent_mps, health);
            if (p_drone > S.deploy_posterior_threshold) t_drone += dt;
            else if (p_drone < S.deploy_posterior_off_thr)
                t_drone = (t_drone > dt) ? t_drone - dt : 0.0;
            if (t_drone >= S.deploy_confirm_s) {
                latches.latch_drones(t_s);
                set_phase(Phase::DRONE_HOVER);
            }
        }

        // ---- LANDED detection ------------------------------------------
        if (phase == Phase::PARACHUTE || phase == Phase::DRONE_HOVER) {
            const double p_land = landed_posterior(mu, alt_m, descent_mps);
            // Hysteresis on landed posterior
            if (landed_post_armed) {
                if (p_land < S.landed_post_off) landed_post_armed = false;
            } else {
                if (p_land > S.landed_post_on)  landed_post_armed = true;
            }
            if (landed_post_armed) t_landed += dt;
            else t_landed = (t_landed > dt) ? t_landed - dt : 0.0;
            if (t_landed >= S.landed_confirm_s) {
                if (latches.latch_landed(t_s)) set_phase(Phase::LANDED);
            }
        }

        // ---- VS-IMM gate (with Schmitt-trigger altitude hysteresis) ----
        double gate[N_REGIMES][N_REGIMES];
        build_vs_gate(gate);

        // Altitude-based Landed-column soft gate (Correction 5)
        if (landed_gate_armed) {
            if (alt_m > S.landed_gate_alt_on_m)  {} // keep armed
            else if (alt_m < S.landed_gate_alt_off_m) landed_gate_armed = false;
        } else {
            if (alt_m > S.landed_gate_alt_on_m) landed_gate_armed = true;
        }
        if (landed_gate_armed) {
            const double soft = S.vs_gate_soft_floor;
            for (int i = 0; i < N_REGIMES; ++i)
                if (i != REGIME_LANDED)
                    gate[i][REGIME_LANDED] = soft;
        }

        // ---- Assemble output -------------------------------------------
        SupervisorOutput out{};
        out.phase              = phase;
        out.parachute_deployed = latches.parachute_deployed;
        out.drones_active      = latches.drones_active;
        out.landed             = latches.landed_after_flight;
        out.p_deploy_chute     = chute_posterior(mu, alt_m, descent_mps, health);
        out.p_deploy_drone     = drone_posterior(mu, alt_m, descent_mps, health);
        out.p_landed           = landed_posterior(mu, alt_m, descent_mps);
        out.flight_started     = latches.flight_started;
        out.launch_detected_this_tick = latched_this_tick;
        out.state_code         = phase_to_state_code(phase);
        for (int i = 0; i < N_REGIMES; ++i)
            for (int j = 0; j < N_REGIMES; ++j)
                out.vs_gate[i][j] = gate[i][j];
        return out;
    }

private:
    LaunchDetector launch_det;
    double t_chute            = 0.0;
    double t_drone            = 0.0;
    double t_landed           = 0.0;
    bool   landed_gate_armed  = true;
    bool   landed_post_armed  = false;
    double last_f_mag         = -1.0;
    bool   f_mag_valid        = false;
    int    ascent_sign        = 0;
    double ascent_sign_accum  = 0.0;
    bool   ascent_sign_locked = false;

    // Phase-write guard (Corrections 2 + 3)
    void set_phase(Phase p) noexcept {
        if (latches.flight_started && p == Phase::PRE_FLIGHT) return;
        if ((int)p < (int)phase) return;  // No backward transitions
        phase = p;
    }

    // Chute deploy posterior
    double chute_posterior(const double mu[], double alt, double descent_mps,
                           const double health[]) const noexcept {
        const SupervisorConfig& S = SUPERVISOR_CFG;
        if (alt < S.deploy_safety_alt_min_m || alt > S.deploy_safety_alt_max_m) return 0.0;
        if (descent_mps < std::abs(S.descent_on_mps)) return 0.0;
        const double p_regime  = mu[REGIME_BALLISTIC] + mu[REGIME_PARACHUTE];
        const double p_health  = std::min(1.0, health[SENSOR_BARO] * health[SENSOR_IMU]);
        const double p_descent = 1.0 / (1.0 + std::exp(
            -(descent_mps - S.chute_descent_mid_mps) * S.chute_descent_slope));
        return p_regime * p_health * p_descent;
    }

    // Drone activate posterior
    double drone_posterior(const double mu[], double alt, double descent_mps,
                           const double health[]) const noexcept {
        const SupervisorConfig& S = SUPERVISOR_CFG;
        if (!latches.parachute_deployed) return 0.0;
        if (alt > S.drone_alt_max_m || alt < S.drone_alt_min_m) return 0.0;
        if (descent_mps <= 0.0) return 0.0;
        const double p_regime = mu[REGIME_PARACHUTE] + mu[REGIME_DRONE_HOVER];
        const double da = (alt - S.drone_alt_center_m) / S.drone_alt_sigma_m;
        const double p_alt = std::exp(-0.5 * da * da);
        const double p_health = std::min(1.0, health[SENSOR_BARO] * health[SENSOR_IMU]);
        return p_regime * p_alt * p_health;
    }

    // Landed posterior
    double landed_posterior(const double mu[], double alt,
                            double descent_mps) const noexcept {
        const SupervisorConfig& S = SUPERVISOR_CFG;
        if (alt > S.landed_gate_alt_on_m) return 0.0;
        const double dv = descent_mps / S.landed_velocity_sigma_mps;
        const double da = alt / S.landed_altitude_sigma_m;
        return mu[REGIME_LANDED] * std::exp(-dv*dv) * std::exp(-da*da);
    }
};

} // namespace nav
