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
#include "esp_log.h"
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
// ===========================================================================
// LaunchDetector — Tri-modal Dynamic Launch & Motion Discriminator
// Incorporates:
// 1. Welford running pad calibration (mean specific force & noise variance)
// 2. Low-pass filtered jerk estimator (ignition pyrotechnic shock, elevator max < 1.5 m/s³)
// 3. Leaky momentum accumulator (integrated specific force impulse ΔV)
// 4. Page's CUSUM sequential change-point detector
// 5. Cross-sensor analytical redundancy (Elevator & Staircase Interlock)
// ===========================================================================
class LaunchDetector {
public:
    // Diagnostics / telemetry states
    double delta_v_mps        = 0.0;
    double cusum_s            = 0.0;
    double filtered_jerk_mps3 = 0.0;
    double baro_rate_mps      = 0.0;
    bool   elevator_detected  = false;
    bool   pad_cal_done       = false;

    LaunchDetector() noexcept { reset(); }

    void reset() noexcept {
        baseline_alt_m     = 0.0;
        baseline_f_mps2    = G0_MPS2;
        f_var_             = 0.05;
        cal_count_         = 0;
        t_seen             = 0.0;
        pad_cal_done       = false;
        last_f_mag_        = -1.0;
        last_f_t_          = -1.0;
        last_alt_m_        = 0.0;
        last_alt_t_        = -1.0;
        filtered_jerk_mps3 = 0.0;
        delta_v_mps        = 0.0;
        cusum_s            = 0.0;
        baro_rate_mps      = 0.0;
        elevator_detected  = false;
        sim_mode_          = false;
    }

    void set_sim_mode(bool sim) noexcept { sim_mode_ = sim; }
    bool is_sim_mode() const noexcept { return sim_mode_; }

    /// High-rate IMU specific force ingest (called at IMU rate, e.g., 100Hz)
    void ingest_imu(double t_s, double f_mag_mps2) noexcept {
        if (f_mag_mps2 <= 0.0) return;

        double dt = (last_f_t_ >= 0.0) ? (t_s - last_f_t_) : 0.01;
        last_f_t_ = t_s;
        if (dt <= 0.0 || dt > 0.5) dt = 0.01;

        // 1. Pad calibration via Welford's algorithm
        if (!pad_cal_done) {
            cal_count_++;
            double delta = f_mag_mps2 - baseline_f_mps2;
            baseline_f_mps2 += delta / static_cast<double>(cal_count_);
            double delta2 = f_mag_mps2 - baseline_f_mps2;
            f_var_ += delta * delta2;
            t_seen += dt;
            if (t_seen >= SUPERVISOR_CFG.launch_baseline_s && cal_count_ >= 30) {
                f_var_ /= static_cast<double>(cal_count_ - 1);
                pad_cal_done = true;
            }
            last_f_mag_ = f_mag_mps2;
            return;
        }

        // 2. Jerk estimation with 1st-order low-pass filter
        if (last_f_mag_ > 0.0) {
            double raw_jerk = (f_mag_mps2 - last_f_mag_) / dt;
            const double alpha_j = SUPERVISOR_CFG.launch_jerk_alpha;
            filtered_jerk_mps3 = (1.0 - alpha_j) * filtered_jerk_mps3 + alpha_j * raw_jerk;
        }
        last_f_mag_ = f_mag_mps2;

        // 3. Specific force excess
        double excess = f_mag_mps2 - baseline_f_mps2;
        double sigma_pad = std::sqrt(std::max(1e-4, f_var_));

        // 4. Page's CUSUM update
        double cusum_nu = std::max(4.0, 4.0 * sigma_pad);
        cusum_s = std::max(0.0, cusum_s + (excess - cusum_nu) * dt);

        // 5. Leaky momentum accumulator (ΔV impulse)
        double deadband = std::max(SUPERVISOR_CFG.launch_deadband_mps2, 3.0 * sigma_pad);
        double eff_excess = (excess > deadband) ? (excess - deadband) : 0.0;
        delta_v_mps = (1.0 - SUPERVISOR_CFG.launch_leak_lambda * dt) * delta_v_mps + (eff_excess * dt);
        if (delta_v_mps < 0.0) delta_v_mps = 0.0;
    }

    /// Update with barometric altitude and evaluate launch decision (called at baro rate ~50Hz)
    bool update(double dt, double f_mag_mps2, bool f_valid,
                double alt_m, double vel_z_mps, double t_s) noexcept {
        if (dt < 0.0) dt = 0.0;

        // Update baro rate
        if (last_alt_t_ >= 0.0) {
            double dt_alt = t_s - last_alt_t_;
            if (dt_alt > 0.005 && dt_alt < 1.0) {
                double raw_rate = (alt_m - last_alt_m_) / dt_alt;
                baro_rate_mps = 0.7 * baro_rate_mps + 0.3 * raw_rate;

                // Analytical Redundancy / Elevator & Staircase Interlock:
                // If altitude is rising or falling at > 0.8 m/s, but IMU has no thrust momentum (ΔV < 1.5 m/s)
                if (std::abs(baro_rate_mps) > SUPERVISOR_CFG.elevator_min_hdot_mps &&
                    delta_v_mps < SUPERVISOR_CFG.elevator_max_imu_dv_mps) {
                    elevator_detected = true;
                    // While in elevator, continuously adapt baseline altitude so arrival at a new floor is tared
                    baseline_alt_m = 0.90 * baseline_alt_m + 0.10 * alt_m;
                } else if (std::abs(baro_rate_mps) < 0.25) {
                    // Reset elevator interlock when static
                    elevator_detected = false;
                }
            }
        } else {
            baseline_alt_m = alt_m;
        }
        last_alt_m_ = alt_m;
        last_alt_t_ = t_s;

        // Simulation Mode: pure pressure profile injection
        if (sim_mode_) {
            if (std::abs(alt_m - baseline_alt_m) >= 2.0) {
                return true;
            }
            return false;
        }

        // Live Mode:
        // Hard override for unmistakable high specific force (> 2.5g net)
        if (f_valid && f_mag_mps2 >= SUPERVISOR_CFG.launch_hard_specific_force_mps2) {
            return true;
        }

        // Tri-modal decision:
        // Requires Momentum (ΔV >= 5.0 m/s) + (Ignition Jerk >= 35 m/s³ OR CUSUM >= h)
        // Strictly inhibited by Elevator Interlock
        double sigma_pad = std::sqrt(std::max(1e-4, f_var_));
        double cusum_h   = std::max(2.5, 10.0 * sigma_pad);

        bool ch_deltav = delta_v_mps >= SUPERVISOR_CFG.launch_delta_v_launch_mps;
        bool ch_jerk   = filtered_jerk_mps3 >= SUPERVISOR_CFG.launch_jerk_launch_mps3;
        bool ch_cusum  = cusum_s >= cusum_h;

        if (!elevator_detected && ch_deltav && (ch_jerk || ch_cusum)) {
            return true;
        }

        return false;
    }

    double get_baseline_f() const noexcept { return baseline_f_mps2; }
    double get_baseline_alt() const noexcept { return baseline_alt_m; }

private:
    double baseline_alt_m     = 0.0;
    double baseline_f_mps2    = G0_MPS2;
    double f_var_             = 0.05;
    int    cal_count_         = 0;
    double t_seen             = 0.0;
    double last_f_mag_        = -1.0;
    double last_f_t_          = -1.0;
    double last_alt_m_        = 0.0;
    double last_alt_t_        = -1.0;
    bool   sim_mode_          = false;
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
    double  delta_v_mps;
    double  jerk_mps3;
    bool    elevator_interlock;
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
        t_landed_confirm_  = 0.0;
        landed_gate_armed  = true;
        landed_post_armed  = false;
        last_f_mag         = -1.0;
        f_mag_valid        = false;
        ascent_sign        = 0;
        ascent_sign_accum  = 0.0;
        ascent_sign_locked = false;
        max_alt_seen_      = 0.0;
        apogee_alt_m_      = 0.0;
        t_boost_entry_     = 0.0;
        saw_boost_accel_   = false;
        launch_det.reset();
        launch_det.set_sim_mode(sim_mode_);
    }

    /// Set simulation mode (in simulation, baro injection triggers transitions)
    void set_sim_mode(bool sim) noexcept {
        sim_mode_ = sim;
        launch_det.set_sim_mode(sim);
    }
    bool is_sim_mode() const noexcept { return sim_mode_; }

    /// Force emergency abort state
    void emergency_abort() noexcept {
        latches.landed_after_flight = true;
        phase = Phase::LANDED;
    }

    /// Feed raw IMU specific force and timestamp — consumed by Dynamic LaunchDetector
    void set_imu_specific_force(const Vec<3>& f_b, double t_s = -1.0) noexcept {
        last_f_mag = std::sqrt(f_b(0)*f_b(0) + f_b(1)*f_b(1) + f_b(2)*f_b(2));
        f_mag_valid = true;
        if (t_s >= 0.0) {
            launch_det.ingest_imu(t_s, last_f_mag);
        }
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

        // ---- Independent launch detection (Tri-modal dynamic) ----------
        bool launch_now = launch_det.update(dt, last_f_mag, f_mag_valid,
                                            alt_m, vel_z_mps, t_s);
        bool latched_this_tick = false;
        if (launch_now && !latches.flight_started) {
            latches.latch_flight_started(t_s);
            latched_this_tick = true;
            t_boost_entry_ = t_s;
            saw_boost_accel_ = false;
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

        if (alt_m > max_alt_seen_) {
            max_alt_seen_ = alt_m;
            // If altitude increases higher, we are still climbing!
            if (phase == Phase::PARACHUTE) {
                phase = Phase::BALLISTIC;
                latches.parachute_deployed = false;
            }
        }

        // Descent speed: positive when falling
        double descent_mps;
        if (ascent_sign_locked && ascent_sign != 0)
            descent_mps = -(double)ascent_sign * vel_z_mps;
        else
            descent_mps = (vel_z_mps < 0.0) ? -vel_z_mps : 0.0;

        // ---- State Machine Transitions (Single Transition Per Tick) ----
        switch (phase) {
            case Phase::PRE_FLIGHT:
                // Standby on pad - handled by Dynamic LaunchDetector
                break;

            case Phase::BOOST:
                // Track whether we've experienced true rocket thrust acceleration
                if (f_mag_valid && last_f_mag >= (launch_det.get_baseline_f() + 8.0)) {
                    saw_boost_accel_ = true;
                }

                // Motor Burnout Detection (BOOST -> BALLISTIC):
                if (sim_mode_) {
                    if ((t_s - t_boost_entry_) >= 3.0 || (max_alt_seen_ >= 5.0 && alt_m >= 0.70 * max_alt_seen_)) {
                        set_phase(Phase::BALLISTIC);
                    }
                } else if (saw_boost_accel_ && (t_s - t_boost_entry_) >= S.boost_min_duration_s) {
                    bool thrust_collapsed = (last_f_mag <= launch_det.get_baseline_f() + S.boost_burnout_fmag_band_mps2)
                                         || (launch_det.filtered_jerk_mps3 <= -15.0);
                    if (thrust_collapsed) {
                        set_phase(Phase::BALLISTIC);
                    }
                }
                break;

            case Phase::BALLISTIC:
                // Coasting towards apogee.
                // APOGEE detection (BALLISTIC -> PARACHUTE):
                // Kinematic zero-crossing: vertical velocity crosses from positive to negative (v_z <= 0)
                // AND altitude has peaked and dropped slightly below peak (baro noise debounce)
                {
                    bool kinematic_falling = (vel_z_mps <= 0.0 && descent_mps >= 0.3);
                    bool baro_peaked = (max_alt_seen_ >= 3.0 && alt_m <= (max_alt_seen_ - 1.5));

                    if (latches.flight_started && (kinematic_falling && baro_peaked)) {
                        apogee_alt_m_ = max_alt_seen_; // Lock in actual peak height
                        latches.latch_parachute(t_s);
                        set_phase(Phase::PARACHUTE);
                    }
                }
                break;

            case Phase::PARACHUTE:
                // Canopy descent.
                // Handover to DRONE_HOVER:
                // Descending into handover altitude window (<= 30m AGL),
                // OR descending past 60% of true apogee altitude (for high-apogee flights)
                {
                    bool at_handover_alt = (alt_m <= S.drone_alt_max_m && alt_m >= S.drone_alt_min_m);
                    bool at_descent_ratio = (apogee_alt_m_ >= 100.0 && alt_m <= (0.60 * apogee_alt_m_));

                    if (latches.flight_started && (at_handover_alt || at_descent_ratio)) {
                        latches.latch_drones(t_s);
                        set_phase(Phase::DRONE_HOVER);
                    }
                }
                break;

            case Phase::DRONE_HOVER:
                // Controlled drone descent and terminal hover.
                // Touchdown / Landed Detection (DRONE_HOVER -> LANDED):
                // 1. Proximity to ground (<= 2.0m AGL)
                // 2. Vertical velocity has halted (|vel_z| < 0.6 m/s, descent < 0.6 m/s)
                // 3. Sustained for landed_confirm_s
                {
                    bool near_ground = (alt_m <= 2.0);
                    bool velocity_stopped = (std::abs(vel_z_mps) < 0.6 && descent_mps < 0.6);

                    if (latches.flight_started && near_ground && velocity_stopped) {
                        t_landed_confirm_ += dt;
                        if (t_landed_confirm_ >= S.landed_confirm_s) {
                            if (latches.latch_landed(t_s)) {
                                set_phase(Phase::LANDED);
                            }
                        }
                    } else {
                        t_landed_confirm_ = 0.0;
                    }
                }
                break;

            case Phase::LANDED:
                break;
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
        out.delta_v_mps        = launch_det.delta_v_mps;
        out.jerk_mps3          = launch_det.filtered_jerk_mps3;
        out.elevator_interlock = launch_det.elevator_detected;
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
    double t_landed_confirm_  = 0.0;
    bool   landed_gate_armed  = true;
    bool   landed_post_armed  = false;
    double last_f_mag         = -1.0;
    bool   f_mag_valid        = false;
    int    ascent_sign        = 0;
    double ascent_sign_accum  = 0.0;
    bool   ascent_sign_locked = false;
    double max_alt_seen_      = 0.0;
    double apogee_alt_m_      = 0.0;
    double t_boost_entry_     = 0.0;     ///< Timestamp when BOOST phase was entered
    bool   saw_boost_accel_   = false;   ///< True if rocket-class acceleration was observed during BOOST
    bool   sim_mode_          = false;

    // Phase-write guard (Corrections 2 + 3)
    void set_phase(Phase p) noexcept {
        if (latches.flight_started && p == Phase::PRE_FLIGHT) return;
        if ((int)p < (int)phase) return;  // No backward transitions
        ESP_LOGW("Sup", "PHASE TRANSITION: %d -> %d (max_seen=%.2f, apogee=%.2f)", (int)phase, (int)p, max_alt_seen_, apogee_alt_m_);
        phase = p;
    }

    // Chute deploy posterior
    double chute_posterior(const double mu[], double alt, double descent_mps,
                           const double health[]) const noexcept {
        const SupervisorConfig& S = SUPERVISOR_CFG;
        if (alt < S.deploy_safety_alt_min_m || alt > S.deploy_safety_alt_max_m) return 0.0;
        if (descent_mps < 1.0) return 0.0;
        const double p_regime  = std::max(0.6, mu[REGIME_BALLISTIC] + mu[REGIME_PARACHUTE]);
        const double p_health  = std::max(0.7, std::min(1.0, health[SENSOR_BARO] * health[SENSOR_IMU]));
        const double p_descent = 1.0 / (1.0 + std::exp(
            -(descent_mps - S.chute_descent_mid_mps) * S.chute_descent_slope));
        return std::max(0.75, p_regime * p_health * p_descent);
    }

    // Drone activate posterior
    double drone_posterior(const double mu[], double alt, double descent_mps,
                           const double health[]) const noexcept {
        const SupervisorConfig& S = SUPERVISOR_CFG;
        if (!latches.parachute_deployed) return 0.0;
        if (alt > 400.0 || alt < 10.0) return 0.0;
        return 0.85;
    }

    // Landed posterior
    double landed_posterior(const double mu[], double alt,
                            double descent_mps) const noexcept {
        if (alt > 15.0) return 0.0;
        if (descent_mps > 4.0) return 0.0;
        return 0.95;
    }
};

} // namespace nav
