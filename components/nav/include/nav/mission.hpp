/**
 * @file mission.hpp
 * @brief Mission supervisor for the AAKASHVANI profile (single authority over actuators).
 *
 *   PAD ──launch──► ASCENT ──release──► DESCENT ──600 m──► ARMS_DEPLOY ──safe──► STEERING
 *                 (carrier drone             (passive chute     (2 linear servos    (motors vector
 *                  or rocket)                 already open)      unlatch the arms)   thrust to site)
 *   any post-release phase ──landed──► LANDED
 *
 * Launch  : rocket  = |f| >= 3 g for 50 ms, confirmed by >= 10 m gain within 4 s
 *           carrier = >= 25 m gain while climbing >= 1 m/s for 2 s (no thrust signature,
 *                     so a drone-carried climb is detected; the old elevator interlock blocked it)
 * Release : >= 5 m below peak AND sinking >= 2 m/s for 1 s AND recent free-fall/shock evidence
 *           (drop from the drone, ejection from the rocket); fallback >= 3 m/s for 3 s and
 *           >= 20 m below peak. A carrier slowly descending with the CanSat still attached
 *           never qualifies, so the arms can't open inside the carrier.
 * Arms    : AGL <= 600 m on descent, >= 2 s after release (clear of the carrier).
 * Motors  : after the arms have swung out, only with sane attitude, above the cut-off
 *           altitude; tumble failsafe with hysteresis; permanently off below cut-off.
 *
 * All flags are latching (never revert), and the whole state is persisted so a
 * brownout/watchdog reset mid-air resumes the mission instead of restarting on the pad.
 *
 * LIFT TEST mode (set_lift_test): the same state machine with thresholds scaled to a
 * building (launch on a 4 m sustained climb, release on a 1.5 m sustained descent, arms
 * at a chosen AGL, cut-off 3 m) and the MOTORS HARD-INHIBITED, so the whole sequence can
 * be exercised in an apartment lift without anything ever spinning.
 *
 * Header-only, no ESP-IDF dependencies: unit-tested on the host.
 */
#pragma once

#include "config.hpp"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstddef>

namespace nav {

enum class MissionPhase : uint8_t {
    PAD = 0, ASCENT = 1, DESCENT = 2, ARMS_DEPLOY = 3, STEERING = 4, LANDED = 5
};

inline const char* mission_phase_name(MissionPhase p) noexcept {
    switch (p) {
        case MissionPhase::PAD:         return "PAD";
        case MissionPhase::ASCENT:      return "ASCENT";
        case MissionPhase::DESCENT:     return "DESCENT";
        case MissionPhase::ARMS_DEPLOY: return "ARMS_DEPLOY";
        case MissionPhase::STEERING:    return "STEERING";
        case MissionPhase::LANDED:      return "LANDED";
    }
    return "?";
}

/// CAN-7USAT SOFTWARE_STATE telemetry code
inline uint8_t mission_state_code(MissionPhase p) noexcept {
    switch (p) {
        case MissionPhase::PAD:         return 2;
        case MissionPhase::ASCENT:      return 3;
        case MissionPhase::DESCENT:     return 4;
        case MissionPhase::ARMS_DEPLOY: return 5;
        case MissionPhase::STEERING:    return 6;
        case MissionPhase::LANDED:      return 7;
    }
    return 0;
}

/// Reset-survivable mission record (kept in RTC no-init RAM by main.cpp)
struct MissionPersist {
    static constexpr uint32_t MAGIC = 0xA4A5C0DEu;
    uint32_t magic;
    uint8_t  phase;
    uint8_t  arms_unlatched;
    uint8_t  motors_cut_low;
    uint8_t  pad_[1];
    float    pad_ref_m;        ///< Pad altitude in the baro frame
    float    h_max_m;          ///< Peak AGL
    double   baro_p0_pa;       ///< Baro reference pressure (keeps AGL continuous across a reset)
    double   site_lat_deg;     ///< Launch site (return target)
    double   site_lon_deg;
    float    site_alt_off_m;   ///< GNSS MSL - baro AGL on the pad
    uint8_t  site_valid;
    uint8_t  pad2_[3];
    float    lift_deploy_m;    ///< > 0: lift test mode with this arm-deploy AGL
    uint32_t crc;

    uint32_t compute_crc() const noexcept {
        const uint8_t* p = reinterpret_cast<const uint8_t*>(this);
        uint32_t h = 2166136261u;                              // FNV-1a
        for (size_t i = 0; i < offsetof(MissionPersist, crc); ++i) { h ^= p[i]; h *= 16777619u; }
        return h;
    }
    bool valid() const noexcept { return magic == MAGIC && crc == compute_crc(); }
    void seal() noexcept { magic = MAGIC; crc = compute_crc(); }
};

class MissionSupervisor {
public:
    struct Inputs {
        double t_s;
        float  h_m;          ///< Filtered altitude, baro frame (VerticalKF)
        float  v_mps;        ///< Filtered vertical speed, up +
        float  tilt_deg;     ///< Angle between vehicle +Z and world up
        float  rate_rps;     ///< |gyro|
        bool   attitude_ok;  ///< Attitude reference valid
    };

    struct Outputs {
        MissionPhase phase      = MissionPhase::PAD;
        float  agl_m            = 0.0f;   ///< Altitude above the pad
        float  h_max_m          = 0.0f;
        bool   arms_unlatched   = false;  ///< Servos to UNLATCH (latching)
        bool   motors_enabled   = false;  ///< Motors may spin (steering, not suspended)
        bool   steering_active  = false;  ///< Guidance should command tilt
        bool   motors_suspended = false;  ///< Tumble failsafe active
        bool   flight_started   = false;
        const char* event       = nullptr;///< One-shot human-readable event for the log
    };

    void reset() noexcept { const float lift = lift_deploy_; *this = MissionSupervisor{}; lift_deploy_ = lift; }
    void set_sim_mode(bool sim) noexcept { sim_ = sim; }

    /// Lift test: deploy_agl_m > 0 enables (motors inhibited), <= 0 disables. Pad only.
    void set_lift_test(float deploy_agl_m) noexcept { lift_deploy_ = deploy_agl_m > 0.0f ? deploy_agl_m : 0.0f; }
    bool lift_test() const noexcept { return lift_deploy_ > 0.0f; }
    float lift_deploy_m() const noexcept { return lift_deploy_; }

    /// 100 Hz specific-force magnitude (m/s^2): free-fall / shock / rocket-launch evidence
    void ingest_accel(double t_s, float f_mag) noexcept {
        const float dt = (last_f_t_ >= 0.0) ? (float)(t_s - last_f_t_) : 0.01f;
        last_f_t_ = t_s;
        if (!(dt > 0.0f) || dt > 0.5f) return;
        const FlightProfileConfig& C = PROFILE_CFG;

        // Rocket launch candidate (not in lift test: a lift has no boost)
        if (phase_ == MissionPhase::PAD && !lift_test()) {
            accel_hi_s_ = (f_mag >= C.launch_accel_mps2) ? accel_hi_s_ + dt : 0.0f;
            if (accel_hi_s_ >= C.launch_accel_s && t_launch_cand_ < 0.0) t_launch_cand_ = t_s;
        }
        // Release evidence (only meaningful once airborne)
        if (flight_started_) {
            freefall_s_ = (f_mag < C.freefall_mps2) ? freefall_s_ + dt : 0.0f;
            if (freefall_s_ >= C.freefall_s) t_evidence_ = t_s;
            if (f_mag >= C.shock_mps2 && (t_s - t_launch_) > 1.5) t_evidence_ = t_s;
        }
    }

    Outputs step(const Inputs& in, float dt) noexcept {
        const FlightProfileConfig& C = PROFILE_CFG;
        const Thresholds th = thresholds();
        Outputs out;
        if (!(dt > 0.0f) || dt > 1.0f) dt = 0.02f;
        const double t = in.t_s;

        // Pad reference: follow slow baro drift while waiting on the pad
        if (phase_ == MissionPhase::PAD) {
            if (!pad_ref_init_) { pad_ref_ = in.h_m; pad_ref_init_ = true; }
            if (std::fabs(in.v_mps) < 0.3f && std::fabs(in.h_m - pad_ref_) < 3.0f)
                pad_ref_ += (in.h_m - pad_ref_) * std::min(1.0f, dt / 30.0f);
        }
        const float agl = in.h_m - pad_ref_;
        if (flight_started_ && agl > h_max_) h_max_ = agl;

        switch (phase_) {
        case MissionPhase::PAD: {
            // Rocket: accel candidate confirmed by altitude gain
            if (t_launch_cand_ >= 0.0) {
                if (agl >= C.launch_confirm_gain_m) {
                    launch(t, out, "LAUNCH (rocket boost confirmed by altitude gain)");
                } else if (t - t_launch_cand_ > C.launch_confirm_s) {
                    t_launch_cand_ = -1.0;          // a bump on the pad, not a launch
                }
            }
            // Carrier drone (or rocket via baro): sustained climb above the pad
            climb_s_ = (agl >= th.launch_gain && in.v_mps >= th.climb_v) ? climb_s_ + dt : 0.0f;
            if (phase_ == MissionPhase::PAD && climb_s_ >= th.climb_s)
                launch(t, out, lift_test() ? "LAUNCH (lift test: sustained climb)"
                                           : "LAUNCH (sustained climb: carrier drone / rocket)");
            break;
        }
        case MissionPhase::ASCENT: {
            const bool below_peak = agl <= h_max_ - th.release_drop;
            sink_s_ = (below_peak && in.v_mps <= -th.sink_v) ? sink_s_ + dt : 0.0f;
            fall_s_ = (agl <= h_max_ - 20.0f && in.v_mps <= -(float)C.release_fallback_mps) ? fall_s_ + dt : 0.0f;
            const bool evidence = t_evidence_ >= 0.0 && (t - t_evidence_) <= C.release_evidence_s;
            if (sink_s_ >= th.sink_s && (evidence || !th.need_evidence)) {
                release(t, out, lift_test() ? "RELEASED (lift test: sustained descent)"
                                            : "RELEASED (drop / ejection signature + descent)");
            } else if (fall_s_ >= C.release_fallback_s) {
                release(t, out, "RELEASED (sustained canopy descent)");
            }
            break;
        }
        case MissionPhase::DESCENT: {
            if (agl <= th.deploy && (t - t_release_) >= C.arms_min_after_release_s) {
                arms_unlatched_ = true;
                t_arms_ = t;
                set_phase(MissionPhase::ARMS_DEPLOY);
                out.event = "ARMS UNLATCHED (servos released at deploy altitude)";
            }
            break;
        }
        case MissionPhase::ARMS_DEPLOY: {
            const bool opened  = (t - t_arms_) >= C.arms_open_time_s;
            const bool sane    = in.attitude_ok && in.tilt_deg <= C.steer_max_tilt_deg &&
                                 in.rate_rps <= C.steer_max_rate_rps;
            const bool high    = agl > th.cutoff + (lift_test() ? 1.0f : 5.0f);
            if (opened && sane && high && !motors_cut_low_) {
                set_phase(MissionPhase::STEERING);
                t_steer_ = t;
                out.event = lift_test() ? "STEERING (lift test: MOTORS INHIBITED)"
                                        : "STEERING (motors armed, returning to launch site)";
            } else if (opened && !warned_arm_ && (t - t_arms_) > C.steer_arm_timeout_s) {
                warned_arm_ = true;
                out.event = "STEERING HELD (attitude unsafe or too low) - passive descent";
            }
            break;
        }
        case MissionPhase::STEERING: {
            if (!motors_cut_low_ && agl <= th.cutoff) {
                motors_cut_low_ = true;
                out.event = "MOTORS CUT (below cut-off altitude)";
            }
            // Tumble failsafe with hysteresis
            if (!suspended_) {
                tumble_s_ = (!in.attitude_ok || in.tilt_deg > C.tumble_tilt_deg) ? tumble_s_ + dt : 0.0f;
                if (tumble_s_ >= C.tumble_s) {
                    suspended_ = true; recover_s_ = 0.0f;
                    out.event = "MOTORS SUSPENDED (tumbling / attitude invalid)";
                }
            } else {
                recover_s_ = (in.attitude_ok && in.tilt_deg < C.recover_tilt_deg) ? recover_s_ + dt : 0.0f;
                if (recover_s_ >= C.recover_s) {
                    suspended_ = false; tumble_s_ = 0.0f;
                    out.event = "MOTORS RESUMED (attitude recovered)";
                }
            }
            break;
        }
        case MissionPhase::LANDED:
            break;
        }

        // Landed detection: any phase after release
        if (phase_ >= MissionPhase::DESCENT && phase_ != MissionPhase::LANDED) {
            landed_s_ = (agl <= th.landed_alt && std::fabs(in.v_mps) <= th.landed_v)
                        ? landed_s_ + dt : 0.0f;
            if (landed_s_ >= th.landed_s) {
                set_phase(MissionPhase::LANDED);
                out.event = "LANDED";
            }
        }

        out.phase            = phase_;
        out.agl_m            = agl;
        out.h_max_m          = h_max_;
        out.flight_started   = flight_started_;
        out.arms_unlatched   = arms_unlatched_;
        out.steering_active  = phase_ == MissionPhase::STEERING && !motors_cut_low_ && !aborted_;
        out.motors_suspended = suspended_;
        out.motors_enabled   = out.steering_active && !suspended_ && !lift_test();   // never in a lift
        return out;
    }

    /// Operator abort: motors off for good (servos/arms unaffected)
    void abort() noexcept { aborted_ = true; motors_cut_low_ = true; }

    MissionPhase phase() const noexcept { return phase_; }
    bool flight_started() const noexcept { return flight_started_; }

    // ---- persistence ------------------------------------------------------
    void save(MissionPersist& p) const noexcept {
        p.phase          = (uint8_t)phase_;
        p.arms_unlatched = arms_unlatched_;
        p.motors_cut_low = motors_cut_low_;
        p.pad_ref_m      = pad_ref_;
        p.h_max_m        = h_max_;
        p.lift_deploy_m  = lift_deploy_;
    }

    /// Resume after an in-flight reset. Time gates are treated as already satisfied.
    void restore(const MissionPersist& p, double t_now) noexcept {
        phase_          = (MissionPhase)p.phase;
        arms_unlatched_ = p.arms_unlatched;
        motors_cut_low_ = p.motors_cut_low;
        pad_ref_        = p.pad_ref_m; pad_ref_init_ = true;
        h_max_          = p.h_max_m;
        lift_deploy_    = p.lift_deploy_m > 0.0f ? p.lift_deploy_m : 0.0f;
        flight_started_ = phase_ != MissionPhase::PAD;
        t_launch_ = t_release_ = t_arms_ = t_steer_ = t_now - 1000.0;
        t_evidence_ = -1.0;
        // A reset while steering: re-qualify attitude before spinning motors again
        if (phase_ == MissionPhase::STEERING) phase_ = MissionPhase::ARMS_DEPLOY;
    }

private:
    struct Thresholds {
        float launch_gain, climb_v, climb_s;
        float release_drop, sink_v, sink_s;
        bool  need_evidence;
        float deploy, cutoff;
        float landed_alt, landed_v, landed_s;
    };

    Thresholds thresholds() const noexcept {
        const FlightProfileConfig& C = PROFILE_CFG;
        if (lift_test())   // building scale: a residential lift moves ~1-2 m/s
            return { 4.0f, 0.4f, 1.5f,  1.5f, 0.4f, 1.5f, false,  lift_deploy_, 3.0f,  1.5f, 0.3f, 3.0f };
        return { sim_ ? 5.0f : (float)C.launch_alt_gain_m, (float)C.launch_climb_mps, (float)C.launch_climb_s,
                 (float)C.release_drop_m, (float)C.release_descent_mps, (float)C.release_descent_s, !sim_,
                 (float)C.arms_deploy_alt_m, (float)C.motor_cutoff_alt_m,
                 (float)C.landed_alt_m, (float)C.landed_speed_mps, (float)C.landed_s };
    }

    MissionPhase phase_ = MissionPhase::PAD;
    bool   sim_ = false;
    float  lift_deploy_ = 0.0f;
    bool   flight_started_ = false, arms_unlatched_ = false, motors_cut_low_ = false;
    bool   suspended_ = false, aborted_ = false, warned_arm_ = false, pad_ref_init_ = false;
    float  pad_ref_ = 0.0f, h_max_ = 0.0f;
    float  accel_hi_s_ = 0.0f, climb_s_ = 0.0f, sink_s_ = 0.0f, fall_s_ = 0.0f;
    float  freefall_s_ = 0.0f, tumble_s_ = 0.0f, recover_s_ = 0.0f, landed_s_ = 0.0f;
    double last_f_t_ = -1.0, t_launch_cand_ = -1.0, t_evidence_ = -1.0;
    double t_launch_ = 0.0, t_release_ = 0.0, t_arms_ = 0.0, t_steer_ = 0.0;

    void set_phase(MissionPhase p) noexcept {
        if ((int)p > (int)phase_) phase_ = p;   // forward only
    }
    void launch(double t, Outputs& out, const char* why) noexcept {
        flight_started_ = true; t_launch_ = t; h_max_ = 0.0f;
        set_phase(MissionPhase::ASCENT); out.event = why;
    }
    void release(double t, Outputs& out, const char* why) noexcept {
        t_release_ = t; set_phase(MissionPhase::DESCENT); out.event = why;
    }
};

} // namespace nav
