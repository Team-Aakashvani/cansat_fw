// Host verification: VerticalKF + MissionSupervisor + ReturnGuidance + SteerController
#define _USE_MATH_DEFINES
#include "nav/vertical_kf.hpp"
#include "nav/mission.hpp"
#include "nav/guidance.hpp"
#include "control/steer_controller.hpp"
#include <cstdio>
#include <cmath>
#include <random>
#include <vector>
#include <string>
#include <functional>
#include <memory>

using namespace nav;
static int fails = 0;
static void check(bool ok, const char* what, double v = NAN) {
    if (std::isnan(v)) printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    else               printf("  [%s] %-62s %.3f\n", ok ? "PASS" : "FAIL", what, v);
    if (!ok) ++fails;
}

// ---------------------------------------------------------------------------
// Truth profile: returns true altitude (m AGL), vertical accel (m/s^2), and the
// specific force magnitude the IMU feels; 'attached' events modelled per scenario.
// ---------------------------------------------------------------------------
struct Truth { double h, v, a, fmag; };

struct Scenario {
    std::string name;
    double dt = 0.01, T = 0;
    std::function<Truth(double)> truth;
    std::vector<std::pair<double, double>> baro_spikes;    // (t, +metres) single-sample
    std::vector<std::pair<double, double>> baro_pulses;    // (t_start, +metres for 0.4 s)
    bool expect_launch = true, expect_release = true, expect_arms = true;
};

// Integrates a piecewise accel profile to keep h/v/a consistent
struct Kin { double h = 0, v = 0; int stage = 0; double t_mark = 0; };

static Scenario rocket() {
    Scenario s; s.name = "ROCKET: 8 g boost (clips at 4 g) -> coast -> ejection -> chute 6 m/s"; s.T = 160;
    auto st = std::make_shared<Kin>();
    s.truth = [st](double t) {
        const double dt = 0.01;
        double a, fmag;
        const double g = 9.80665;
        int& released = st->stage; double& t_rel = st->t_mark;
        if (t < 5.0) { a = 0; fmag = g; st->h = 0; st->v = 0; st->stage = 0; }
        else if (t < 6.6) { a = 8 * g; fmag = 9 * g; }                 // boost
        else if (!released && st->v > 0) { a = -g - 0.00035 * st->v * st->v; fmag = 0.00035 * st->v * st->v; }
        else {
            if (!released) { released = 1; t_rel = t; }
            const double tr = t - t_rel;
            if (tr < 0.6) { a = -g; fmag = 0.3; }                       // ejected, falling free
            else if (tr < 0.8) { a = 3 * g; fmag = 4 * g; }            // chute snatch shock
            else { const double vt = -6.0; a = (vt - st->v) * 2.0; fmag = g + a; }
        }
        if (released == 2 || (st->h <= 0 && t > 7 && st->v < 0)) {   // on the ground
            released = 2; st->h = 0; st->v = 0; return Truth{0.0, 0.0, 0.0, g};
        }
        st->v += a * dt; st->h += st->v * dt; if (st->h < 0) st->h = 0;
        return Truth{st->h, st->v, a, std::fabs(fmag)};
    };
    s.baro_spikes = {{30.0, 45.0}, {60.0, -38.0}, {90.0, 60.0}};
    s.T = 200;
    s.baro_pulses = {{0.0, 0.0}};
    return s;
}

static Scenario drone_carrier() {
    Scenario s; s.name = "DRONE CARRIER: climb 4 m/s -> hover 20 s -> drop -> chute 6 m/s"; s.T = 380;
    auto st = std::make_shared<Kin>();
    s.truth = [st](double t) {
        const double g = 9.80665, dt = 0.01;
        double a = 0, fmag;
        if (t < 5) { st->h = 0; st->v = 0; st->stage = 0; }
        else if (st->stage == 0) { a = (4.0 - st->v) * 1.5; if (st->h >= 750) { st->stage = 1; st->t_mark = t; } }
        else if (st->stage == 1) { a = -st->v * 1.5; if (t - st->t_mark >= 20) { st->stage = 2; st->t_mark = t; } }
        else {
            const double td = t - st->t_mark;
            if (td < 1.0) { a = -g; fmag = 0.2; st->v += a * dt; st->h += st->v * dt; return Truth{st->h, st->v, a, fmag}; }
            else if (td < 1.25) a = 2.5 * g;
            else a = (-6.0 - st->v) * 2.0;
        }
        fmag = std::fabs(g + a);
        st->v += a * dt; st->h += st->v * dt;
        if (st->h < 0) { st->h = 0; st->v = 0; a = 0; fmag = g; }
        return Truth{st->h, st->v, a, fmag};
    };
    s.baro_spikes = {{100.0, 40.0}, {250.0, 55.0}};
    s.baro_pulses = {{228.5, 25.0}};    // ejection / snatch pressure pulse just after the drop
    return s;
}

static Scenario carrier_abort() {
    Scenario s; s.name = "CARRIER ABORT: climb to 750 m, carrier descends 2.5 m/s with CanSat attached";
    s.T = 520; s.expect_release = false; s.expect_arms = false;
    auto st = std::make_shared<Kin>();
    s.truth = [st](double t) {
        const double g = 9.80665, dt = 0.01;
        double a;
        if (t < 5) { a = 0; st->h = 0; st->v = 0; st->stage = 0; }
        else if (st->h < 750 && st->stage == 0) { a = (4.0 - st->v) * 1.5; }
        else { st->stage = 1; a = (-2.5 - st->v) * 1.0; }
        st->v += a * dt; st->h += st->v * dt;
        if (st->h < 0) { st->h = 0; st->v = 0; a = 0; }
        return Truth{st->h, st->v, a, std::fabs(g + a)};
    };
    return s;
}

static Scenario pad_bumps() {
    Scenario s; s.name = "PAD HANDLING: 5 g knocks, picked up 1.5 m and set down";
    s.T = 60; s.expect_launch = false; s.expect_release = false; s.expect_arms = false;
    s.truth = [](double t) {
        const double g = 9.80665;
        double h = 0, fmag = g;
        if (std::fmod(t, 7.0) < 0.06) fmag = 5 * g;          // knocks
        if (t > 30 && t < 40) h = 1.5 * std::sin((t - 30) / 10 * M_PI);
        return Truth{h, 0, 0, fmag};
    };
    return s;
}

struct RunResult {
    bool launched = false, released = false, arms = false, steering = false, landed = false;
    bool motors_ever = false;
    double arms_true_alt = NAN, max_alt_err = 0, max_alt_err_after_launch = 0, t_land = NAN;
    double motor_cut_alt = NAN;
};

static RunResult run(Scenario& s, unsigned seed, bool verbose, double reset_at = -1, float lift = 0.0f) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> nb(0, 0.25), na(0, 0.15);
    VerticalKF kf; MissionSupervisor ms;
    ms.set_lift_test(lift);
    MissionPersist persist{};
    RunResult r;
    double t_last_baro = -1;
    const char* last_event = nullptr;
    for (double t = 0; t < s.T; t += s.dt) {
        Truth tr = s.truth(t);
        // IMU: world-up specific force minus g, with BNO055 4 g clipping on the measurement
        double f_up = tr.a + 9.80665;
        bool clipped = false;
        if (std::fabs(f_up) > 39.2) { f_up = std::copysign(39.2, f_up); clipped = true; }
        double fmag = std::min(tr.fmag, 39.2 * std::sqrt(3.0));
        if (tr.fmag > 39.2) clipped = true;
        kf.predict((float)(f_up - 9.80665 + na(rng) + 0.08), (float)s.dt, clipped);
        ms.ingest_accel(t, (float)(fmag + na(rng)));

        if (t - t_last_baro >= 0.02 - 1e-9) {
            t_last_baro = t;
            double z = tr.h + nb(rng);
            for (auto& sp : s.baro_spikes) if (std::fabs(t - sp.first) < 0.011) z += sp.second;
            for (auto& pu : s.baro_pulses) if (pu.second != 0 && t >= pu.first && t < pu.first + 0.4) z += pu.second;
            const MissionPhase ph = ms.phase();
            if (ph == MissionPhase::STEERING) kf.set_noise(VERT_CFG.sigma_a_steering, VERT_CFG.baro_sigma_steering_m);
            else if (ph == MissionPhase::ASCENT) kf.set_noise(VERT_CFG.sigma_a_ascent, VERT_CFG.baro_sigma_m);
            else if (ph == MissionPhase::PAD) kf.set_noise(VERT_CFG.sigma_a_pad, VERT_CFG.baro_sigma_m);
            else kf.set_noise(VERT_CFG.sigma_a_descent, VERT_CFG.baro_sigma_m);
            kf.update_baro((float)z, t);
            const auto& o = kf.output();
            const double err = std::fabs(o.h_m - tr.h);
            r.max_alt_err = std::max(r.max_alt_err, err);
            if (ms.flight_started()) r.max_alt_err_after_launch = std::max(r.max_alt_err_after_launch, err);

            MissionSupervisor::Inputs in{t, o.h_m, o.v_mps, 8.0f, 0.5f, true};
            auto out = ms.step(in, 0.02f);
            if (out.event && out.event != last_event) {
                if (verbose) printf("     t=%7.2f  true h=%7.1f  est h=%7.1f  v=%6.2f  %s\n", t, tr.h, o.h_m, o.v_mps, out.event);
                last_event = out.event;
                if (std::string(out.event).find("MOTORS CUT") == 0) r.motor_cut_alt = tr.h;
            }
            if (out.flight_started) r.launched = true;
            if (out.phase >= MissionPhase::DESCENT && out.phase != MissionPhase::LANDED) r.released = true;
            if (out.phase == MissionPhase::LANDED && !r.landed) { r.landed = true; r.t_land = t; if (r.launched) r.released = true; }
            if (out.arms_unlatched && !r.arms) { r.arms = true; r.arms_true_alt = tr.h; }
            if (out.phase == MissionPhase::STEERING) r.steering = true;
            if (out.motors_enabled) r.motors_ever = true;

            // Mid-air reset: persist, rebuild everything, restore
            ms.save(persist); persist.seal();
            if (reset_at > 0 && t >= reset_at && t < reset_at + 0.02) {
                MissionPersist copy = persist;
                ms = MissionSupervisor{}; kf = VerticalKF{};
                if (copy.valid()) ms.restore(copy, t);
                if (verbose) printf("     t=%7.2f  *** SIMULATED BROWNOUT RESET -> restored phase %s ***\n", t, mission_phase_name(ms.phase()));
            }
        }
    }
    return r;
}

static void mission_tests() {
    printf("\n=== Mission + vertical filter: flight scenarios ===\n");
    std::vector<Scenario> scen = { rocket(), drone_carrier(), carrier_abort(), pad_bumps() };
    for (auto& s : scen) {
        printf("\n -- %s\n", s.name.c_str());
        RunResult r = run(s, 1, true);
        check(r.launched == s.expect_launch, s.expect_launch ? "launch detected" : "no false launch");
        check(r.released == s.expect_release, s.expect_release ? "release detected" : "no false release");
        check(r.arms == s.expect_arms, s.expect_arms ? "arms unlatched" : "arms stay LATCHED");
        if (s.expect_arms) {
            check(std::fabs(r.arms_true_alt - 600.0) < 12.0, "arms unlatch true altitude within 600 +/- 12 m", r.arms_true_alt);
            check(r.steering, "steering engaged");
            check(r.motor_cut_alt > 6 && r.motor_cut_alt < 14, "motors cut near 10 m AGL (true alt)", r.motor_cut_alt);
            check(r.landed, "landed detected");
        }
        if (s.expect_launch)
            check(r.max_alt_err_after_launch < 8.0, "max altitude error in flight incl. spikes/clipping (m)", r.max_alt_err_after_launch);

        // Monte Carlo over noise seeds
        int ok = 0; const int N = 30; double worst_arm = 0;
        for (unsigned k = 2; k < 2 + N; ++k) {
            Scenario c = s;
            if (s.name.rfind("ROCKET", 0) == 0) c = rocket();
            else if (s.name.rfind("DRONE", 0) == 0) c = drone_carrier();
            else if (s.name.rfind("CARRIER", 0) == 0) c = carrier_abort();
            else c = pad_bumps();
            RunResult m = run(c, k, false);
            bool good = (m.launched == s.expect_launch) && (m.released == s.expect_release) && (m.arms == s.expect_arms);
            if (s.expect_arms) { good = good && m.landed && std::fabs(m.arms_true_alt - 600) < 12; worst_arm = std::max(worst_arm, std::fabs(m.arms_true_alt - 600)); }
            ok += good;
        }
        check(ok == N, "Monte Carlo: all noise seeds behave identically (count)", ok);
        if (s.expect_arms) check(worst_arm < 12, "Monte Carlo: worst arms-unlatch error vs 600 m (m)", worst_arm);
    }

    printf("\n -- BROWNOUT RESET mid-descent (rocket, reset at t=60 s, while steering)\n");
    Scenario s = rocket();
    RunResult r = run(s, 3, true, 60.0);
    check(r.arms && r.landed, "mission resumes after reset: arms + landing still happen");
}

// ---------------------------------------------------------------------------
// Apartment lift: ~1.5 m/s, gentle accel, door/HVAC pressure transients at each stop
static Scenario lift_ride(double top_m) {
    Scenario s; s.name = "LIFT"; s.T = 10 + top_m / 1.5 + 20 + top_m / 1.5 + 25;
    auto st = std::make_shared<Kin>();
    const double t_up = 10.0, t_top = t_up + top_m / 1.5 + 2.0, t_down = t_top + 18.0;
    s.truth = [st, top_m, t_up, t_down](double t) {
        const double g = 9.80665, dt = 0.01;
        double vt;
        if (t < t_up) vt = 0;
        else if (st->stage == 0) { vt = 1.5; if (st->h >= top_m) st->stage = 1; }
        else if (t < t_down) vt = 0;
        else { vt = st->h > 0.05 ? -1.5 : 0.0; }
        if (t < 1.0) { st->h = 0; st->v = 0; st->stage = 0; }
        const double a = std::max(-1.0, std::min(1.0, (vt - st->v) / 0.5));   // lift accel limit 1 m/s^2
        st->v += a * dt; st->h += st->v * dt;
        if (st->h < 0) { st->h = 0; st->v = 0; }
        return Truth{st->h, st->v, a, std::fabs(g + a)};
    };
    // Door / shaft pressure transients (~1.5 m equivalent, 1 s) at both stops and mid-ride
    s.baro_pulses = {{t_top + 3.0, 1.5}, {t_top + 10.0, -1.2}, {t_up + 8.0, 0.8}, {s.T - 12.0, 1.5}};
    return s;
}

static void lift_tests() {
    printf("\n=== Apartment lift scenarios ===\n");
    struct Case { const char* name; double top; float lift; bool launch, release, arms; };
    const Case cases[] = {
        {"LIFT TEST (deploy 10 m), 10 floors / 30 m", 30.0, 10.0f, true, true, true},
        {"LIFT TEST (deploy 10 m), 4 floors / 12 m",  12.0, 10.0f, true, true, true},
        {"FLIGHT CONFIG, 10 floors / 30 m (lift = carrier descending)", 30.0, 0.0f, true, false, false},
        {"FLIGHT CONFIG, 6 floors / 18 m",            18.0, 0.0f, false, false, false},
    };
    for (const Case& c : cases) {
        printf("\n -- %s\n", c.name);
        int ok = 0; RunResult r0;
        for (unsigned seed = 1; seed <= 20; ++seed) {
            Scenario sc = lift_ride(c.top);
            RunResult r = run(sc, seed, seed == 1, -1, c.lift);
            if (seed == 1) r0 = r;
            bool good = r.launched == c.launch && r.released == c.release && r.arms == c.arms && !r.motors_ever;
            if (c.arms) good = good && std::fabs(r.arms_true_alt - std::min(10.0, c.top - 1.5)) < 3.5 && r.landed;
            ok += good;
        }
        check(r0.launched == c.launch, c.launch ? "launch detected" : "no launch (below threshold)");
        check(r0.released == c.release, c.release ? "release detected" : "no release (arms stay latched)");
        check(r0.arms == c.arms, c.arms ? "arms unlatched" : "arms stay LATCHED");
        if (c.arms) check(std::fabs(r0.arms_true_alt - std::min(10.0, c.top - 1.5)) < 3.5, "arms true altitude (m)", r0.arms_true_alt);
        if (c.arms) check(r0.landed, "landed detected at the ground floor");
        check(!r0.motors_ever, "motors NEVER enabled");
        check(ok == 20, "20 noise seeds identical (count)", ok);
    }
}

// ---------------------------------------------------------------------------
static void controller_tests() {
    printf("\n=== Steering controller: mixer signs ===\n");
    control::SteerController c;
    auto th = c.update(0.2f, 0, 0, 0, 0, 0, 0, 0.35f, 0.01f);
    check(th.m[0] > th.m[1] && th.m[3] > th.m[2], "+tilt_x demand: FL,RL above FR,RR");
    c.reset(); th = c.update(0, 0.2f, 0, 0, 0, 0, 0, 0.35f, 0.01f);
    check(th.m[2] > th.m[1] && th.m[3] > th.m[0], "+tilt_y demand: RR,RL above FL,FR");
    c.reset(); th = c.update(0, 0, 0, 0, 0, 0, 1.0f, 0.35f, 0.01f);
    check(th.m[0] < th.m[1] && th.m[2] < th.m[3], "+yaw rate: CW motors (FL,RR) slow to oppose it");
    c.reset(); th = c.update(0.6f, 0.6f, 0, 0, 0, 0, 0, 0.79f, 0.01f);
    bool in_range = true; for (float m : th.m) in_range &= (m >= ACT_CFG.idle_throttle - 1e-6f && m <= ACT_CFG.max_throttle + 1e-6f);
    check(in_range, "saturation: all outputs within [idle, max]");

    // Rigid-body single-axis attitude sim (rough CanSat inertia) for stability
    printf("\n=== Steering controller: closed-loop tilt step (single axis rigid body) ===\n");
    c.reset();
    const double I = 0.006;            // kg m^2
    const double arm = 0.12, Tmax = 4 * 9.81 * 0.45 / 4;   // per-motor max thrust (N) ~ 0.45 kg total
    double th_x = 0, w = 0, overshoot = 0, settle_err = 0;
    for (int k = 0; k < 400; ++k) {
        auto o = c.update(0.30f, 0, (float)th_x, 0, (float)w, 0, 0, 0.35f, 0.01f);
        const double dF = ((o.m[0] + o.m[3]) - (o.m[1] + o.m[2])) * Tmax;  // left - right
        const double tau = dF * arm / 2 - 0.02 * w - 0.6 * std::sin(th_x) * 0.05 * 9.81; // pendulum restoring
        w += tau / I * 0.01; th_x += w * 0.01;
        overshoot = std::max(overshoot, th_x - 0.30);
        if (k > 300) settle_err = std::max(settle_err, std::fabs(th_x - 0.30));
    }
    check(overshoot < 0.12, "tilt step 17 deg: overshoot (rad)", overshoot);
    check(settle_err < 0.05, "tilt step: steady-state error after 3 s (rad)", settle_err);
}

// ---------------------------------------------------------------------------
// Canopy + CanSat horizontal dynamics: air-relative velocity with drag time constant tau,
// thrust T/m along the TRUE body +Z; guidance believes heading + hdg_err.
static double sim_return(double e0, double n0, double wind_e, double T_over_m, double hdg_err_deg,
                         double& worst, unsigned seed) {
    ReturnGuidance G;
    std::mt19937 rng(seed); std::normal_distribution<double> ng(0, 2.0);
    double e = e0, n = n0, vae = 0, van = 0, ve = wind_e, vn = 0;
    double tilt_x = 0, tilt_y = 0, ge = e, gn = n, gve = ve, gvn = vn, fix_t = 0;
    std::vector<double> he, hn, hve, hvn;
    const double heading = 0.7, tau = 2.0;
    double h = 590; worst = std::hypot(e, n);
    for (double t = 0; h > 10; t += 0.01) {
        h -= 6.0 * 0.01;
        if (std::fmod(t + 1e-9, 1.0) < 0.01) {          // 1 Hz fix delivered 1 s late
            he.push_back(e); hn.push_back(n); hve.push_back(ve); hvn.push_back(vn);
            const size_t k = he.size() >= 2 ? he.size() - 2 : 0;
            ge = he[k] + ng(rng); gn = hn[k] + ng(rng); gve = hve[k]; gvn = hvn[k]; fix_t = t;
        }
        auto o = G.update((float)-ge, (float)-gn, (float)gve, (float)gvn, (float)(t - fix_t),
                          (float)(heading + hdg_err_deg * M_PI / 180.0), 0.01f);
        tilt_x += (o.tilt_x_rad - tilt_x) * 0.01 / 0.3;     // attitude loop ~0.3 s
        tilt_y += (o.tilt_y_rad - tilt_y) * 0.01 / 0.3;
        const double ux = std::sin(tilt_y), uy = -std::sin(tilt_x) * std::cos(tilt_y);
        const double c = std::cos(heading), s2 = std::sin(heading);
        const double the = c * ux - s2 * uy, thn = s2 * ux + c * uy;
        vae += (T_over_m * the - vae / tau) * 0.01;
        van += (T_over_m * thn - van / tau) * 0.01;
        ve = wind_e + vae; vn = van;
        e += ve * 0.01; n += vn * 0.01;
        worst = std::max(worst, std::hypot(e, n));
    }
    return std::hypot(e, n);
}

static void guidance_tests() {
    printf("\n=== Return-to-launch guidance: closed loop under canopy ===\n");
    printf("     canopy descent 6 m/s from 590 m (~97 s), GNSS 1 Hz + 1 s lag + 2 m noise, attitude lag 0.3 s\n");
    const double g = 9.81;
    const double airspeed = 0.7 * g * std::sin(std::atan(GUIDE_CFG.a_max_mps2 / g)) * 2.0;
    printf("     thrust 0.7 W, max tilt %.0f deg, drag tau 2 s -> achievable airspeed ~%.1f m/s\n",
           std::atan(GUIDE_CFG.a_max_mps2 / g) * 57.3, airspeed);
    double worst;
    for (double err : {0.0, 30.0, 60.0}) {
        const double d = sim_return(160, -80, 0.0, 0.7 * g, err, worst, 7);
        char m[120]; snprintf(m, sizeof m, "no wind, 179 m out, heading error %2.0f deg: final distance (m)", err);
        check(d < (err < 45 ? 12.0 : 25.0), m, d);
    }
    {
        const double d = sim_return(150, 60, 2.0, 0.7 * g, 0.0, worst, 8);
        check(d < 25.0, "2 m/s wind (away from site), 162 m out: final distance (m)", d);
    }
    {
        const double d0 = std::hypot(300.0, 120.0);
        const double d = sim_return(300, 120, 3.0, 0.5 * g, 0.0, worst, 9);
        printf("     over-matched: 3 m/s wind vs ~%.1f m/s airspeed at 0.5 W, 323 m out -> %.0f m\n",
               0.5 * g * std::sin(std::atan(GUIDE_CFG.a_max_mps2 / g)) * 2.0, d);
        check(d <= d0 + 10.0, "over-matched by wind: no worse than start (holds ground)", d - d0);
    }
    ReturnGuidance G2; auto o = G2.update(100, 0, 0, 0, 10.0f, 0, 0.01f);
    check(!o.active && o.tilt_x_rad == 0 && o.tilt_y_rad == 0, "stale GNSS fix: no tilt command (level)");
}

int main() {
    mission_tests();
    lift_tests();
    controller_tests();
    guidance_tests();
    printf("\n%s (%d failures)\n", fails ? "SOME TESTS FAILED" : "ALL TESTS PASSED", fails);
    return fails ? 1 : 0;
}
