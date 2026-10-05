/**
 * @file vertical_kf.hpp
 * @brief Vertical-channel Kalman filter: altitude, vertical speed, accel bias (float).
 *
 * Replaces the 5 x 15-state IMM for everything the mission logic needs (release,
 * 600 m arm unlatch, motor cut-off, landing). Attitude comes from the BNO055
 * fusion quaternion; horizontal state for guidance comes from GNSS.
 *
 *   x = [h, v, b]      h: altitude AGL (m), v: vertical speed (m/s, up +),
 *                      b: bias of the world-up specific-force measurement (m/s^2)
 *   predict: a = a_up_meas - b          (a_up_meas = (R(q) f_body).z - g)
 *            h += v dt + a dt^2/2,  v += a dt
 *   update : barometric altitude (50 Hz), GNSS altitude (weak, ~1 Hz)
 *
 * Spike / fault protection (measurement side):
 *   1. median-of-3 pre-filter on raw baro altitude (kills single-sample glitches)
 *   2. physical rate limit vs. the last accepted sample
 *   3. chi-square innovation gate (NIS) against the predicted uncertainty
 *   4. hold-then-resync: a stream rejected for resync_after_s whose sample-to-sample
 *      noise is nominal means the PREDICTION diverged -> re-sync to the sensor;
 *      an erratic stream stays rejected (the filter coasts on the IMU)
 * Accel side: clipped samples (BNO055 4 g limit) and shocks inflate process noise
 * for that step, so the filter leans on the baro until things settle.
 *
 * Header-only, no ESP-IDF dependencies: unit-tested on the host.
 */
#pragma once

#include "config.hpp"
#include <cmath>
#include <algorithm>

namespace nav {

class VerticalKF {
public:
    struct Output {
        float h_m      = 0.0f;
        float v_mps    = 0.0f;
        float bias     = 0.0f;
        float sigma_h  = 0.0f;
        bool  baro_held = false;   ///< Baro currently rejected (filter coasting on IMU)
        bool  clipped   = false;   ///< Last accel sample saturated
        unsigned baro_rejects = 0; ///< Total rejected baro samples (diagnostics)
        unsigned resyncs      = 0;
    };

    void reset(float h0) noexcept {
        x_[0] = h0; x_[1] = 0.0f; x_[2] = 0.0f;
        for (auto& r : P_) for (auto& c : r) c = 0.0f;
        P_[0][0] = 1.0f; P_[1][1] = 1.0f; P_[2][2] = 0.25f;
        med_n_ = 0; d2_n_ = 0; clip_hold_s_ = 0.0f; last_acc_t_ = -1.0; last_acc_z_ = h0;
        gate_ = Gate{}; initialised_ = true;
        out_.baro_rejects = 0; out_.resyncs = 0;
    }

    bool initialised() const noexcept { return initialised_; }

    /// Phase-scheduled noise (replaces the IMM's per-regime models)
    void set_noise(float sigma_a, float baro_sigma) noexcept {
        sigma_a_ = sigma_a; baro_sigma_ = baro_sigma;
    }

    /// High-rate predict. a_up: world-up specific force minus g (m/s^2).
    void predict(float a_up, float dt, bool clipped) noexcept {
        if (!initialised_ || !(dt > 0.0f) || dt > 0.5f || !std::isfinite(a_up)) return;
        const float a  = a_up - x_[2];
        const float dt2 = dt * dt;
        x_[0] += x_[1] * dt + 0.5f * a * dt2;
        x_[1] += a * dt;

        // P = F P F^T + Q,  F = [[1, dt, -dt^2/2], [0, 1, -dt], [0, 0, 1]]
        const float f02 = -0.5f * dt2, f12 = -dt;
        float FP[3][3];
        for (int j = 0; j < 3; ++j) {
            FP[0][j] = P_[0][j] + dt * P_[1][j] + f02 * P_[2][j];
            FP[1][j] = P_[1][j] + f12 * P_[2][j];
            FP[2][j] = P_[2][j];
        }
        for (int i = 0; i < 3; ++i) {
            P_[i][0] = FP[i][0] + dt * FP[i][1] + f02 * FP[i][2];
            P_[i][1] = FP[i][1] + f12 * FP[i][2];
            P_[i][2] = FP[i][2];
        }
        if (clipped) clip_hold_s_ = 0.5f; else if (clip_hold_s_ > 0.0f) clip_hold_s_ -= dt;
        const float sa = clipped ? std::max(sigma_a_, VERT_CFG.sigma_a_clipped) : sigma_a_;
        const float q = sa * sa;
        const float g0 = 0.5f * dt2, g1 = dt;
        P_[0][0] += g0 * g0 * q; P_[0][1] += g0 * g1 * q;
        P_[1][0] += g0 * g1 * q; P_[1][1] += g1 * g1 * q;
        P_[2][2] += VERT_CFG.sigma_bias * VERT_CFG.sigma_bias * dt;
        symmetrise();
        out_.clipped = clipped;
    }

    /// Barometric altitude update. Returns true if fused.
    bool update_baro(float z_raw, double t_s) noexcept {
        if (!std::isfinite(z_raw)) return false;
        if (!initialised_) { reset(z_raw); last_acc_t_ = t_s; return true; }

        // 1. median-of-3
        med_[med_n_ % 3] = z_raw; med_n_++;
        const float z = (med_n_ >= 3) ? median3(med_[0], med_[1], med_[2]) : z_raw;

        // 2. physical rate limit vs. last accepted sample
        bool plausible = true;
        if (last_acc_t_ >= 0.0) {
            const float dt = (float)(t_s - last_acc_t_);
            if (dt > 0.0f && dt < 5.0f)
                plausible = std::fabs(z - last_acc_z_) <= VERT_CFG.baro_max_rate_mps * dt + 3.0f * baro_sigma_;
        }

        // Sensor's own smoothness: second difference of the raw stream. For any real
        // trajectory a*dt^2 is millimetres, so this measures sensor noise / glitches
        // independently of how wrong the prediction is.
        d2_[d2_n_ % 3] = z_raw; d2_n_++;
        float d2 = 0.0f;
        if (d2_n_ >= 3) {
            const float zk = d2_[(d2_n_ - 1) % 3], zk1 = d2_[(d2_n_ - 2) % 3], zk2 = d2_[(d2_n_ - 3) % 3];
            d2 = (zk - 2.0f * zk1 + zk2) / baro_sigma_;
        }

        // 3. chi-square gate (bypassed while the accel is clipped or just recovered:
        //    the prediction is knowingly wrong then and the baro is the reference)
        const float R = baro_sigma_ * baro_sigma_;
        const float nu = z - x_[0];
        const float S = P_[0][0] + R;
        const float nis = nu * nu / S;
        const bool consistent = plausible && (nis <= VERT_CFG.baro_gate_chi2 || clip_hold_s_ > 0.0f);

        if (!consistent) {
            out_.baro_rejects++;
            if (gate_decide(d2, t_s)) {                           // 4. re-sync
                x_[0] = z;
                P_[0][0] = R + nu * nu;
                P_[1][1] += 4.0f;
                for (int k = 1; k < 3; ++k) { P_[0][k] = P_[k][0] = 0.0f; }
                last_acc_z_ = z; last_acc_t_ = t_s;
                out_.resyncs++;
                return true;
            }
            return false;
        }
        gate_ = Gate{};
        scalar_update(nu, S, R);
        last_acc_z_ = z; last_acc_t_ = t_s;
        return true;
    }

    /// GNSS altitude (already converted to the baro's AGL frame). Weak, gated.
    bool update_gnss_alt(float z) noexcept {
        if (!initialised_ || !std::isfinite(z)) return false;
        const float R = VERT_CFG.gnss_alt_sigma_m * VERT_CFG.gnss_alt_sigma_m;
        const float nu = z - x_[0];
        const float S = P_[0][0] + R;
        if (nu * nu / S > VERT_CFG.gnss_gate_chi2) return false;
        scalar_update(nu, S, R);
        return true;
    }

    const Output& output() noexcept {
        out_.h_m = x_[0]; out_.v_mps = x_[1]; out_.bias = x_[2];
        out_.sigma_h = std::sqrt(std::max(0.0f, P_[0][0]));
        out_.baro_held = gate_.rejecting;
        return out_;
    }

private:
    struct Gate {
        bool   rejecting = false;
        double t_first   = 0.0;
        float  diff_sq   = 0.0f;
        int    n_diff    = 0;
    };

    float  x_[3] = {0.0f, 0.0f, 0.0f};
    float  P_[3][3] = {};
    float  sigma_a_ = VERT_CFG.sigma_a_pad;
    float  baro_sigma_ = VERT_CFG.baro_sigma_m;
    float  med_[3] = {};
    unsigned med_n_ = 0;
    float  d2_[3] = {};
    unsigned d2_n_ = 0;
    float  clip_hold_s_ = 0.0f;
    double last_acc_t_ = -1.0;
    float  last_acc_z_ = 0.0f;
    Gate   gate_{};
    bool   initialised_ = false;
    Output out_{};

    static float median3(float a, float b, float c) noexcept {
        return std::max(std::min(a, b), std::min(std::max(a, b), c));
    }

    void scalar_update(float nu, float S, float R) noexcept {
        // H = [1 0 0]; Joseph form for numerical robustness in float
        const float K[3] = { P_[0][0] / S, P_[1][0] / S, P_[2][0] / S };
        for (int i = 0; i < 3; ++i) x_[i] += K[i] * nu;
        float IKH_P[3][3];
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                IKH_P[i][j] = P_[i][j] - K[i] * P_[0][j];
        float Pn[3][3];
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                Pn[i][j] = IKH_P[i][j] - IKH_P[i][0] * K[j] + K[i] * R * K[j];
        for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) P_[i][j] = Pn[i][j];
        symmetrise();
    }

    void symmetrise() noexcept {
        for (int i = 0; i < 3; ++i) {
            if (P_[i][i] < 1e-6f) P_[i][i] = 1e-6f;
            for (int j = i + 1; j < 3; ++j) {
                const float m = 0.5f * (P_[i][j] + P_[j][i]);
                P_[i][j] = P_[j][i] = m;
            }
        }
    }

    /// Returns true when a rejected-but-smooth stream should re-sync the state.
    /// d2: normalised second difference of the raw baro (white noise -> rms ~ sqrt(6)).
    bool gate_decide(float d2, double t_s) noexcept {
        Gate& g = gate_;
        if (!g.rejecting) { g = Gate{}; g.rejecting = true; g.t_first = t_s; }
        g.diff_sq += d2 * d2; g.n_diff++;
        if ((t_s - g.t_first) >= VERT_CFG.resync_after_s && g.n_diff >= 10) {
            const float rms = std::sqrt(g.diff_sq / (float)g.n_diff);
            if (rms < 6.0f) { g = Gate{}; return true; }       // smooth: prediction diverged
            g.t_first = t_s; g.diff_sq = 0.0f; g.n_diff = 0;   // erratic: keep rejecting
        }
        return false;
    }
};

} // namespace nav
