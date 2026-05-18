/**
 * @file imm.hpp
 * @brief Variable-Structure IMM on the shared 15-DOF inertial state.
 *
 * Faithful C++ port of imm.py.
 *
 * The IMM keeps N_REGIMES parallel EKFs that share the same INS mechanisation.
 * Models differ ONLY in process-noise spectral density (σ_a, σ_w).
 * The transition matrix Π_k is gated each step by the supervisor's VS-IMM mask.
 *
 * IMM cycle (Bar-Shalom §11.6):
 *   Step 1: Mixing weights μ_{i|j} = Π_ij·μ_i / c̄_j
 *   Step 2: Mix states (pos/vel/bias linear; attitude on-manifold)
 *   Step 3: Per-model predict (real IMU drives all models)
 *   Step 4: Per-model measurement update (called externally)
 *   Step 5: Re-weight from Gaussian likelihoods + EMA smoothing
 *   Step 6: Fuse (probabilistic weighted mean; spread-of-means covariance)
 *
 * Posterior smoothing (EMA, α=0.35) prevents tick-to-tick mode flipping
 * while remaining mathematically honest (Bayesian update, not a filter).
 *
 * @reference Mazor et al., "Interacting Multiple Model Methods in Target
 *            Tracking: A Survey", IEEE TAES 1998.
 */
#pragma once

#include "ekf.hpp"
#include "nav_state.hpp"
#include "config.hpp"
#include "frames.hpp"
#include <cmath>
#include <algorithm>

namespace nav {

// ---------------------------------------------------------------------------
// IMMOutput — fused estimate from one IMM cycle
// ---------------------------------------------------------------------------
struct IMMOutput {
    NavState     nav;
    Mat<N_ERR,N_ERR> P;
    double       mu[N_REGIMES];     ///< Regime posterior probabilities
    int          most_likely;       ///< argmax(mu)
    double       last_log_lik;
};

// ---------------------------------------------------------------------------
// Per-model measurement storage (for the IMM update step)
// ---------------------------------------------------------------------------
struct ModelUpdateRecord {
    bool   accepted;
    double mahalanobis_sq;
    double innovation_cov[MAX_MEAS_DIM][MAX_MEAS_DIM];
    int    meas_dim;
};

// ---------------------------------------------------------------------------
// IMMFilter
// ---------------------------------------------------------------------------
class IMMFilter {
public:

    ErrorStateEKF models[N_REGIMES];
    double        mu[N_REGIMES];
    double        c_bar[N_REGIMES];   ///< Predicted mode probabilities
    double        last_log_lik;

    IMMFilter() noexcept {
        // Initialise uniform prior
        for (int i = 0; i < N_REGIMES; ++i) {
            mu[i]    = IMM_CFG.initial_probs[i];
            c_bar[i] = IMM_CFG.initial_probs[i];
        }
        last_log_lik = 0.0;
    }

    void set_initial_nav(const NavState& nav) noexcept {
        for (int i = 0; i < N_REGIMES; ++i) models[i].reset(nav);
    }

    /// Concentrate prior mass on one regime (called at supervisor state events).
    void bias_to_regime(int idx, double certainty = 0.85) noexcept {
        if (idx < 0) idx = 0;
        if (idx >= N_REGIMES) idx = N_REGIMES - 1;
        if (certainty < 0.0) certainty = 0.0;
        if (certainty > 1.0) certainty = 1.0;
        const double rest = (1.0 - certainty) / (N_REGIMES - 1);
        for (int i = 0; i < N_REGIMES; ++i) mu[i] = rest;
        mu[idx] = certainty;
        safe_normalise(mu);
    }

    // -----------------------------------------------------------------------
    // IMM predict: mix states, then propagate all models with real IMU
    // -----------------------------------------------------------------------
    void predict(const Vec<3>& f_b, const Vec<3>& omega_b, double dt,
                 const double vs_gate[N_REGIMES][N_REGIMES]) noexcept {
        if (dt <= 0.0) return;

        // Step 1: Build gated transition matrix and mixing weights
        double Pi[N_REGIMES][N_REGIMES];
        build_gated_Pi(vs_gate, Pi);

        // c̄_j = Σ_i Π_ij · μ_i
        for (int j = 0; j < N_REGIMES; ++j) {
            c_bar[j] = 0.0;
            for (int i = 0; i < N_REGIMES; ++i)
                c_bar[j] += Pi[i][j] * mu[i];
            if (c_bar[j] < 1.0e-300) c_bar[j] = 1.0e-300;
        }

        // μ_{i|j} = Π_ij · μ_i / c̄_j
        double mu_ij[N_REGIMES][N_REGIMES];
        for (int j = 0; j < N_REGIMES; ++j)
            for (int i = 0; i < N_REGIMES; ++i)
                mu_ij[i][j] = Pi[i][j] * mu[i] / c_bar[j];

        // Step 2: Mix states for each target model j
        NavState mixed_nav[N_REGIMES];
        Mat<N_ERR,N_ERR> mixed_P[N_REGIMES];
        for (int j = 0; j < N_REGIMES; ++j)
            mix_states(j, mu_ij, mixed_nav[j], mixed_P[j]);

        // Step 3: Per-model predict
        for (int j = 0; j < N_REGIMES; ++j) {
            models[j].nav = mixed_nav[j];
            models[j].P   = mixed_P[j];
            models[j].predict(
                f_b, omega_b, dt,
                IMM_CFG.sigma_a[j], IMM_CFG.sigma_w[j],
                IMM_CFG.sigma_ba,   IMM_CFG.sigma_bg
            );
        }
    }

    // -----------------------------------------------------------------------
    // IMM update: re-weight from per-model measurement likelihoods.
    // Call AFTER applying the same measurement update to all models.
    // records[j].accepted must be set; innovation_cov and mahalanobis_sq filled.
    // -----------------------------------------------------------------------
    void update(const ModelUpdateRecord records[N_REGIMES]) noexcept {
        // Compute per-model log-likelihoods from innovation statistics
        double log_lik[N_REGIMES];
        for (int i = 0; i < N_REGIMES; ++i) {
            if (!records[i].accepted) {
                log_lik[i] = -50.0;
                continue;
            }
            const int M = records[i].meas_dim;
            // log|S| via Cholesky
            double L[MAX_MEAS_DIM][MAX_MEAS_DIM] = {};
            double S_tmp[MAX_MEAS_DIM][MAX_MEAS_DIM];
            for (int r = 0; r < M; ++r)
                for (int c2 = 0; c2 < M; ++c2)
                    S_tmp[r][c2] = records[i].innovation_cov[r][c2];
            bool ok = chol_small_m(S_tmp, L, M);
            if (!ok) { log_lik[i] = -50.0; continue; }
            double logdet = 0.0;
            for (int k = 0; k < M; ++k) logdet += std::log(L[k][k]);
            logdet *= 2.0;
            const double d2 = records[i].mahalanobis_sq;
            log_lik[i] = -0.5 * d2 - 0.5 * (M * std::log(2.0 * PI) + logdet);
        }

        // Numerically stable softmax on (log_lik + log(c̄))
        double log_post[N_REGIMES];
        for (int i = 0; i < N_REGIMES; ++i)
            log_post[i] = log_lik[i] + std::log(c_bar[i]);

        double max_lp = log_post[0];
        for (int i = 1; i < N_REGIMES; ++i) if (log_post[i] > max_lp) max_lp = log_post[i];
        double sum_exp = 0.0;
        double post[N_REGIMES];
        for (int i = 0; i < N_REGIMES; ++i) {
            post[i] = std::exp(log_post[i] - max_lp);
            sum_exp += post[i];
        }
        if (sum_exp < 1.0e-300) sum_exp = 1.0e-300;
        double mu_raw[N_REGIMES];
        for (int i = 0; i < N_REGIMES; ++i) mu_raw[i] = post[i] / sum_exp;

        // EMA smoothing: μ_k = (1−α)·μ_{k-1} + α·μ_raw
        const double alpha = IMM_CFG.mu_alpha;
        for (int i = 0; i < N_REGIMES; ++i)
            mu[i] = (1.0 - alpha) * mu[i] + alpha * mu_raw[i];
        safe_normalise(mu);

        // Overall data log-likelihood
        double ll_sum = 0.0;
        for (int i = 0; i < N_REGIMES; ++i)
            ll_sum += std::exp(log_lik[i]) * mu[i];
        last_log_lik = (ll_sum > 1.0e-300) ? std::log(ll_sum) : -50.0;
    }

    // -----------------------------------------------------------------------
    // IMM fuse: probabilistic weighted mean (spread-of-means covariance)
    // -----------------------------------------------------------------------
    IMMOutput fuse() const noexcept {
        IMMOutput out{};

        // Linear state weighted mean
        Vec<3> p_f = Vec<3>::zero();
        Vec<3> v_f = Vec<3>::zero();
        Vec<3> ba_f = Vec<3>::zero();
        Vec<3> bg_f = Vec<3>::zero();
        for (int i = 0; i < N_REGIMES; ++i) {
            const double w = mu[i];
            for (int k = 0; k < 3; ++k) {
                p_f(k)  += w * models[i].nav.p(k);
                v_f(k)  += w * models[i].nav.v(k);
                ba_f(k) += w * models[i].nav.ba(k);
                bg_f(k) += w * models[i].nav.bg(k);
            }
        }

        // Attitude: iterative on-manifold quaternion mean (Markley et al.)
        // Warm-start from the most likely model's attitude
        int best = 0;
        for (int i = 1; i < N_REGIMES; ++i) if (mu[i] > mu[best]) best = i;
        Quat q_f = models[best].nav.q;
        for (int iter = 0; iter < 4; ++iter) {
            Vec<3> err = Vec<3>::zero();
            for (int i = 0; i < N_REGIMES; ++i) {
                // dq = model[i].q ⊗ q_f*
                const Quat dq = quat_mul(models[i].nav.q, quat_conjugate(q_f));
                const Vec<3> rv = rotvec_from_quat(dq);
                for (int k = 0; k < 3; ++k) err(k) += mu[i] * rv(k);
            }
            q_f = quat_normalize(quat_mul(quat_from_rotvec(err), q_f));
        }

        out.nav.p = p_f; out.nav.v = v_f;
        out.nav.q = q_f;
        out.nav.ba = ba_f; out.nav.bg = bg_f;

        // Spread-of-means covariance correction
        out.P = Mat<N_ERR,N_ERR>::zero();
        for (int i = 0; i < N_REGIMES; ++i) {
            // d = error_between(model[i], fused)
            Vec<N_ERR> d = Vec<N_ERR>::zero();
            for (int k = 0; k < 3; ++k) d(EIDX_P_0+k) = models[i].nav.p(k) - p_f(k);
            for (int k = 0; k < 3; ++k) d(EIDX_V_0+k) = models[i].nav.v(k) - v_f(k);
            {
                const Quat dq = quat_mul(models[i].nav.q, quat_conjugate(out.nav.q));
                const Vec<3> rv = rotvec_from_quat(dq);
                for (int k = 0; k < 3; ++k) d(EIDX_TH_0+k) = rv(k);
            }
            for (int k = 0; k < 3; ++k) d(EIDX_BA_0+k) = models[i].nav.ba(k) - ba_f(k);
            for (int k = 0; k < 3; ++k) d(EIDX_BG_0+k) = models[i].nav.bg(k) - bg_f(k);

            // P_f += μ_i · (P_i + d·dᵀ)
            for (int r = 0; r < N_ERR; ++r)
                for (int c = 0; c < N_ERR; ++c)
                    out.P(r,c) += mu[i] * (models[i].P(r,c) + d(r)*d(c));
        }
        out.P.symmetrise();

        out.most_likely = best;
        out.last_log_lik = last_log_lik;
        for (int i = 0; i < N_REGIMES; ++i) out.mu[i] = mu[i];

        return out;
    }

private:
    // Apply VS-IMM gating and renormalise rows
    static void build_gated_Pi(const double vs_gate[N_REGIMES][N_REGIMES],
                                double Pi_out[N_REGIMES][N_REGIMES]) noexcept {
        for (int i = 0; i < N_REGIMES; ++i) {
            double row_sum = 0.0;
            for (int j = 0; j < N_REGIMES; ++j) {
                Pi_out[i][j] = IMM_CFG.Pi[i][j] * vs_gate[i][j];
                row_sum += Pi_out[i][j];
            }
            if (row_sum < 1.0e-300) {
                // Degenerate row → self-loop
                for (int j = 0; j < N_REGIMES; ++j) Pi_out[i][j] = 0.0;
                Pi_out[i][i] = 1.0;
            } else {
                for (int j = 0; j < N_REGIMES; ++j) Pi_out[i][j] /= row_sum;
            }
        }
    }

    // Mix states for target model j from mixing weights mu_ij[:,j]
    void mix_states(int j, const double mu_ij[N_REGIMES][N_REGIMES],
                    NavState& nav_j, Mat<N_ERR,N_ERR>& P_j) const noexcept {
        // Linear parts
        Vec<3> p{}, v{}, ba{}, bg{};
        for (int i = 0; i < N_REGIMES; ++i) {
            const double w = mu_ij[i][j];
            for (int k = 0; k < 3; ++k) {
                p(k)  += w * models[i].nav.p(k);
                v(k)  += w * models[i].nav.v(k);
                ba(k) += w * models[i].nav.ba(k);
                bg(k) += w * models[i].nav.bg(k);
            }
        }

        // Attitude: 2 iterations of manifold mean, warm-start at model j
        Quat q = models[j].nav.q;
        for (int iter = 0; iter < 2; ++iter) {
            Vec<3> err{};
            for (int i = 0; i < N_REGIMES; ++i) {
                const Quat dq = quat_mul(models[i].nav.q, quat_conjugate(q));
                const Vec<3> rv = rotvec_from_quat(dq);
                for (int k = 0; k < 3; ++k) err(k) += mu_ij[i][j] * rv(k);
            }
            q = quat_normalize(quat_mul(quat_from_rotvec(err), q));
        }

        nav_j.p = p; nav_j.v = v; nav_j.q = q; nav_j.ba = ba; nav_j.bg = bg;

        // Covariance with spread-of-means correction
        P_j = Mat<N_ERR,N_ERR>::zero();
        for (int i = 0; i < N_REGIMES; ++i) {
            Vec<N_ERR> d{};
            for (int k = 0; k < 3; ++k) d(EIDX_P_0+k) = models[i].nav.p(k) - p(k);
            for (int k = 0; k < 3; ++k) d(EIDX_V_0+k) = models[i].nav.v(k) - v(k);
            {
                const Quat dq = quat_mul(models[i].nav.q, quat_conjugate(nav_j.q));
                const Vec<3> rv = rotvec_from_quat(dq);
                for (int k = 0; k < 3; ++k) d(EIDX_TH_0+k) = rv(k);
            }
            for (int k = 0; k < 3; ++k) d(EIDX_BA_0+k) = models[i].nav.ba(k) - ba(k);
            for (int k = 0; k < 3; ++k) d(EIDX_BG_0+k) = models[i].nav.bg(k) - bg(k);
            const double w = mu_ij[i][j];
            for (int r = 0; r < N_ERR; ++r)
                for (int c = 0; c < N_ERR; ++c)
                    P_j(r,c) += w * (models[i].P(r,c) + d(r)*d(c));
        }
        P_j.symmetrise();
    }

    static void safe_normalise(double v[N_REGIMES]) noexcept {
        double s = 0.0;
        for (int i = 0; i < N_REGIMES; ++i) s += v[i];
        if (s < 1.0e-300) {
            for (int i = 0; i < N_REGIMES; ++i) v[i] = 1.0 / N_REGIMES;
        } else {
            for (int i = 0; i < N_REGIMES; ++i) v[i] /= s;
        }
    }

    // Variable-size Cholesky for M up to MAX_MEAS_DIM
    static bool chol_small_m(const double A[MAX_MEAS_DIM][MAX_MEAS_DIM],
                              double L[MAX_MEAS_DIM][MAX_MEAS_DIM],
                              int M) noexcept {
        for (int j = 0; j < M; ++j) {
            double s = A[j][j];
            for (int k = 0; k < j; ++k) s -= L[j][k]*L[j][k];
            if (s <= 0.0) return false;
            L[j][j] = std::sqrt(s);
            double inv = 1.0 / L[j][j];
            for (int i = j+1; i < M; ++i) {
                double s2 = A[i][j];
                for (int k = 0; k < j; ++k) s2 -= L[i][k]*L[j][k];
                L[i][j] = s2 * inv;
            }
        }
        return true;
    }
};

} // namespace nav
