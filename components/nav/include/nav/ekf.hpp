/**
 * @file ekf.hpp
 * @brief Error-State Square-Root EKF with Joseph-form update and health gating.
 *
 * Faithful C++ port of ekf.py.
 *
 * Maintains a 15×15 covariance over the error-state δx defined in nav_state.hpp
 * and applies asynchronous measurement updates from any sensor.
 *
 * Numerical safeguards (doctrine-mandated):
 * 1. Joseph form:  P⁺ = (I−KH)P⁻(I−KH)ᵀ + K·R_eff·Kᵀ
 * 2. Symmetrisation: P ← ½(P + Pᵀ) after every write.
 * 3. PSD jitter: if any diagonal < 0 → add ε·(tr/n)·I.
 * 4. Cholesky-based innovation solve (no full matrix inverse).
 * 5. Bounded execution: O(m³ + n²m) with fixed shapes; no allocation.
 * 6. Health-aware R inflation: R_eff = R / max(H, H_floor).
 */
#pragma once

#include "matrix.hpp"
#include "nav_state.hpp"
#include "config.hpp"
#include <cmath>

namespace nav {

// ---------------------------------------------------------------------------
// Maximum measurement dimension supported (6 for GPS position+velocity)
// ---------------------------------------------------------------------------
constexpr int MAX_MEAS_DIM = 6;

// ---------------------------------------------------------------------------
// UpdateResult — diagnostics from one measurement update
// ---------------------------------------------------------------------------
template<int M>
struct UpdateResult {
    bool   accepted;
    double mahalanobis_sq;
    double innovation[M];
    double normalised_residual[M];   ///< ν_i / √S_ii — for FDIR
    double innovation_cov[M][M];     ///< S = H·P·Hᵀ + R_eff
    double health_used;
    double t_s;
};

// ---------------------------------------------------------------------------
// Measurement model function types
// Use function pointers for the hot path (no std::function overhead).
// h_fn:  predicted measurement from nav state
// H_fn:  (M × 15) Jacobian of measurement w.r.t. error state
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// ErrorStateEKF  — owns NavState and 15×15 covariance P
// ---------------------------------------------------------------------------
class ErrorStateEKF {
public:
    NavState         nav;
    Mat<N_ERR,N_ERR> P;

    // ---- Construction / reset ------------------------------------------

    explicit ErrorStateEKF() noexcept {
        reset_covariance();
        nav.set_attitude(
            ESTIMATOR_CFG.initial_q_w,
            ESTIMATOR_CFG.initial_q_x,
            ESTIMATOR_CFG.initial_q_y,
            ESTIMATOR_CFG.initial_q_z
        );
    }

    void reset(const NavState& initial_nav) noexcept {
        nav = initial_nav;
        reset_covariance();
    }

    void reset_covariance() noexcept {
        P = Mat<N_ERR,N_ERR>::zero();
        for (int i = EIDX_P_0;  i < EIDX_P_END;  ++i) P(i,i) = sq(ESTIMATOR_CFG.pos_init_std_m);
        for (int i = EIDX_V_0;  i < EIDX_V_END;  ++i) P(i,i) = sq(ESTIMATOR_CFG.vel_init_std_mps);
        for (int i = EIDX_TH_0; i < EIDX_TH_END; ++i) P(i,i) = sq(ESTIMATOR_CFG.att_init_std_rad);
        for (int i = EIDX_BA_0; i < EIDX_BA_END; ++i) P(i,i) = sq(ESTIMATOR_CFG.ba_init_std_mps2);
        for (int i = EIDX_BG_0; i < EIDX_BG_END; ++i) P(i,i) = sq(ESTIMATOR_CFG.bg_init_std_radps);
    }

    // ---- Prediction -------------------------------------------------------

    void predict(const Vec<3>& f_b, const Vec<3>& omega_b, double dt,
                 double sigma_a, double sigma_w,
                 double sigma_ba, double sigma_bg) noexcept {
        if (dt <= 0.0) return;

        const Mat<N_ERR,N_ERR> Phi = StrapdownINS::error_state_transition(nav, f_b, omega_b, dt);
        const Mat<N_ERR,N_ERR> Qd  = StrapdownINS::process_noise_cov(nav, sigma_a, sigma_w,
                                                                       sigma_ba, sigma_bg, dt);
        // Mechanise
        StrapdownINS::propagate(nav, f_b, omega_b, dt);

        // P⁺ = Φ·P·Φᵀ + Q
        P = Phi * P * Phi.T() + Qd;
        sanitise_covariance();
    }

    // ---- Generic measurement update (m ≤ MAX_MEAS_DIM) ------------------

    template<int M>
    UpdateResult<M> measurement_update(
            double t_s,
            const double z_arr[M],
            // h(state) → z_hat: fills pred[M]
            void (*h_fn)(const NavState&, double pred[M]),
            // H(state) → Jacobian: fills H_mat[M][N_ERR]
            void (*H_fn)(const NavState&, double H_mat[M][N_ERR]),
            const double R_arr[M][M],
            double health = 1.0,
            double gate_chi2 = -1.0) noexcept {

        UpdateResult<M> res{};
        res.t_s = t_s;

        const double h_floor = ESTIMATOR_CFG.health_floor;
        const double h_eff   = (health > h_floor) ? health : h_floor;
        res.health_used = h_eff;

        // --- Build typed matrix views ------------------------------------
        double z_hat[M];
        h_fn(nav, z_hat);

        double H_raw[M][N_ERR];
        H_fn(nav, H_raw);

        // Innovation  ν = z − ẑ
        double innov[M];
        for (int i = 0; i < M; ++i) {
            innov[i] = z_arr[i] - z_hat[i];
            res.innovation[i] = innov[i];
        }

        // S = H·P·Hᵀ + R_eff  (M×M)
        double S[M][M];
        for (int i = 0; i < M; ++i) {
            for (int j = 0; j < M; ++j) {
                double sum = R_arr[i][j] / h_eff;
                // H·P row i, column j  (then ·Hᵀ)
                for (int k = 0; k < N_ERR; ++k) {
                    double HPk = 0.0;
                    for (int l = 0; l < N_ERR; ++l)
                        HPk += H_raw[i][l] * P(l,k);
                    sum += HPk * H_raw[j][k];
                }
                S[i][j] = (i == j) ? sum : 0.5*(sum); // off-diag averaged below
            }
        }
        // Symmetrise S
        for (int i = 0; i < M; ++i)
            for (int j = i+1; j < M; ++j) {
                double avg = 0.5*(S[i][j] + S[j][i]);
                S[i][j] = S[j][i] = avg;
            }

        // Store S in result
        for (int i = 0; i < M; ++i)
            for (int j = 0; j < M; ++j)
                res.innovation_cov[i][j] = S[i][j];

        // Cholesky of S
        double L[M][M] = {};
        bool chol_ok = cholesky_small<M>(S, L);
        if (!chol_ok) {
            // Jitter S and retry
            for (int i = 0; i < M; ++i) S[i][i] += 1.0e-6 * (S[i][i] + 1.0);
            chol_ok = cholesky_small<M>(S, L);
        }

        // Normalised residual  ν_i / √S_ii
        for (int i = 0; i < M; ++i) {
            double sii = S[i][i]; if (sii < 1.0e-12) sii = 1.0e-12;
            res.normalised_residual[i] = innov[i] / std::sqrt(sii);
        }

        // Mahalanobis² = νᵀ S⁻¹ ν  (via two triangular solves)
        double w[M], y[M];
        forward_sub_small<M>(L, innov, w);
        backward_sub_small<M>(L, w, y);
        double mah_sq = 0.0;
        for (int i = 0; i < M; ++i) mah_sq += innov[i] * y[i];
        res.mahalanobis_sq = mah_sq;

        // Chi² gate (if enabled)
        if (gate_chi2 > 0.0 && mah_sq > gate_chi2) {
            res.accepted = false;
            return res;
        }
        res.accepted = true;

        // Kalman gain K = P·Hᵀ·S⁻¹  (15×M)
        // We solve S·Kᵀ = H·P  ↔  solve Kᵀ from Cholesky of S
        // PHt = P · Hᵀ  (15 × M)
        double PHt[N_ERR][M] = {};
        for (int i = 0; i < N_ERR; ++i)
            for (int j = 0; j < M; ++j) {
                double sum = 0.0;
                for (int k = 0; k < N_ERR; ++k)
                    sum += P(i,k) * H_raw[j][k];
                PHt[i][j] = sum;
            }

        // Solve L·Lᵀ·Kᵀ = Hᵀ·P  per column of PHt
        double K[N_ERR][M] = {};
        for (int col = 0; col < M; ++col) {
            double b_col[M], ww[M], kk[M];
            for (int i = 0; i < M; ++i) b_col[i] = PHt[col][i]; // col-th row of PHt = col of (PHt)ᵀ
            // Wait — K = PHt · S⁻¹ but S is MxM, PHt is NxM.
            // Better: solve for each row of K independently isn't right.
            // K_row_i = PHt[i,:] · S⁻¹ = solve(Sᵀ, PHt[i,:]ᵀ)ᵀ
            // since S is symmetric, Sᵀ = S, so: solve(S, PHt[i,:]) gives K[i,:]
            for (int i = 0; i < M; ++i) b_col[i] = PHt[col][i];
            forward_sub_small<M>(L, b_col, ww);
            backward_sub_small<M>(L, ww, kk);
            for (int i = 0; i < M; ++i) K[col][i] = kk[i];
        }

        // Error state injection: δx = K · ν  (15×1)
        Vec<N_ERR> dx = Vec<N_ERR>::zero();
        for (int i = 0; i < N_ERR; ++i)
            for (int j = 0; j < M; ++j)
                dx(i) += K[i][j] * innov[j];
        inject_error_state(nav, dx);

        // Joseph-form covariance update:
        // P⁺ = (I−KH)·P·(I−KH)ᵀ + K·R_eff·Kᵀ
        // Let  A = I − K·H
        double A[N_ERR][N_ERR] = {};
        for (int i = 0; i < N_ERR; ++i) A[i][i] = 1.0;
        for (int i = 0; i < N_ERR; ++i)
            for (int j = 0; j < N_ERR; ++j)
                for (int m = 0; m < M; ++m)
                    A[i][j] -= K[i][m] * H_raw[m][j];

        // P_tmp = A · P · Aᵀ
        double AP[N_ERR][N_ERR] = {};
        for (int i = 0; i < N_ERR; ++i)
            for (int k = 0; k < N_ERR; ++k) {
                if (A[i][k] == 0.0) continue;
                for (int j = 0; j < N_ERR; ++j)
                    AP[i][j] += A[i][k] * P(k,j);
            }
        double P_new[N_ERR][N_ERR] = {};
        for (int i = 0; i < N_ERR; ++i)
            for (int j = 0; j < N_ERR; ++j) {
                double s = 0.0;
                for (int k = 0; k < N_ERR; ++k)
                    s += AP[i][k] * A[j][k]; // A[j][k] = Aᵀ[k][j]
                P_new[i][j] = s;
            }

        // + K · R_eff · Kᵀ
        const double inv_h = 1.0 / h_eff;
        for (int i = 0; i < N_ERR; ++i)
            for (int j = 0; j < N_ERR; ++j) {
                double s = 0.0;
                for (int m2 = 0; m2 < M; ++m2)
                    for (int n2 = 0; n2 < M; ++n2)
                        s += K[i][m2] * R_arr[m2][n2] * inv_h * K[j][n2];
                P_new[i][j] += s;
            }

        for (int i = 0; i < N_ERR; ++i)
            for (int j = 0; j < N_ERR; ++j)
                P(i,j) = P_new[i][j];

        sanitise_covariance();
        return res;
    }

    // -----------------------------------------------------------------------
    // Numerical hygiene
    // -----------------------------------------------------------------------
    void sanitise_covariance() noexcept {
        P.symmetrise();
        if (P.has_negative_diagonal()) P.jitter_psd(1.0e-6);
        P.clamp_diagonal_max(ESTIMATOR_CFG.cov_ceiling);
    }

    double alt_std() const noexcept { return std::sqrt(std::max(P(2,2), 0.0)); }
    double vel_z_std() const noexcept { return std::sqrt(std::max(P(5,5), 0.0)); }

private:
    static constexpr double sq(double x) noexcept { return x * x; }

    // Small (M≤6) Cholesky — no heap, no templates from matrix.hpp needed here
    template<int M>
    static bool cholesky_small(const double A[M][M], double L[M][M]) noexcept {
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

    template<int M>
    static void forward_sub_small(const double L[M][M],
                                  const double b[M], double x[M]) noexcept {
        for (int i = 0; i < M; ++i) {
            double s = b[i];
            for (int j = 0; j < i; ++j) s -= L[i][j]*x[j];
            x[i] = s / L[i][i];
        }
    }

    template<int M>
    static void backward_sub_small(const double L[M][M],
                                   const double b[M], double x[M]) noexcept {
        for (int i = M-1; i >= 0; --i) {
            double s = b[i];
            for (int j = i+1; j < M; ++j) s -= L[j][i]*x[j];
            x[i] = s / L[i][i];
        }
    }
};

} // namespace nav
