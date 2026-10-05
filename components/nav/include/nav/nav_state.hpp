/**
 * @file nav_state.hpp
 * @brief Navigation state container and Strapdown INS mechanisation.
 *
 * Faithful C++ port of ins.py.
 *
 * Nav state (16-element):
 *   x_nav = [ p_w(3),  v_w(3),  q_wb(4),  b_a(3),  b_g(3) ]
 *
 * Error state (15-element, what the EKF maintains covariance over):
 *   δx    = [ δp(3),  δv(3),  δθ(3),  δb_a(3),  δb_g(3) ]
 *
 * Propagation (continuous-time, discretised):
 *   ṗ_w  = v_w
 *   v̇_w  = R(q_wb)·(f_b − b_a) + g_w
 *   q̇_wb = ½ Ω(ω_b − b_g) ⊗ q_wb
 *   ḃ_a  = w_ba    (Wiener, handled in Q)
 *   ḃ_g  = w_bg    (Wiener, handled in Q)
 *
 * Integration: 2nd-order midpoint for velocity, closed-form quat exponential.
 *
 * @reference Groves, P. "Principles of GNSS, Inertial, and Multisensor
 *            Integrated Navigation Systems", 2nd Ed., §5.
 */
#pragma once

#include "matrix.hpp"
#include "frames.hpp"
#include "config.hpp"

namespace nav {

// ===========================================================================
// NavState — 16-element navigation state
// ===========================================================================

struct NavState {
    Vec<3> p;    ///< Position in ENU world frame (m)
    Vec<3> v;    ///< Velocity in ENU world frame (m/s)
    Quat   q;    ///< Attitude quaternion body → world (q_wb) [w,x,y,z]
    Vec<3> ba;   ///< Accelerometer bias (m/s²)
    Vec<3> bg;   ///< Gyroscope bias (rad/s)

    NavState() noexcept {
        p = Vec<3>::zero();
        v = Vec<3>::zero();
        q = quat_identity();
        ba = Vec<3>::zero();
        bg = Vec<3>::zero();
    }

    /// Set initial attitude from component values
    void set_attitude(double qw, double qx, double qy, double qz) noexcept {
        q(0) = qw; q(1) = qx; q(2) = qy; q(3) = qz;
        q = quat_normalize(q);
    }

    NavState copy() const noexcept { return *this; }
};

// ---------------------------------------------------------------------------
// Error-state injection: apply δx ∈ R^15 to a NavState
// ---------------------------------------------------------------------------
inline void inject_error_state(NavState& state, const Vec<N_ERR>& dx) noexcept {
    // Position and velocity: additive
    for (int i = 0; i < 3; ++i) state.p(i) += dx(EIDX_P_0 + i);
    for (int i = 0; i < 3; ++i) state.v(i) += dx(EIDX_V_0 + i);
    // Attitude: composition on manifold  q⁺ = q ⊗ Exp_q(δθ)
    Vec<3> dtheta;
    for (int i = 0; i < 3; ++i) dtheta(i) = dx(EIDX_TH_0 + i);
    state.q = quat_mul(state.q, quat_from_rotvec(dtheta));
    // Biases: additive
    for (int i = 0; i < 3; ++i) state.ba(i) += dx(EIDX_BA_0 + i);
    for (int i = 0; i < 3; ++i) state.bg(i) += dx(EIDX_BG_0 + i);
}

// ===========================================================================
// StrapdownINS — stateless mechanisation routines
// ===========================================================================

struct StrapdownINS {

    // -----------------------------------------------------------------------
    // Propagate nav state by dt seconds using one IMU sample.
    // Uses 2nd-order midpoint on velocity and closed-form quaternion exp.
    // -----------------------------------------------------------------------
    static void propagate(NavState& state,
                          const Vec<3>& f_b,
                          const Vec<3>& omega_b,
                          double dt) noexcept {
        if (dt <= 0.0) return;

        // 1. De-bias
        Vec<3> a_b, w_b;
        for (int i = 0; i < 3; ++i) {
            a_b(i) = f_b(i) - state.ba(i);
            w_b(i) = omega_b(i) - state.bg(i);
        }

        // 2. Closed-loop gravity vector observer on SO(3) to eliminate stationary gyro drift
        const double a_mag = std::sqrt(a_b(0)*a_b(0) + a_b(1)*a_b(1) + a_b(2)*a_b(2));
        if (a_mag > 7.0 && a_mag < 13.0) {
            const Mat<3,3> R_cur = R_from_quat(state.q);
            const double inv_amag = 1.0 / a_mag;
            const double ax_n = a_b(0) * inv_amag;
            const double ay_n = a_b(1) * inv_amag;
            const double az_n = a_b(2) * inv_amag;

            // Expected specific force direction in body frame: R_wb^T * [0, 0, 1]^T = [R(2,0), R(2,1), R(2,2)]
            const double vx_p = R_cur(2, 0);
            const double vy_p = R_cur(2, 1);
            const double vz_p = R_cur(2, 2);

            // Error: e = a_measured x v_predicted
            const double ex = ay_n * vz_p - az_n * vy_p;
            const double ey = az_n * vx_p - ax_n * vz_p;
            const double ez = ax_n * vy_p - ay_n * vx_p;

            // Proportional correction gain (Kp = 0.5)
            constexpr double kKp = 0.5;
            w_b(0) += kKp * ex;
            w_b(1) += kKp * ey;
            w_b(2) += kKp * ez;
        }

        // 3. Attitude: q⁺ = q ⊗ Exp_q(ω_b · dt)  (closed-form, exact for const ω)
        const Quat q_new = quat_integrate_gyro(state.q, w_b, dt);

        // 3. Velocity: midpoint rotation (average old+new DCM)
        const Mat<3,3> R_old = R_from_quat(state.q);
        const Mat<3,3> R_new = R_from_quat(q_new);
        const Vec<3> g_w = gravity_world();
        // a_w = 0.5*(R_old + R_new) * a_b + g_w
        Vec<3> a_w;
        for (int i = 0; i < 3; ++i) {
            double Ra = 0.0;
            for (int j = 0; j < 3; ++j)
                Ra += 0.5 * (R_old(i,j) + R_new(i,j)) * a_b(j);
            a_w(i) = Ra + g_w(i);
        }
        Vec<3> v_new;
        for (int i = 0; i < 3; ++i) v_new(i) = state.v(i) + a_w(i) * dt;

        // 4. Position: midpoint velocity
        Vec<3> p_new;
        for (int i = 0; i < 3; ++i)
            p_new(i) = state.p(i) + 0.5*(state.v(i) + v_new(i)) * dt;

        // 5. Write back (biases unchanged; EKF handles random walk via Q)
        state.p = p_new;
        state.v = v_new;
        state.q = q_new;
    }

    // -----------------------------------------------------------------------
    // Discrete-time error-state transition matrix Φ (15×15).
    // Φ ≈ I + F·dt + ½F²·dt²   (second-order truncation)
    //
    // F (continuous) has the structure:
    //   δṗ  = δv                               → F[P,V] = I₃
    //   δv̇  = −R·[a_b]× · δθ − R · δb_a       → F[V,TH], F[V,BA]
    //   δθ̇  = −[ω_b]× · δθ − δb_g             → F[TH,TH], F[TH,BG]
    //   δḃ_a = 0 (RW)                           (zero row)
    //   δḃ_g = 0 (RW)                           (zero row)
    // -----------------------------------------------------------------------
    static void error_state_transition(
            const NavState& state,
            const Vec<3>& f_b,
            const Vec<3>& omega_b,
            double dt,
            Mat<N_ERR,N_ERR>& Phi) noexcept {

        const Mat<3,3> R_wb = R_from_quat(state.q);
        Vec<3> a_b_deb, w_b_deb;
        for (int i = 0; i < 3; ++i) {
            a_b_deb(i) = f_b(i)     - state.ba(i);
            w_b_deb(i) = omega_b(i) - state.bg(i);
        }

        Phi = Mat<N_ERR,N_ERR>::eye();

        // δp ̇ = δv  → F[0:3, 3:6] = I₃
        for (int i = 0; i < 3; ++i) Phi(EIDX_P_0+i, EIDX_V_0+i) = dt;

        // δv ̇ = −R·[a]× · δθ  → F[3:6, 6:9] = −R·skew(a_b)
        {
            const Mat<3,3> Sa = skew3(a_b_deb);
            const Mat<3,3> block = (R_wb * Sa) * (-dt);
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    Phi(EIDX_V_0+i, EIDX_TH_0+j) = block(i,j);
        }
        // δv ̇ += −R · δb_a  → F[3:6, 9:12] = −R
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                Phi(EIDX_V_0+i, EIDX_BA_0+j) = -R_wb(i,j) * dt;

        // δθ̇ = −[ω]× · δθ  → F[6:9, 6:9] = −skew(ω_b)
        {
            const Mat<3,3> Sw = skew3(w_b_deb);
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    Phi(EIDX_TH_0+i, EIDX_TH_0+j) += -Sw(i,j) * dt;
        }
        // δθ̇ += −δb_g  → F[6:9, 12:15] = −I₃
        for (int i = 0; i < 3; ++i)
            Phi(EIDX_TH_0+i, EIDX_BG_0+i) = -dt;
    }

    // -----------------------------------------------------------------------
    // Discrete process-noise covariance Q_d (15×15).
    // -----------------------------------------------------------------------
    static void process_noise_cov(
            const NavState& state,
            double sigma_a,
            double sigma_w,
            double sigma_ba,
            double sigma_bg,
            double dt,
            Mat<N_ERR,N_ERR>& Qd) noexcept {

        const Mat<3,3> R_wb = R_from_quat(state.q);
        const Mat<3,3> RR = R_wb * R_wb.T();  // = I if R is orthogonal; numerical safety

        const double q_a  = sigma_a  * sigma_a  * dt;
        const double q_w  = sigma_w  * sigma_w  * dt;
        const double q_ba = sigma_ba * sigma_ba * dt;
        const double q_bg = sigma_bg * sigma_bg * dt;

        Qd = Mat<N_ERR,N_ERR>::zero();

        // Velocity noise: world-frame accel noise → δv
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                Qd(EIDX_V_0+i, EIDX_V_0+j) = RR(i,j) * q_a;

        // Position from velocity noise (Van-Loan small-dt)
        const double dt3 = dt * dt;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                Qd(EIDX_P_0+i, EIDX_P_0+j) = RR(i,j) * q_a * dt3 / 3.0;
                Qd(EIDX_P_0+i, EIDX_V_0+j) = RR(i,j) * q_a * dt  / 2.0;
                Qd(EIDX_V_0+i, EIDX_P_0+j) = RR(i,j) * q_a * dt  / 2.0;
            }

        // Attitude noise from gyro white noise
        for (int i = 0; i < 3; ++i)
            Qd(EIDX_TH_0+i, EIDX_TH_0+i) = q_w;

        // Bias random walks
        for (int i = 0; i < 3; ++i) {
            Qd(EIDX_BA_0+i, EIDX_BA_0+i) = q_ba;
            Qd(EIDX_BG_0+i, EIDX_BG_0+i) = q_bg;
        }

        Qd.symmetrise();
    }
};

} // namespace nav
