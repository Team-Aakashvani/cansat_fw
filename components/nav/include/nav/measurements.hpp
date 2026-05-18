/**
 * @file measurements.hpp
 * @brief Measurement models h(x) and Jacobians H(x) for EKF aiding sensors.
 *
 * Faithful C++ port of measurements.py.
 * All Jacobians are analytically derived — never finite-differenced.
 *
 * Conventions:
 *   Error state δx = [δp(0:3), δv(3:6), δθ(6:9), δba(9:12), δbg(12:15)]
 *
 * Sensor models implemented:
 *   1. Barometer:   z = p_w.z  (altitude AGL, scalar)
 *   2. GNSS/GPS:    z = [p_w; v_w]  (position + velocity, 6-vector)
 *   3. Magnetometer: z = R_wbᵀ · B_w  (body-frame field vector, 3-vector)
 */
#pragma once

#include "nav_state.hpp"
#include "frames.hpp"
#include "config.hpp"

namespace nav {

// ===========================================================================
// 1. Barometer: z = h(x) = p_w.z   (1×1)
// ===========================================================================

inline void h_baro(const NavState& s, double pred[1]) noexcept {
    pred[0] = s.p(2);
}

inline void H_baro(const NavState& /*s*/, double H[1][N_ERR]) noexcept {
    for (int j = 0; j < N_ERR; ++j) H[0][j] = 0.0;
    H[0][EIDX_P_0 + 2] = 1.0;   // ∂h/∂(δp.z) = 1
}

// ===========================================================================
// 2. GNSS: z = [p_w; v_w]   (6×1)
// ===========================================================================

inline void h_gnss(const NavState& s, double pred[6]) noexcept {
    pred[0] = s.p(0); pred[1] = s.p(1); pred[2] = s.p(2);
    pred[3] = s.v(0); pred[4] = s.v(1); pred[5] = s.v(2);
}

inline void H_gnss(const NavState& /*s*/, double H[6][N_ERR]) noexcept {
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < N_ERR; ++j)
            H[i][j] = 0.0;
    // ∂[p_w]/∂δp = I₃
    for (int i = 0; i < 3; ++i) H[i][EIDX_P_0 + i] = 1.0;
    // ∂[v_w]/∂δv = I₃
    for (int i = 0; i < 3; ++i) H[3+i][EIDX_V_0 + i] = 1.0;
}

// ===========================================================================
// 3. Magnetometer: z = R_wbᵀ · B_w   (3×1)
//    Requires the reference field vector B_w (set at initialisation).
// ===========================================================================

struct MagMeasModel {
    double B_w[3];  ///< World-frame reference field (e.g. from WMM model)

    void set_field(double bx, double by, double bz) noexcept {
        B_w[0]=bx; B_w[1]=by; B_w[2]=bz;
    }

    void h(const NavState& s, double pred[3]) const noexcept {
        // B_b = R_wb^T · B_w
        const Mat<3,3> R_bw = R_from_quat(s.q).T();
        Vec<3> Bw; Bw(0)=B_w[0]; Bw(1)=B_w[1]; Bw(2)=B_w[2];
        const Vec<3> B_b = R_bw * Bw;
        pred[0]=B_b(0); pred[1]=B_b(1); pred[2]=B_b(2);
    }

    void H(const NavState& s, double Hmat[3][N_ERR]) const noexcept {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < N_ERR; ++j)
                Hmat[i][j] = 0.0;
        // ∂(R_bwᵀ·B_w)/∂δθ = R_bw · skew(B_w)
        // Error-state attitude Jacobian: ∂(R_bwᵀ·v)/∂δθ = R_bw · [v]×
        // (Sola §6.1.4 or Taylor expansion of R_bw(q⊕δθ))
        Vec<3> Bw; Bw(0)=B_w[0]; Bw(1)=B_w[1]; Bw(2)=B_w[2];
        const Mat<3,3> R_bw = R_from_quat(s.q).T();
        const Mat<3,3> skBw = skew3(Bw);
        const Mat<3,3> block = R_bw * skBw;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                Hmat[i][EIDX_TH_0 + j] = block(i,j);
    }
};

// Static singleton with pre-set India reference field (WMM for ~15°N, 78°E)
// dip = ~19° (magnetic inclination), strength ~42µT
inline MagMeasModel& get_mag_model() noexcept {
    static MagMeasModel model;
    // Default: mid-India, 45µT, inclination 19°, declination -0.5°
    // B_w in ENU: [B*cos(dip)*sin(decl), B*cos(dip)*cos(decl), -B*sin(dip)]
    // Approx for CAN-7USAT competition site (to be updated via telecommand)
    static bool init = false;
    if (!init) {
        const double dip_rad   = 19.0 * PI / 180.0;
        const double decl_rad  = -0.5 * PI / 180.0;
        const double B_uT      = 43.0;
        model.B_w[0] = B_uT * std::cos(dip_rad) * std::sin(decl_rad);
        model.B_w[1] = B_uT * std::cos(dip_rad) * std::cos(decl_rad);
        model.B_w[2] = -B_uT * std::sin(dip_rad);
        init = true;
    }
    return model;
}

// C-style wrapper functions for EKF template use (magnetometer)
inline void h_mag(const NavState& s, double pred[3]) noexcept {
    get_mag_model().h(s, pred);
}
inline void H_mag(const NavState& s, double Hmat[3][N_ERR]) noexcept {
    get_mag_model().H(s, Hmat);
}

} // namespace nav
