/**
 * @file frames.hpp
 * @brief SO(3) algebra and coordinate frame utilities.
 *
 * Faithful C++ port of frames.py.
 * Conventions:
 *   - Quaternion: Hamilton, q = [w, x, y, z], body → world (q_wb).
 *   - World frame: ENU (+East, +North, +Up).
 *   - Gravity: g_w = [0, 0, -9.80665] m/s².
 *
 * All functions are pure, no dynamic allocation, safe for ISR/RT use.
 *
 * @reference Sola, J. "Quaternion kinematics for the error-state Kalman
 *            filter" (2017), §1–§4.
 */
#pragma once

#include "matrix.hpp"
#include "config.hpp"
#include <cmath>
#include <algorithm>

namespace nav {

// ---------------------------------------------------------------------------
// Quaternion type alias (w, x, y, z stored in Vec<4> as [0]=w, [1]=x, ...)
// ---------------------------------------------------------------------------
using Quat = Vec<4>;

// ---------------------------------------------------------------------------
// Gravity vector in ENU world frame (z is up)
// ---------------------------------------------------------------------------
inline Vec<3> gravity_world() noexcept {
    Vec<3> g;
    g(0) = 0.0; g(1) = 0.0; g(2) = -G0_MPS2;
    return g;
}

// ===========================================================================
// Quaternion algebra
// ===========================================================================

/// Normalise a quaternion; returns identity if near-zero.
inline Quat quat_normalize(const Quat& q) noexcept {
    constexpr double EPS = 1.0e-12;
    double n = std::sqrt(q(0)*q(0) + q(1)*q(1) + q(2)*q(2) + q(3)*q(3));
    if (n < EPS) {
        Quat id;
        id(0) = 1.0; id(1) = 0.0; id(2) = 0.0; id(3) = 0.0;
        return id;
    }
    Quat out;
    double inv = 1.0 / n;
    out(0) = q(0)*inv; out(1) = q(1)*inv;
    out(2) = q(2)*inv; out(3) = q(3)*inv;
    return out;
}

/// Identity quaternion [1, 0, 0, 0].
inline Quat quat_identity() noexcept {
    Quat q;
    q(0)=1.0; q(1)=0.0; q(2)=0.0; q(3)=0.0;
    return q;
}

/// Hamilton product: a ⊗ b  (rotation: first b, then a).
inline Quat quat_mul(const Quat& a, const Quat& b) noexcept {
    const double aw=a(0), ax=a(1), ay=a(2), az=a(3);
    const double bw=b(0), bx=b(1), by=b(2), bz=b(3);
    Quat r;
    r(0) = aw*bw - ax*bx - ay*by - az*bz;
    r(1) = aw*bx + ax*bw + ay*bz - az*by;
    r(2) = aw*by - ax*bz + ay*bw + az*bx;
    r(3) = aw*bz + ax*by - ay*bx + az*bw;
    return quat_normalize(r);
}

/// Quaternion conjugate (= inverse for unit quaternion).
inline Quat quat_conjugate(const Quat& q) noexcept {
    Quat r;
    r(0)=q(0); r(1)=-q(1); r(2)=-q(2); r(3)=-q(3);
    return r;
}

/// Rotation matrix body → world  from unit quaternion q_wb.
/// R = (w² - ‖v‖²)I + 2v·vᵀ + 2w[v]×
inline Mat<3,3> R_from_quat(const Quat& q) noexcept {
    const Quat qn = quat_normalize(q);
    const double w=qn(0), x=qn(1), y=qn(2), z=qn(3);
    const double xx=x*x, yy=y*y, zz=z*z;
    const double wx=w*x, wy=w*y, wz=w*z;
    const double xy=x*y, xz=x*z, yz=y*z;
    Mat<3,3> R;
    R(0,0)=1.0-2.0*(yy+zz); R(0,1)=2.0*(xy-wz);     R(0,2)=2.0*(xz+wy);
    R(1,0)=2.0*(xy+wz);     R(1,1)=1.0-2.0*(xx+zz); R(1,2)=2.0*(yz-wx);
    R(2,0)=2.0*(xz-wy);     R(2,1)=2.0*(yz+wx);     R(2,2)=1.0-2.0*(xx+yy);
    return R;
}

/// Exponential map  Exp_q : R³ → S³  (rotation vector → quaternion).
/// Stable Taylor approximation for ‖θ‖ → 0.
inline Quat quat_from_rotvec(const Vec<3>& theta) noexcept {
    const double a = std::sqrt(theta(0)*theta(0) + theta(1)*theta(1) + theta(2)*theta(2));
    Quat q;
    if (a < 1.0e-8) {
        // Second-order accurate near-identity
        q(0) = 1.0;
        q(1) = 0.5*theta(0);
        q(2) = 0.5*theta(1);
        q(3) = 0.5*theta(2);
        return quat_normalize(q);
    }
    const double half = 0.5 * a;
    double s, c;
    if (std::abs(half) < 1.0e-4) {
        s = half - (half*half*half) / 6.0;
        c = 1.0 - (half*half) / 2.0;
    } else {
        s = std::sin(half);
        c = std::cos(half);
    }
    const double scale = s / a;
    q(0) = c;
    q(1) = scale * theta(0);
    q(2) = scale * theta(1);
    q(3) = scale * theta(2);
    return quat_normalize(q);
}

/// Logarithm map  Log_q : S³ → R³  (quaternion → rotation vector).
inline Vec<3> rotvec_from_quat(const Quat& q) noexcept {
    Quat qn = quat_normalize(q);
    if (qn(0) < 0.0) {
        qn(0)=-qn(0); qn(1)=-qn(1); qn(2)=-qn(2); qn(3)=-qn(3);
    }
    const double nv = std::sqrt(qn(1)*qn(1) + qn(2)*qn(2) + qn(3)*qn(3));
    Vec<3> rv;
    if (nv < 1.0e-8) {
        rv(0) = 2.0*qn(1);
        rv(1) = 2.0*qn(2);
        rv(2) = 2.0*qn(3);
        return rv;
    }
    const double angle = 2.0 * std::atan2(nv, qn(0));
    const double scale = angle / nv;
    rv(0) = scale * qn(1);
    rv(1) = scale * qn(2);
    rv(2) = scale * qn(3);
    return rv;
}

/// Closed-form attitude integration assuming constant ω over dt.
/// q⁺ = q ⊗ Exp_q(ω · dt)
inline Quat quat_integrate_gyro(const Quat& q,
                                 const Vec<3>& omega_body,
                                 double dt) noexcept {
    Vec<3> theta;
    theta(0) = omega_body(0)*dt;
    theta(1) = omega_body(1)*dt;
    theta(2) = omega_body(2)*dt;
    return quat_mul(q, quat_from_rotvec(theta));
}

/// Rotate a body-frame vector to world frame: v_w = R_wb · v_b
inline Vec<3> body_to_world(const Vec<3>& v_b, const Quat& q_wb) noexcept {
    return R_from_quat(q_wb) * v_b;
}

/// Rotate a world-frame vector to body frame: v_b = R_wb^T · v_w
inline Vec<3> world_to_body(const Vec<3>& v_w, const Quat& q_wb) noexcept {
    return R_from_quat(q_wb).T() * v_w;
}

// ===========================================================================
// Euler angles (ZYX — yaw/pitch/roll) for telemetry display only
// ===========================================================================
struct EulerAngles {
    double roll_rad;   ///< φ — rotation about x (forward)
    double pitch_rad;  ///< θ — rotation about y (right)
    double yaw_rad;    ///< ψ — rotation about z (up)
};

inline EulerAngles euler_from_quat(const Quat& q) noexcept {
    const Mat<3,3> R = R_from_quat(q);
    EulerAngles e;
    e.pitch_rad = std::asin(-R(2,0));
    e.roll_rad  = std::atan2(R(2,1), R(2,2));
    e.yaw_rad   = std::atan2(R(1,0), R(0,0));
    return e;
}

// ===========================================================================
// ISA barometric model (altitude ↔ pressure)
// ===========================================================================

/// ISA troposphere: pressure [Pa] from altitude [m]
inline double isa_altitude_to_pressure(double alt_m) noexcept {
    constexpr double exponent = G0_MPS2 / (R_AIR_JPKGK * ISA_LAPSE_KPM);
    double ratio = 1.0 - ISA_LAPSE_KPM * alt_m / T0_K;
    if (ratio < 0.0) ratio = 0.001; // floor at ~38km
    return P0_PA * std::pow(ratio, exponent);
}

/// ISA troposphere: altitude [m] from pressure [Pa]
inline double isa_pressure_to_altitude(double p_pa) noexcept {
    constexpr double exponent = R_AIR_JPKGK * ISA_LAPSE_KPM / G0_MPS2;
    double ratio = p_pa / P0_PA;
    if (ratio < 1.0e-6) ratio = 1.0e-6;
    return (T0_K / ISA_LAPSE_KPM) * (1.0 - std::pow(ratio, exponent));
}

/// Linearised barometric noise std: dh/dp ≈ T₀/(g·ρ₀·p₀) at sea level
inline double baro_pressure_to_altitude_sigma(double sigma_p_pa) noexcept {
    // dh/dp = -T₀/(L·p_eff) but at pad conditions ≈ T₀/(g₀·ρ₀·p₀)^-1
    constexpr double dh_dp = T0_K / (G0_MPS2 * P0_PA / R_AIR_JPKGK);
    return sigma_p_pa * dh_dp;
}

} // namespace nav
