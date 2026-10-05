/**
 * @file imu_attitude.cpp
 * @brief Singularity-free, mount-agnostic attitude reference implementation.
 */
#include "drivers/imu_attitude.hpp"
#include "esp_log.h"
#include <cmath>
#include <algorithm>

static const char* TAG = "AttitudeRef";

namespace drivers {

namespace {

constexpr double S2         = 0.70710678118654752440; // sqrt(0.5)
constexpr double RAD_TO_DEG = 180.0 / 3.14159265358979323846;

inline Quat q_mul(const Quat& a, const Quat& b) noexcept {
    return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    };
}

inline Quat q_conj(const Quat& q) noexcept {
    return { q.w, -q.x, -q.y, -q.z };
}

inline Quat q_norm(const Quat& q) noexcept {
    double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n < 1e-12) return { 1.0, 0.0, 0.0, 0.0 };
    double inv = 1.0 / n;
    return { q.w * inv, q.x * inv, q.y * inv, q.z * inv };
}

/// v_out = R(q) v  (q unit)
inline void q_rotate(const Quat& q, const double v[3], double out[3]) noexcept {
    const double w = q.w, x = q.x, y = q.y, z = q.z;
    out[0] = (1.0 - 2.0*(y*y + z*z)) * v[0] + 2.0*(x*y - w*z) * v[1] + 2.0*(x*z + w*y) * v[2];
    out[1] = 2.0*(x*y + w*z) * v[0] + (1.0 - 2.0*(x*x + z*z)) * v[1] + 2.0*(y*z - w*x) * v[2];
    out[2] = 2.0*(x*z - w*y) * v[0] + 2.0*(y*z + w*x) * v[1] + (1.0 - 2.0*(x*x + y*y)) * v[2];
}

/// World up (+Z) expressed in the sensor frame for a body->world quaternion: R(q)^T e_z.
inline void fused_up(const Quat& q, double up[3]) noexcept {
    up[0] = 2.0 * (q.x * q.z - q.w * q.y);
    up[1] = 2.0 * (q.y * q.z + q.w * q.x);
    up[2] = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
}

/// Shortest-arc rotation taking unit vector a onto unit vector b (a.b > -1 assumed).
inline Quat shortest_arc(const double a[3], const double b[3]) noexcept {
    const double d = a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
    return q_norm({ 1.0 + d,
                    a[1]*b[2] - a[2]*b[1],
                    a[2]*b[0] - a[0]*b[2],
                    a[0]*b[1] - a[1]*b[0] });
}

/**
 * @brief Get mount quaternion q_m for a given mount class.
 * Maps vehicle coordinates to sensor coordinates so vehicle +Z is always the upward axis.
 */
Quat get_mount_quat(MountClass m) noexcept {
    switch (m) {
        case MountClass::Z_UP:   return { 1.0, 0.0, 0.0, 0.0 };
        case MountClass::Z_DOWN: return { 0.0, 1.0, 0.0, 0.0 };
        case MountClass::Y_UP:   return { S2, -S2, 0.0, 0.0 };
        case MountClass::Y_DOWN: return { S2,  S2, 0.0, 0.0 };
        case MountClass::X_UP:   return { S2, 0.0,  S2, 0.0 };
        case MountClass::X_DOWN: return { S2, 0.0, -S2, 0.0 };
        default:                 return { 1.0, 0.0, 0.0, 0.0 };
    }
}

/**
 * @brief Classify orientation from gravity reaction vector (points UP when stationary).
 * @param min_cos Dominant axis must be at least this aligned with gravity (0 = always classify).
 */
MountClass classify_mount(const double grav[3], double min_cos = 0.0) noexcept {
    double n = std::sqrt(grav[0] * grav[0] + grav[1] * grav[1] + grav[2] * grav[2]);
    if (n < 1.0) return MountClass::UNKNOWN;

    double ux = grav[0] / n;
    double uy = grav[1] / n;
    double uz = grav[2] / n;

    double ax = std::abs(ux);
    double ay = std::abs(uy);
    double az = std::abs(uz);

    if (az >= ax && az >= ay) {
        if (az < min_cos) return MountClass::UNKNOWN;
        return (uz > 0) ? MountClass::Z_UP : MountClass::Z_DOWN;
    } else if (ay >= ax && ay >= az) {
        if (ay < min_cos) return MountClass::UNKNOWN;
        return (uy > 0) ? MountClass::Y_UP : MountClass::Y_DOWN;
    } else {
        if (ax < min_cos) return MountClass::UNKNOWN;
        return (ux > 0) ? MountClass::X_UP : MountClass::X_DOWN;
    }
}

} // anonymous namespace

const char* mount_class_name(MountClass m) noexcept {
    switch (m) {
        case MountClass::Z_UP:   return "PARALLEL (Z_UP, chip top up)";
        case MountClass::Z_DOWN: return "PARALLEL (Z_DOWN, inverted)";
        case MountClass::Y_UP:   return "PERPENDICULAR (Y_UP)";
        case MountClass::Y_DOWN: return "PERPENDICULAR (Y_DOWN)";
        case MountClass::X_UP:   return "PERPENDICULAR (X_UP)";
        case MountClass::X_DOWN: return "PERPENDICULAR (X_DOWN)";
        default:                 return "UNKNOWN";
    }
}

bool mount_is_perpendicular(MountClass m) noexcept {
    return (m == MountClass::X_UP || m == MountClass::X_DOWN ||
            m == MountClass::Y_UP || m == MountClass::Y_DOWN);
}

/**
 * @brief Decompose a vehicle quaternion into intrinsic Z-X-Y Euler angles (degrees).
 * Order: heading (Z), then tilt about X, then tilt about Y — matches Three.js 'ZXY'.
 *
 * Near tilt_x = ±90 deg only the sum/difference of heading and tilt_y is observable.
 * There the last well-defined tilt_y is held and heading takes the remainder, so the
 * rebuilt rotation is still exact and the numbers stay continuous.
 */
void AttitudeReference::quat_to_euler_zxy(const Quat& q, double& tilt_x_deg,
                                          double& tilt_y_deg, double& rot_z_deg) noexcept {
    const double w = q.w, x = q.x, y = q.y, z = q.z;

    // R21 = 2*(y*z + w*x) = sin(tilt_x)
    double m32 = 2.0 * (y * z + w * x);
    m32 = std::max(-1.0, std::min(1.0, m32));
    tilt_x_deg = std::asin(m32) * RAD_TO_DEG;

    if (std::abs(m32) < GIMBAL_SIN) {
        // -R20 = -2*(x*z - w*y), R22 = 1 - 2*(x*x + y*y)
        tilt_y_deg = std::atan2(-2.0 * (x * z - w * y), 1.0 - 2.0 * (x * x + y * y)) * RAD_TO_DEG;
        // -R01 = -2*(x*y - w*z), R11 = 1 - 2*(x*x + z*z)
        rot_z_deg  = std::atan2(-2.0 * (x * y - w * z), 1.0 - 2.0 * (x * x + z * z)) * RAD_TO_DEG;
        roll_hold_deg_ = tilt_y_deg;
    } else {
        // Gimbal lock: roll and heading are no longer separable. Hold roll and solve
        // heading exactly from Rz(yaw) = R * Ry(-roll) * Rx(-tilt_x).
        tilt_y_deg = roll_hold_deg_;
        const double hr = -0.5 * tilt_y_deg / RAD_TO_DEG;
        const double hp = -0.5 * std::asin(m32);
        const Quat qz = q_mul(q_mul(q, Quat{ std::cos(hr), 0.0, std::sin(hr), 0.0 }),
                              Quat{ std::cos(hp), std::sin(hp), 0.0, 0.0 });
        rot_z_deg = 2.0 * std::atan2(qz.z, qz.w) * RAD_TO_DEG;
    }

    rot_z_deg = std::fmod(rot_z_deg, 360.0);
    if (rot_z_deg < 0.0) rot_z_deg += 360.0;
    if (rot_z_deg >= 359.995) rot_z_deg = 0.0;   // -0.0001 would otherwise print as "360.00"
}

void AttitudeReference::capture(const Quat& q_unit, const double grav[3], MountClass forced_class) noexcept {
    MountClass mc = (forced_class != MountClass::UNKNOWN) ? forced_class : classify_mount(grav);
    if (mc == MountClass::UNKNOWN) mc = MountClass::Z_UP;
    const Quat q_snap = get_mount_quat(mc);

    // Up direction in the sensor frame. Prefer the fused estimate (self-consistent with the
    // quaternion, so the reference attitude is exactly level); fall back to the accelerometer
    // when the fusion disagrees with it (not converged / wrong convention not yet resolved).
    double up[3];
    fused_up(q_unit, up);
    const double gn = std::sqrt(grav[0]*grav[0] + grav[1]*grav[1] + grav[2]*grav[2]);
    bool used_fused = true;
    if (gn > 1.0) {
        const double g_u[3] = { grav[0]/gn, grav[1]/gn, grav[2]/gn };
        if (up[0]*g_u[0] + up[1]*g_u[1] + up[2]*g_u[2] < CONVERGED_COS) {
            up[0] = g_u[0]; up[1] = g_u[1]; up[2] = g_u[2];
            used_fused = false;
        }
    }

    // Level trim: absorb the residual angle between the snapped axis and true up.
    static constexpr double EZ[3] = { 0.0, 0.0, 1.0 };
    double a_s[3];
    q_rotate(q_snap, EZ, a_s);
    const double c = std::max(-1.0, std::min(1.0, a_s[0]*up[0] + a_s[1]*up[1] + a_s[2]*up[2]));
    const double trim_deg = std::acos(c) * RAD_TO_DEG;
    if (trim_deg <= LEVEL_TRIM_MAX_DEG) {
        q_m_ = q_norm(q_mul(shortest_arc(a_s, up), q_snap));
    } else {
        q_m_ = q_snap;
        ESP_LOGW(TAG, "Vehicle tilted %.1f deg from the nearest mount axis at reference; "
                      "not trimming (output will show the real tilt)", trim_deg);
    }

    // Heading tare: yaw of the vehicle X axis projected on the horizontal plane. The vehicle
    // is within 45 deg of upright here, so the projection is never degenerate.
    static constexpr double EX[3] = { 1.0, 0.0, 0.0 };
    double xw[3];
    q_rotate(q_mul(q_unit, q_m_), EX, xw);
    const double psi = std::atan2(xw[1], xw[0]);
    q_yaw0_ = { std::cos(0.5 * psi), 0.0, 0.0, std::sin(0.5 * psi) };

    q_out_prev_    = {};
    roll_hold_deg_ = 0.0;
    mount_.store(mc);
    referenced_.store(true);
    ref_count_.fetch_add(1);

    ESP_LOGI(TAG, "Orientation identified & referenced: %s, level trim %.2f deg (%s), "
                  "heading %.1f deg, convention %s (ref_count=%lu)",
             mount_class_name(mc), trim_deg, used_fused ? "fused" : "accel",
             psi * RAD_TO_DEG, conj_.load() ? "WORLD->BODY" : "BODY->WORLD",
             (unsigned long)ref_count_.load());
}

AttitudeOutput AttitudeReference::update(const Quat& q_raw, const double grav[3],
                                         const double gyro[3], double dt_s) noexcept {
    AttitudeOutput out{};
    Quat q_in = q_norm(q_raw);

    // Check stationarity: low angular velocity
    double gyro_mag = std::sqrt(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
    bool is_still = (gyro_mag < STILL_GYRO_RAD_S);

    if (is_still) {
        still_s_ += dt_s;
    } else {
        still_s_ = 0.0;
        cand_s_ = 0.0;
    }

    const double gn = std::sqrt(grav[0]*grav[0] + grav[1]*grav[1] + grav[2]*grav[2]);
    const bool   g_ok = (gn > 7.0 && gn < 12.5);

    // ---- Quaternion convention check (body->world vs world->body) --------
    // At rest the accelerometer reads the gravity reaction (world +Z) in the
    // sensor frame. For a body->world quaternion that vector equals the third
    // row of R(q); for the conjugate convention it is the third row of R(q*).
    // Whichever fits better wins, after consistent evidence for CONJ_CONFIRM_S.
    // (A pure heading offset gives no evidence; any tilt — e.g. the
    //  perpendicular bench mount — gives strong evidence.)
    bool conv_changed = false;
    if (is_still && g_ok) {
        const double ux = grav[0]/gn, uy = grav[1]/gn, uz = grav[2]/gn;
        const double w = q_in.w, x = q_in.x, y = q_in.y, z = q_in.z;
        const double r22 = 1.0 - 2.0 * (x*x + y*y);
        const double p0x = 2.0*(x*z - w*y), p0y = 2.0*(y*z + w*x);   // R(q)  row 2
        const double p1x = 2.0*(x*z + w*y), p1y = 2.0*(y*z - w*x);   // R(q*) row 2
        const double e0 = std::sqrt((ux-p0x)*(ux-p0x) + (uy-p0y)*(uy-p0y) + (uz-r22)*(uz-r22));
        const double e1 = std::sqrt((ux-p1x)*(ux-p1x) + (uy-p1y)*(uy-p1y) + (uz-r22)*(uz-r22));
        if (std::abs(e0 - e1) > CONJ_EVIDENCE_MIN) {
            const bool want_conj = (e1 < e0);
            if (want_conj != conj_.load()) {
                conj_vote_s_ += dt_s;
                if (conj_vote_s_ >= CONJ_CONFIRM_S) {
                    conj_.store(want_conj);
                    conj_confirmed_.store(true);
                    conj_vote_s_ = 0.0;
                    conv_changed = true;
                    ESP_LOGW(TAG, "Quaternion convention resolved: %s",
                             want_conj ? "WORLD->BODY (conjugating)" : "BODY->WORLD");
                }
            } else {
                conj_vote_s_ = 0.0;
                conj_confirmed_.store(true);
            }
        }
    }
    const Quat q_unit = conj_.load() ? q_conj(q_in) : q_in;

    // Fusion converged == its gravity direction agrees with the accelerometer at rest
    bool converged = false;
    if (g_ok) {
        double up[3];
        fused_up(q_unit, up);
        converged = (up[0]*grav[0] + up[1]*grav[1] + up[2]*grav[2]) / gn >= CONVERGED_COS;
    }

    // Manual tare requested (or convention flipped: old reference is in the wrong convention)
    if (tare_req_.exchange(false) || (conv_changed && referenced_.load())) {
        capture(q_unit, grav, MountClass::UNKNOWN);
        still_s_ = 0.0;
    }

    // Initial auto-referencing once the board settles on the bench / pad and the
    // fusion has converged (never reference a quaternion that is still swinging in)
    if (!referenced_.load()) {
        if (is_still && ((still_s_ >= INITIAL_STILL_S && converged) || still_s_ >= FALLBACK_STILL_S)) {
            capture(q_unit, grav, MountClass::UNKNOWN);
        } else {
            out.referenced = false;
            return out;
        }
    }

    // Optional auto-remount detection: board held still in a different mount class
    if (auto_remount_.load() && is_still && still_s_ >= 1.0) {
        MountClass current_detected = classify_mount(grav, REMOUNT_AXIS_COS);
        if (current_detected != MountClass::UNKNOWN && current_detected != mount_.load()) {
            if (current_detected == cand_) {
                cand_s_ += dt_s;
                if (cand_s_ >= REMOUNT_STILL_S) {
                    ESP_LOGI(TAG, "Auto-remount detected: switching from %s to %s",
                             mount_class_name(mount_.load()), mount_class_name(current_detected));
                    capture(q_unit, grav, current_detected);
                    cand_s_ = 0.0;
                }
            } else {
                cand_ = current_detected;
                cand_s_ = 0.0;
            }
        } else {
            cand_ = MountClass::UNKNOWN;
            cand_s_ = 0.0;
        }
    }

    // Vehicle attitude against true gravity, heading zeroed at the reference:
    //   q_vw  = q_sensor (x) q_m        (vehicle -> world)
    //   q_out = q_yaw0* (x) q_vw
    Quat q_out = q_norm(q_mul(q_conj(q_yaw0_), q_mul(q_unit, q_m_)));

    // q and -q are the same rotation; keep the output on one hemisphere so downstream
    // interpolation (GUI slerp, filters) never sees a spurious 360-deg jump.
    if (q_out.w * q_out_prev_.w + q_out.x * q_out_prev_.x +
        q_out.y * q_out_prev_.y + q_out.z * q_out_prev_.z < 0.0) {
        q_out = { -q_out.w, -q_out.x, -q_out.y, -q_out.z };
    }
    q_out_prev_ = q_out;

    quat_to_euler_zxy(q_out, out.tilt_x_deg, out.tilt_y_deg, out.rot_z_deg);
    out.q_rel      = q_out;
    out.mount      = mount_.load();
    out.referenced = true;
    return out;
}

void AttitudeReference::sensor_to_vehicle(const double s[3], double v[3]) const noexcept {
    Quat qc = q_conj(q_m_);
    Quat qv = { 0.0, s[0], s[1], s[2] };
    Quat qr = q_mul(q_mul(qc, qv), q_m_);
    v[0] = qr.x;
    v[1] = qr.y;
    v[2] = qr.z;
}

} // namespace drivers
