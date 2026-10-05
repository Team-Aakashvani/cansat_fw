/**
 * @file imu_attitude.hpp
 * @brief Singularity-free, mount-agnostic attitude reference for the fused IMU quaternion.
 *
 * Problem solved
 * --------------
 * The Bosch on-chip Euler output (registers 0x1A..0x1F) is only well-behaved when the
 * sensor lies flat. When the board stands perpendicular to the bench, the resting Bosch
 * roll sits at ~±90 deg: the Euler gimbal-lock point. A 1-2 deg tilt then makes Bosch
 * reflect roll and jump pitch/heading by 180 deg ("random inversions").
 *
 * Approach
 * --------
 *  1. Work only with the fused QUATERNION (registers 0x20..0x27). Quaternions have no
 *     singularities anywhere; all composition is done on quaternions.
 *  2. The quaternion convention (body->world vs world->body) is verified at runtime
 *     against gravity, so a convention mismatch can never mirror the motion.
 *  3. At boot (once the board is still AND the fused gravity agrees with the
 *     accelerometer, i.e. the fusion has converged) the mount is identified from the
 *     gravity vector: which sensor axis points up (Z_UP/Z_DOWN = parallel to bench,
 *     X_/Y_ = perpendicular). A 90-deg "mount" rotation q_m maps that axis onto vehicle
 *     +Z (up). The small residual misalignment (hand-mounted board, sloped bench) is
 *     absorbed into q_m when below LEVEL_TRIM_MAX_DEG ("level calibration").
 *  4. Output is the gravity-referenced VEHICLE attitude with only the heading zeroed:
 *        q_vw  = q_sensor (x) q_m               (vehicle -> world)
 *        q_out = q_yaw0* (x) q_vw               (heading tare, a world-Z rotation)
 *     so tilt is always measured against true gravity and rest reads 0,0,0 for every mount.
 *  5. Euler angles are extracted in intrinsic Z-X-Y order, identical to Three.js 'ZXY'.
 *     At the only remaining Euler singularity (a real ±90 deg tilt about vehicle X) roll
 *     is held and yaw absorbs the combined angle, so the numbers never jump. Consumers that
 *     need exact motion (the 3D viewer) should use the quaternion q_rel instead.
 *
 * The mount is identified at boot and on every tare request (CMD,TARE). Automatic
 * re-mount detection is off by default, because holding the vehicle on its side during
 * bench tilt tests would otherwise be mistaken for a new mount.
 */
#pragma once

#include <atomic>
#include <cstdint>

namespace drivers {

struct Quat {
    double w = 1.0, x = 0.0, y = 0.0, z = 0.0;
};

enum class MountClass : uint8_t {
    UNKNOWN = 0,
    Z_UP,     ///< Sensor flat, chip facing up          (parallel to bench)
    Z_DOWN,   ///< Sensor flat, chip facing down        (parallel to bench)
    X_UP,     ///< Sensor standing, +X axis points up   (perpendicular)
    X_DOWN,   ///< Sensor standing, -X axis points up   (perpendicular)
    Y_UP,     ///< Sensor standing, +Y axis points up   (perpendicular)
    Y_DOWN    ///< Sensor standing, -Y axis points up   (perpendicular)
};

const char* mount_class_name(MountClass m) noexcept;
bool        mount_is_perpendicular(MountClass m) noexcept;

struct AttitudeOutput {
    double     tilt_x_deg = 0.0;  ///< Rotation about vehicle X  [-90, +90]
    double     tilt_y_deg = 0.0;  ///< Rotation about vehicle Y  (-180, +180]
    double     rot_z_deg  = 0.0;  ///< Rotation about vehicle Z (heading) [0, 360)
    Quat       q_rel{};           ///< Vehicle -> heading-tared world rotation (hemisphere-continuous)
    MountClass mount      = MountClass::UNKNOWN;
    bool       referenced = false;
};

class AttitudeReference {
public:
    // Tunables
    static constexpr double STILL_GYRO_RAD_S   = 0.08;  ///< |gyro| below this == stationary (~4.5 deg/s)
    static constexpr double INITIAL_STILL_S    = 0.25;  ///< Still + converged time before first reference
    static constexpr double FALLBACK_STILL_S   = 3.0;   ///< Reference anyway after this long still (fusion never agreed)
    static constexpr double CONVERGED_COS      = 0.9945;///< Fused vs accel gravity within ~6 deg == converged
    static constexpr double LEVEL_TRIM_MAX_DEG = 12.0;  ///< Residual mount misalignment absorbed at reference
    static constexpr double REMOUNT_STILL_S    = 3.0;   ///< Still time in a new mount class before re-reference
    static constexpr double REMOUNT_AXIS_COS   = 0.90;  ///< Gravity must be within ~25 deg of an axis
    static constexpr double CONJ_EVIDENCE_MIN  = 0.30;  ///< Min gravity-fit gap to judge quaternion convention
    static constexpr double CONJ_CONFIRM_S     = 0.5;   ///< Consistent evidence time to change convention
    static constexpr double GIMBAL_SIN         = 0.999999999; ///< |sin(tilt_x)| above this (within ~0.003 deg of 90) == gimbal lock

    /**
     * @param q_raw   Fused quaternion as reported by the sensor (any norm, normalised here)
     * @param grav    Gravity (or low-passed accel) in sensor frame, +g on the axis pointing UP
     * @param gyro    Angular rate in sensor frame [rad/s]
     * @param dt_s    Time since previous call [s]
     */
    AttitudeOutput update(const Quat& q_raw, const double grav[3], const double gyro[3], double dt_s) noexcept;

    /// Transform vector from physical sensor coordinates to vehicle body coordinates
    void sensor_to_vehicle(const double s[3], double v[3]) const noexcept;
    Quat mount_quat() const noexcept { return q_m_; }

    void request_tare() noexcept { tare_req_.store(true); }
    void set_auto_remount(bool en) noexcept { auto_remount_.store(en); }
    bool auto_remount() const noexcept { return auto_remount_.load(); }

    // Diagnostics (read from other tasks; values are informational only)
    MountClass mount() const noexcept { return mount_.load(); }
    bool       conjugated() const noexcept { return conj_.load(); }
    bool       convention_confirmed() const noexcept { return conj_confirmed_.load(); }
    uint32_t   reference_count() const noexcept { return ref_count_.load(); }
    bool       referenced() const noexcept { return referenced_.load(); }

private:
    void capture(const Quat& q_unit, const double grav[3], MountClass forced_class) noexcept;
    void quat_to_euler_zxy(const Quat& q, double& tilt_x_deg, double& tilt_y_deg, double& rot_z_deg) noexcept;

    Quat   q_m_{};               ///< Mount rotation (vehicle -> sensor), includes level trim
    Quat   q_yaw0_{};            ///< Heading at the reference instant (rotation about world Z)
    Quat   q_out_prev_{};        ///< Previous output, for hemisphere continuity
    double still_s_       = 0.0;
    double cand_s_        = 0.0;
    MountClass cand_      = MountClass::UNKNOWN;
    double conj_vote_s_   = 0.0;
    double roll_hold_deg_ = 0.0; ///< Last well-defined roll, held through gimbal lock

    std::atomic<bool>       tare_req_{false};
    std::atomic<bool>       auto_remount_{false};
    std::atomic<bool>       referenced_{false};
    std::atomic<bool>       conj_{false};
    std::atomic<bool>       conj_confirmed_{false};
    std::atomic<MountClass> mount_{MountClass::UNKNOWN};
    std::atomic<uint32_t>   ref_count_{0};
};

} // namespace drivers
