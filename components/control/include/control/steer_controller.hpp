/**
 * @file steer_controller.hpp
 * @brief Attitude cascade + quad-X mixer for thrust-vector steering under the canopy.
 *
 *   tilt setpoint (rad) ─► angle P ─► rate setpoint ─► rate PID ─► torque ─► X mixer ─► 4 throttles
 *   yaw: rate damping to 0 (stops the canopy spinning the vehicle, keeps the thrust
 *        direction steady for guidance)
 *
 * Vehicle frame = the attitude reference frame: +X forward, +Y left, +Z up.
 * Motors (top view):  M1 FL (+x,+y)  M2 FR (+x,-y)  M3 RR (-x,-y)  M4 RL (-x,+y)
 *   +torque about X (tilt_x +) raises the +Y side  -> FL, RL up;  FR, RR down
 *   +torque about Y (tilt_y +) raises the -X side  -> RR, RL up;  FL, FR down
 *   +torque about Z (CCW)      CW props' reaction  -> FL, RR up (CW), FR, RL down (CCW)
 * If the motors are mounted rotated relative to the IMU, set frame_yaw_deg;
 * if props spin the other way round, set yaw_sign = -1 (verify props-off first).
 *
 * Header-only, no ESP-IDF dependencies: unit-tested on the host.
 */
#pragma once

#include "nav/config.hpp"
#include <cmath>
#include <algorithm>

namespace control {

class SteerController {
public:
    struct Throttles { float m[4] = {0, 0, 0, 0}; };

    float frame_yaw_deg = 0.0f;   ///< Motor frame rotation vs. vehicle frame about Z
    float yaw_sign      = 1.0f;   ///< +1: FL/RR spin CW (props-in default)

    void reset() noexcept { ix_ = iy_ = ax_ = ay_ = 0.0f; prev_ex_ = prev_ey_ = 0.0f; first_ = true; }

    /**
     * @param sp_x, sp_y   tilt setpoints (rad)
     * @param tilt_x, tilt_y current tilt (rad), ZXY convention
     * @param gx, gy, gz   body rates (rad/s), vehicle frame
     * @param collective   base throttle 0..1
     */
    Throttles update(float sp_x, float sp_y, float tilt_x, float tilt_y,
                     float gx, float gy, float gz, float collective, float dt) noexcept {
        const nav::ActuatorConfig& A = nav::ACT_CFG;
        Throttles out;
        if (!(dt > 0.0f) || dt > 0.1f) dt = 0.01f;
        if (!finite_all(sp_x, sp_y, tilt_x, tilt_y) || !finite_all(gx, gy, gz, collective)) {
            reset();
            for (float& m : out.m) m = A.idle_throttle;
            return out;
        }

        // Outer: angle error -> rate setpoint (PI: the canopy's pendulum torque would
        // otherwise leave a steady tilt shortfall)
        const float eax = sp_x - tilt_x, eay = sp_y - tilt_y;
        const float alim = 0.5f * A.max_rate_rps / std::max(A.ki_angle, 1e-3f);
        ax_ = std::clamp(ax_ + eax * dt, -alim, alim);
        ay_ = std::clamp(ay_ + eay * dt, -alim, alim);
        const float rsx = std::clamp(A.kp_angle * eax + A.ki_angle * ax_, -A.max_rate_rps, A.max_rate_rps);
        const float rsy = std::clamp(A.kp_angle * eay + A.ki_angle * ay_, -A.max_rate_rps, A.max_rate_rps);

        // Inner: rate PID -> torque (derivative on measurement-free error, first-sample guarded)
        const float ex = rsx - gx, ey = rsy - gy;
        const float ilim = 0.5f * A.max_torque / std::max(A.ki_rate, 1e-3f);
        ix_ = std::clamp(ix_ + ex * dt, -ilim, ilim);
        iy_ = std::clamp(iy_ + ey * dt, -ilim, ilim);
        const float dx = first_ ? 0.0f : (ex - prev_ex_) / dt;
        const float dy = first_ ? 0.0f : (ey - prev_ey_) / dt;
        prev_ex_ = ex; prev_ey_ = ey; first_ = false;

        float tx = A.kp_rate * ex + A.ki_rate * ix_ + A.kd_rate * dx;
        float ty = A.kp_rate * ey + A.ki_rate * iy_ + A.kd_rate * dy;
        float tz = -A.kp_yaw_rate * gz * yaw_sign;
        tx = std::clamp(tx, -A.max_torque, A.max_torque);
        ty = std::clamp(ty, -A.max_torque, A.max_torque);
        tz = std::clamp(tz, -0.5f * A.max_torque, 0.5f * A.max_torque);

        // Motor frame rotation
        if (frame_yaw_deg != 0.0f) {
            const float c = std::cos(-frame_yaw_deg * 0.01745329f), s = std::sin(-frame_yaw_deg * 0.01745329f);
            const float rx = c * tx - s * ty, ry = s * tx + c * ty;
            tx = rx; ty = ry;
        }

        const float T = std::clamp(collective, A.idle_throttle, A.max_throttle);
        float m[4] = {
            T + tx - ty + tz,   // M1 FL
            T - tx - ty - tz,   // M2 FR
            T - tx + ty + tz,   // M3 RR
            T + tx + ty - tz,   // M4 RL
        };

        // Desaturate: keep the differential (attitude authority), shift the collective
        float hi = m[0], lo = m[0];
        for (int i = 1; i < 4; ++i) { hi = std::max(hi, m[i]); lo = std::min(lo, m[i]); }
        float shift = 0.0f;
        if (hi > A.max_throttle) shift = A.max_throttle - hi;
        else if (lo < A.idle_throttle) shift = A.idle_throttle - lo;
        for (int i = 0; i < 4; ++i) out.m[i] = std::clamp(m[i] + shift, A.idle_throttle, A.max_throttle);
        return out;
    }

private:
    float ix_ = 0.0f, iy_ = 0.0f, ax_ = 0.0f, ay_ = 0.0f, prev_ex_ = 0.0f, prev_ey_ = 0.0f;
    bool  first_ = true;

    static bool finite_all(float a, float b, float c, float d) noexcept {
        return std::isfinite(a) && std::isfinite(b) && std::isfinite(c) && std::isfinite(d);
    }
};

} // namespace control
