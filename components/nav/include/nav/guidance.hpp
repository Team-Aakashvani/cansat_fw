/**
 * @file guidance.hpp
 * @brief Return-to-launch guidance for a canopy steered by thrust vectoring.
 *
 * The CanSat hangs under its (already open) parachute. Tilting the body points the
 * four motors' thrust partly sideways; that horizontal force drags the whole
 * canopy/CanSat system across the ground. Guidance (GNSS rate, ~1-5 Hz):
 *
 *   e      = site - position                          (ENU, metres)
 *   v_des  = clamp(k_pos * e, v_max)                  (ground velocity toward the site)
 *   a_cmd  = clamp(k_vel * ev + k_vel_i * int(ev), a_max),  ev = v_des - v_gnss
 *            (v_gnss includes the wind drift; the integral learns the steady thrust needed
 *             to hold against wind, otherwise a P-only loop stalls short of the site)
 *   tilt   = direction of a_cmd, magnitude atan(|a_cmd| / g), clamped to max_tilt
 *
 * The world-frame tilt is converted to body tilt setpoints (Three.js/firmware ZXY
 * convention) using the vehicle heading, so the attitude loop needs no knowledge of
 * geography. Heading error only rotates the thrust direction; up to ~60 deg of error
 * the vehicle still closes on the site (the closed loop keeps re-aiming), so the
 * magnetometer disturbance from motor currents degrades accuracy, not safety.
 *
 * Header-only, no ESP-IDF dependencies: unit-tested on the host.
 */
#pragma once

#include "config.hpp"
#include <cmath>

namespace nav {

/// Local tangent-plane offsets (east, north) of (lat, lon) from (lat0, lon0), metres
inline void geo_to_en(double lat0, double lon0, double lat, double lon,
                      double& east, double& north) noexcept {
    constexpr double R = 6371008.8, D2R = 3.14159265358979323846 / 180.0;
    north = (lat - lat0) * D2R * R;
    east  = (lon - lon0) * D2R * R * std::cos(lat0 * D2R);
}

/// Averages GNSS fixes on the pad to define the return target (and GNSS->AGL offset)
class SiteEstimator {
public:
    void add(double lat, double lon, double alt_msl, float baro_agl, int sats, float dt) noexcept {
        if (locked_ || sats < GUIDE_CFG.site_min_sats) return;
        const double w = 1.0 / (double)(++n_);   // running mean (later fixes refine it)
        lat_ += (lat - lat_) * w; lon_ += (lon - lon_) * w;
        alt_off_ += ((alt_msl - baro_agl) - alt_off_) * w;
        t_ += dt;
    }
    void lock() noexcept { locked_ = true; }
    bool valid() const noexcept { return n_ > 0 && t_ >= GUIDE_CFG.site_avg_s; }
    double lat() const noexcept { return lat_; }
    double lon() const noexcept { return lon_; }
    float  alt_offset() const noexcept { return (float)alt_off_; }
    void   set(double lat, double lon, float alt_off) noexcept {
        lat_ = lat; lon_ = lon; alt_off_ = alt_off; n_ = 1; t_ = GUIDE_CFG.site_avg_s; locked_ = true;
    }
private:
    double lat_ = 0.0, lon_ = 0.0, alt_off_ = 0.0;
    unsigned n_ = 0; float t_ = 0.0f; bool locked_ = false;
};

class ReturnGuidance {
public:
    struct Output {
        float tilt_x_rad = 0.0f;   ///< Body tilt setpoint about vehicle X (ZXY convention)
        float tilt_y_rad = 0.0f;   ///< Body tilt setpoint about vehicle Y
        float dist_m     = 0.0f;   ///< Distance to the launch site
        float bearing_deg= 0.0f;   ///< Bearing to the site, degrees from north (clockwise)
        float a_east     = 0.0f, a_north = 0.0f;
        bool  active     = false;  ///< False: no valid fix / site -> level
    };

    /**
     * @param site_e, site_n   site position relative to the current fix (east, north), m
     * @param ve, vn           GNSS ground velocity, m/s
     * @param fix_age_s        time since the fix was received
     * @param heading_enu_rad  yaw of vehicle +X, counter-clockwise from east
     */
    void reset() noexcept { ie_ = in_ = 0.0f; }

    Output update(float site_e, float site_n, float ve, float vn,
                  float fix_age_s, float heading_enu_rad, float dt) noexcept {
        Output o;
        const GuidanceConfig& G = GUIDE_CFG;
        o.dist_m = std::sqrt(site_e * site_e + site_n * site_n);
        o.bearing_deg = std::fmod(std::atan2(site_e, site_n) * 57.29578f + 360.0f, 360.0f);
        if (!(fix_age_s <= G.gnss_timeout_s) || !std::isfinite(heading_enu_rad)) { reset(); return o; }
        if (!(dt > 0.0f) || dt > 0.5f) dt = 0.0f;

        // Desired ground velocity toward the site
        float vde = G.k_pos * site_e, vdn = G.k_pos * site_n;
        const float vd = std::sqrt(vde * vde + vdn * vdn);
        if (vd > G.v_max_mps) { vde *= G.v_max_mps / vd; vdn *= G.v_max_mps / vd; }
        if (o.dist_m < G.arrive_radius_m) { vde = 0.0f; vdn = 0.0f; }

        // Acceleration command: PI velocity loop (wind drift is part of v_gnss)
        const float eve = vde - ve, evn = vdn - vn;
        float ae = G.k_vel * eve + G.k_vel_i * ie_, an = G.k_vel * evn + G.k_vel_i * in_;
        const float am = std::sqrt(ae * ae + an * an);
        if (am > G.a_max_mps2) {
            ae *= G.a_max_mps2 / am; an *= G.a_max_mps2 / am;      // saturated: don't wind up
        } else {
            ie_ += eve * dt; in_ += evn * dt;
            const float il = G.a_max_mps2 / std::max(G.k_vel_i, 1e-3f);
            const float im = std::sqrt(ie_ * ie_ + in_ * in_);
            if (im > il) { ie_ *= il / im; in_ *= il / im; }
        }
        o.a_east = ae; o.a_north = an;

        // World tilt vector (tangent of the tilt angle), clamped
        const float g = (float)G0_MPS2;
        float te = ae / g, tn = an / g;
        const float tmax = std::tan(G.max_tilt_deg * 0.01745329f);
        const float tm = std::sqrt(te * te + tn * tn);
        if (tm > tmax) { te *= tmax / tm; tn *= tmax / tm; }

        // Into the yawed vehicle frame (x' = vehicle heading, y' = 90 deg left of it)
        const float c = std::cos(heading_enu_rad), s = std::sin(heading_enu_rad);
        const float tx =  c * te + s * tn;
        const float ty = -s * te + c * tn;

        // Thrust direction u = normalise(tx, ty, 1); with R = Rz(yaw) Rx(p) Ry(r):
        //   u = (sin r, -sin p cos r, cos p cos r)  =>  r = asin(ux), p = atan2(-uy, uz)
        const float n = std::sqrt(tx * tx + ty * ty + 1.0f);
        o.tilt_y_rad = std::asin(tx / n);
        o.tilt_x_rad = std::atan2(-ty / n, 1.0f / n);
        o.active = true;
        return o;
    }

private:
    float ie_ = 0.0f, in_ = 0.0f;   ///< Velocity-error integral (learned wind compensation)
};

} // namespace nav
