/**
 * @file encoder.cpp
 * @brief CAN-7USAT telemetry encoder — CSV packet builder.
 */
#include "telemetry/encoder.hpp"
#include "nav/config.hpp"
#include "nav/frames.hpp"
#include <cstdio>
#include <cstring>
#include <cmath>

namespace telemetry {

int TelemetryEncoder::encode(const TelemetryFrame& f, char* out, size_t out_len) const noexcept {
    if (!out || out_len < 4) return 0;
    char mission_time[16];
    format_mission_time(f.mission_time_s, mission_time, sizeof(mission_time));
    int n = snprintf(out, out_len,
        "%s,%u,%.2f,%.1f,%.1f,%.2f,"
        "%s,%.6f,%.6f,%.2f,%d,"
        "%.2f,%.2f,%.2f,%u,"
        "%.1f,%d",
        mission_time, (unsigned)f.packet_count, (double)f.altitude_m, (double)f.pressure_pa, (double)f.temperature_c, (double)f.voltage_v,
        f.gnss_time_str, (double)f.latitude_deg, (double)f.longitude_deg, (double)f.gnss_alt_msl_m, f.satellites,
        (double)f.tilt_x_deg, (double)f.tilt_y_deg, (double)f.tilt_z_deg, (unsigned)f.software_state,
        (double)f.cc1101_freq_mhz, (int)f.cc1101_rssi_dbm
    );
    if (n < 0 || (size_t)n >= out_len) { out[out_len - 1] = '\0'; return (int)out_len - 1; }
    return n;
}

TelemetryFrame TelemetryEncoder::make_frame(
        const nav::FlightComputerOutput& fc,
        const drivers::BaroData&         baro,
        const drivers::GNSSData&         gnss,
        const drivers::IMUData&          imu,
        const drivers::PowerData&        pwr,
        uint32_t freq_hz, int8_t rssi,
        uint32_t packet_count,
        uint32_t mission_time_s) noexcept {
    TelemetryFrame f{};
    f.packet_count = packet_count; f.mission_time_s = mission_time_s; f.software_state = fc.sup.state_code;
    f.altitude_m = baro.valid ? (float)baro.altitude_agl_m : (float)fc.imm.nav.p(2);
    f.pressure_pa = baro.valid ? (float)baro.pressure_pa : 101325.0f;
    f.temperature_c = baro.valid ? (float)baro.temperature_c : 25.0f;
    f.voltage_v = pwr.valid ? (float)pwr.voltage_v : 0.0f;
    f.satellites = gnss.satellites;
    if (gnss.time_str[0] != '\0') {
        snprintf(f.gnss_time_str, sizeof(f.gnss_time_str), "%s", gnss.time_str);
    } else {
        snprintf(f.gnss_time_str, sizeof(f.gnss_time_str), "00:00:00");
    }

    if (gnss.valid || gnss.lat_deg != 0.0 || gnss.lon_deg != 0.0) {
        f.latitude_deg = gnss.lat_deg;
        f.longitude_deg = gnss.lon_deg;
        f.gnss_alt_msl_m = gnss.alt_msl_m;
    } else {
        f.latitude_deg = 0.0;
        f.longitude_deg = 0.0;
        f.gnss_alt_msl_m = 0.0;
    }
    double q_norm_sq = fc.imm.nav.q(0)*fc.imm.nav.q(0) + fc.imm.nav.q(1)*fc.imm.nav.q(1) +
                        fc.imm.nav.q(2)*fc.imm.nav.q(2) + fc.imm.nav.q(3)*fc.imm.nav.q(3);
    if (imu.euler_valid) {
        f.tilt_x_deg = (float)imu.euler_pitch_deg;
        f.tilt_y_deg = (float)imu.euler_roll_deg;
        f.tilt_z_deg = (float)imu.euler_yaw_deg;
    } else if (imu.valid) {
        double ax = imu.acc_x;
        double ay = imu.acc_y;
        double az = imu.acc_z;
        double pitch_deg = std::atan2(-ax, std::sqrt(ay * ay + az * az)) * (180.0 / nav::PI);
        double roll_deg  = std::atan2(ay, az) * (180.0 / nav::PI);
        double yaw_deg   = (double)(imu.gyr_z * 180.0 / nav::PI);

        f.tilt_x_deg = (float)pitch_deg;
        f.tilt_y_deg = (float)roll_deg;
        f.tilt_z_deg = (float)yaw_deg;
    } else if (fc.t_s > 0.0 && q_norm_sq > 0.5) {
        nav::EulerAngles ea = nav::euler_from_quat(fc.imm.nav.q);
        f.tilt_x_deg = (float)(ea.pitch_rad * 180.0 / nav::PI);
        f.tilt_y_deg = (float)(ea.roll_rad  * 180.0 / nav::PI);
        f.tilt_z_deg = (float)(ea.yaw_rad   * 180.0 / nav::PI);
    } else {
        f.tilt_x_deg = 0.0f;
        f.tilt_y_deg = 0.0f;
        f.tilt_z_deg = 0.0f;
    }
    f.cc1101_freq_mhz = (float)freq_hz / 1.0e6f; f.cc1101_rssi_dbm = rssi;
    return f;
}

void TelemetryEncoder::format_mission_time(uint32_t seconds, char* buf, size_t len) noexcept {
    uint32_t h = seconds / 3600, m = (seconds % 3600) / 60, s = seconds % 60;
    snprintf(buf, len, "%02u:%02u:%02u", (unsigned)h, (unsigned)m, (unsigned)s);
}

} // namespace telemetry
