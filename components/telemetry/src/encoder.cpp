/**
 * @file encoder.cpp
 * @brief CAN-7USAT telemetry encoder — CSV packet builder.
 *
 * Produces the mandatory CSV line at exactly 1Hz.
 * All floating-point values are formatted with fixed decimal places.
 */
#include "telemetry/encoder.hpp"
#include "nav/config.hpp"
#include "nav/frames.hpp"
#include <cstdio>
#include <cstring>
#include <cmath>

namespace telemetry {

int TelemetryEncoder::encode(const TelemetryFrame& f,
                              char* out, size_t out_len) const noexcept {
    if (!out || out_len < 4) return 0;

    char mission_time[12];
    format_mission_time(f.mission_time_s, mission_time, sizeof(mission_time));

    // CAN-7USAT CSV format (fields after TEAM_ID):
    // MISSION_TIME,PACKET_COUNT,ALTITUDE,PRESSURE,TEMPERATURE,
    // VOLTAGE,GNSS_TIME,LATITUDE,LONGITUDE,GNSS_ALT,SATS,
    // TILT_X,TILT_Y,ROT_Z,SOFTWARE_STATE,
    // CC1101_FREQ,CC1101_RSSI,P4_REC,P4_SD,P4_FPS
    int n = snprintf(out, out_len,
        "%s,%u,%.2f,%.1f,%.1f,%.2f,"    // 6 fields
        "%s,%.6f,%.6f,%.2f,%d,"          // 5 fields
        "%.2f,%.2f,%.2f,%u,"             // 4 fields
        "%.1f,%d,%u,%.2f,%u",            // 5 extended fields
        mission_time,
        (unsigned)f.packet_count,
        (double)f.altitude_m,
        (double)f.pressure_pa,
        (double)f.temperature_c,
        (double)f.voltage_v,
        f.gnss_time_str,
        (double)f.latitude_deg,
        (double)f.longitude_deg,
        (double)f.gnss_alt_msl_m,
        f.satellites,
        (double)f.accel_x_mps2,
        (double)f.accel_y_mps2,
        (double)f.gyro_z_dps,
        (unsigned)f.software_state,
        (double)f.cc1101_freq_mhz,
        (int)f.cc1101_rssi_dbm,
        (unsigned)f.p4_recording,
        (double)f.p4_sd_gb,
        (unsigned)f.p4_fps
    );

    if (n < 0 || (size_t)n >= out_len) {
        out[out_len - 1] = '\0';
        return (int)out_len - 1;
    }
    return n;
}

TelemetryFrame TelemetryEncoder::make_frame(
        const nav::FlightComputerOutput& fc,
        const drivers::BaroData&         baro,
        const drivers::GNSSData&         gnss,
        const drivers::IMUData&          imu,
        const drivers::PowerData&        pwr,
        const p4_link::P4Status&         p4,
        uint32_t freq_hz, int8_t rssi,
        uint32_t packet_count,
        uint32_t mission_time_s) noexcept {

    TelemetryFrame f{};
    f.packet_count   = packet_count;
    f.mission_time_s = mission_time_s;
    f.software_state = fc.sup.state_code;

    // Barometric fields
    f.altitude_m    = baro.valid ? (float)baro.altitude_agl_m  : (float)fc.imm.nav.p(2);
    f.pressure_pa   = baro.valid ? (float)baro.pressure_pa     : 101325.0f;
    f.temperature_c = baro.valid ? (float)baro.temperature_c   : 25.0f;

    // Power
    f.voltage_v = pwr.valid ? (float)pwr.voltage_v : 0.0f;

    // GNSS
    if (gnss.valid) {
        f.latitude_deg  = (float)gnss.lat_deg;
        f.longitude_deg = (float)gnss.lon_deg;
        f.gnss_alt_msl_m= (float)gnss.alt_msl_m;
        f.satellites    = gnss.satellites;
        strncpy(f.gnss_time_str, gnss.time_str, sizeof(f.gnss_time_str) - 1);
    } else {
        strncpy(f.gnss_time_str, "00:00:00", sizeof(f.gnss_time_str));
    }

    // IMU — use EKF outputs if IMU data stale
    if (imu.valid) {
        f.accel_x_mps2 = (float)imu.acc_x;
        f.accel_y_mps2 = (float)imu.acc_y;
        f.gyro_z_dps   = (float)(imu.gyr_z * 180.0 / nav::PI);
    } else {
        // Fall back to EKF state
        nav::EulerAngles ea = nav::euler_from_quat(fc.imm.nav.q);
        f.accel_x_mps2 = (float)(ea.pitch_rad * 180.0 / nav::PI);
        f.accel_y_mps2 = (float)(ea.roll_rad  * 180.0 / nav::PI);
        f.gyro_z_dps   = 0.0f;
    }

    // Extended Mission
    f.cc1101_freq_mhz = (float)freq_hz / 1.0e6f;
    f.cc1101_rssi_dbm = rssi;
    f.p4_recording    = p4.recording ? 1 : 0;
    f.p4_sd_gb        = p4.sd_free_gb;
    f.p4_fps          = p4.fps;

    return f;
}

void TelemetryEncoder::format_mission_time(uint32_t seconds,
                                            char* buf, size_t len) noexcept {
    uint32_t h = seconds / 3600;
    uint32_t m = (seconds % 3600) / 60;
    uint32_t s = seconds % 60;
    snprintf(buf, len, "%02u:%02u:%02u", (unsigned)h, (unsigned)m, (unsigned)s);
}

} // namespace telemetry
