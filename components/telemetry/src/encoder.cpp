/**
 * @file encoder.cpp
 * @brief CAN-7USAT India 2026 telemetry frame builder.
 */
#include "telemetry/encoder.hpp"
#include "nav/config.hpp"
#include <cstdio>
#include <cstring>
#include <cmath>

namespace telemetry {

const char* TelemetryEncoder::header() noexcept {
    return "TEAM_ID,TIME_STAMPING_S,PACKET_COUNT,ALTITUDE_M,PRESSURE_PA,TEMP_C,VOLTAGE_V,"
           "GNSS_TIME_S,GNSS_LATITUDE,GNSS_LONGITUDE,GNSS_ALTITUDE_M,GNSS_SATS,"
           "ACC_X_MPS2,ACC_Y_MPS2,ACC_Z_MPS2,ROLL_DEG,PITCH_DEG,GYRO_SPIN_RATE_DPS,"
           "FLIGHT_SOFTWARE_STATE,HEADING_DEG,HUMIDITY_PCT,VOC_INDEX,NOX_INDEX";
}

int TelemetryEncoder::encode(const TelemetryFrame& f, char* out, size_t out_len) const noexcept {
    if (!out || out_len < 4) return 0;
    int n = snprintf(out, out_len,
        "%s,%.1f,%lu,%.1f,%.0f,%.1f,%.2f,"
        "%.0f,%.6f,%.6f,%.1f,%d,"
        "%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%s,"
        "%.1f,%.1f,%u,%u",
        nav::TEAM_ID_STR, f.time_s, (unsigned long)f.packet_count, (double)f.altitude_m,
        (double)f.pressure_pa, (double)f.temperature_c, (double)f.voltage_v,
        f.gnss_time_s, f.latitude_deg, f.longitude_deg, f.gnss_alt_msl_m, f.satellites,
        (double)f.acc_x, (double)f.acc_y, (double)f.acc_z, (double)f.roll_deg, (double)f.pitch_deg,
        (double)f.spin_rate_dps, f.state ? f.state : "BOOT",
        (double)f.heading_deg, (double)f.humidity_pct, (unsigned)f.voc_index, (unsigned)f.nox_index);
    if (n < 0 || (size_t)n >= out_len) { out[out_len - 1] = '\0'; return (int)out_len - 1; }
    return n;
}

TelemetryFrame TelemetryEncoder::make_frame(
        const char* state, float alt_est_m,
        const drivers::BaroData& baro, const drivers::GNSSData& gnss,
        const drivers::IMUData& imu, const drivers::PowerData& pwr,
        float humidity_pct, uint16_t voc_index, uint16_t nox_index,
        uint32_t packet_count, double time_s) noexcept {
    TelemetryFrame f{};
    f.time_s        = time_s;
    f.packet_count  = packet_count;
    f.state         = state;
    f.altitude_m    = alt_est_m;                         // relative to the pad (mission AGL)
    f.pressure_pa   = baro.valid ? (float)baro.pressure_pa : 0.0f;
    f.temperature_c = baro.valid ? (float)baro.temperature_c : 0.0f;
    f.voltage_v     = pwr.valid ? (float)pwr.voltage_v : 0.0f;
    f.gnss_time_s   = gnss.gnss_time_s;
    f.satellites    = gnss.satellites;
    if (gnss.valid || gnss.lat_deg != 0.0 || gnss.lon_deg != 0.0) {
        f.latitude_deg = gnss.lat_deg; f.longitude_deg = gnss.lon_deg; f.gnss_alt_msl_m = gnss.alt_msl_m;
    }
    if (imu.valid) {
        f.acc_x = (float)imu.acc_x; f.acc_y = (float)imu.acc_y; f.acc_z = (float)imu.acc_z;
        f.spin_rate_dps = (float)(imu.gyr_z * 180.0 / nav::PI);
    }
    if (imu.euler_valid) {                               // ZXY: tilt_x = pitch, tilt_y = roll
        f.pitch_deg   = (float)imu.euler_pitch_deg;
        f.roll_deg    = (float)imu.euler_roll_deg;
        f.heading_deg = (float)imu.euler_yaw_deg;
    }
    f.humidity_pct = humidity_pct;
    f.voc_index = voc_index; f.nox_index = nox_index;
    return f;
}

} // namespace telemetry
