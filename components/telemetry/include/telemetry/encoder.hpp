/**
 * @file encoder.hpp
 * @brief CAN-7USAT India 2026 telemetry frame (guidelines section 6, telemetry table).
 *
 * Required fields, in order:
 *   1 TEAM_ID (2026-IN-SPACeCAN-7USAT-XXX)   2 TIME_STAMPING (s since power-on)
 *   3 PACKET_COUNT   4 ALTITUDE (m, rel. ground, 0.1)   5 PRESSURE (Pa, 1)
 *   6 TEMP (C, 0.1)  7 VOLTAGE (V, 0.01)   8 GNSS_TIME (s)   9/10 GNSS LAT/LON (deg, 1e-4)
 *  11 GNSS_ALTITUDE (m, 0.1)  12 GNSS_SATS
 *  13 ACCELEROMETER DATA: ACC_X, ACC_Y, ACC_Z (m/s^2) + ROLL, PITCH (deg)
 *  14 GYRO_SPIN_RATE (deg/s)  15 FLIGHT_SOFTWARE_STATE
 * Optional mission data appended: HEADING, HUMIDITY, VOC_INDEX, NOX_INDEX.
 */
#pragma once

#include "nav/config.hpp"
#include "drivers/bmp585.hpp"
#include "drivers/ngps01.hpp"
#include "drivers/bno085.hpp"
#include "drivers/ina260.hpp"
#include <cstdint>
#include <cstdio>

namespace telemetry {

struct TelemetryFrame {
    double      time_s;            ///< since power-on
    uint32_t    packet_count;
    float       altitude_m;
    float       pressure_pa;
    float       temperature_c;
    float       voltage_v;
    double      gnss_time_s;       ///< UTC seconds of day from the receiver
    double      latitude_deg;
    double      longitude_deg;
    double      gnss_alt_msl_m;
    int         satellites;
    float       acc_x, acc_y, acc_z;   ///< vehicle-frame specific force, m/s^2
    float       roll_deg, pitch_deg;
    float       spin_rate_dps;     ///< no mechanical gyro fitted yet: CanSat body spin rate about Z
    const char* state;             ///< flight software state name
    float       heading_deg;
    float       humidity_pct;
    uint16_t    voc_index, nox_index;
};

class TelemetryEncoder {
public:
    static constexpr size_t BUF_LEN = 384;
    TelemetryEncoder() noexcept = default;

    /// Full frame including the TEAM_ID field (no trailing newline)
    int encode(const TelemetryFrame& frame, char* out, size_t out_len) const noexcept;

    /// Column header for Flight_<TEAM_ID>.csv
    static const char* header() noexcept;

    static TelemetryFrame make_frame(
        const char*                      state,
        float                            alt_est_m,     ///< Filtered altitude AGL (fallback when baro invalid)
        const drivers::BaroData&         baro,
        const drivers::GNSSData&         gnss,
        const drivers::IMUData&          imu,
        const drivers::PowerData&        pwr,
        float humidity_pct, uint16_t voc_index, uint16_t nox_index,
        uint32_t packet_count,
        double time_s) noexcept;
};

} // namespace telemetry
