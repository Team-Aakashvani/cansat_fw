/**
 * @file encoder.hpp
 * @brief CAN-7USAT compliant telemetry packet encoder.
 */
#pragma once

#include "nav/flight_computer.hpp"
#include "drivers/bmp585.hpp"
#include "drivers/ngps01.hpp"
#include "drivers/bno085.hpp"
#include "drivers/ina260.hpp"
#include <cstdint>
#include <cstdio>

namespace telemetry {

struct TelemetryFrame {
    float    altitude_m;
    float    pressure_pa;
    float    temperature_c;
    float    voltage_v;
    double   latitude_deg;
    double   longitude_deg;
    double   gnss_alt_msl_m;
    int      satellites;
    float    tilt_x_deg;
    float    tilt_y_deg;
    float    tilt_z_deg;
    char     gnss_time_str[16];
    uint8_t  software_state;
    uint32_t packet_count;
    uint32_t mission_time_s;
    float    cc1101_freq_mhz;
    int16_t  cc1101_rssi_dbm;
};

class TelemetryEncoder {
public:
    static constexpr size_t BUF_LEN = 384;
    TelemetryEncoder() noexcept = default;
    int encode(const TelemetryFrame& frame, char* out, size_t out_len) const noexcept;
    static TelemetryFrame make_frame(
        const nav::FlightComputerOutput& fc_out,
        const drivers::BaroData&         baro,
        const drivers::GNSSData&         gnss,
        const drivers::IMUData&          imu,
        const drivers::PowerData&        pwr,
        uint32_t freq_hz, int8_t rssi,
        uint32_t packet_count,
        uint32_t mission_time_s) noexcept;
private:
    static void format_mission_time(uint32_t seconds, char* buf, size_t len) noexcept;
};

} // namespace telemetry
