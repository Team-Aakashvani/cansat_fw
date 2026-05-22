/**
 * @file encoder.hpp
 * @brief CAN-7USAT compliant telemetry packet encoder.
 *
 * Produces the mandatory 1Hz CSV downlink exactly as specified in
 * CAN-7USAT India 2026 guidelines §5.3 (Telemetry Data Format):
 *
 *   <TEAM_ID>,<MISSION_TIME>,<PACKET_COUNT>,<ALTITUDE>,<PRESSURE>,
 *   <TEMPERATURE>,<VOLTAGE>,<GNSS_TIME>,<LATITUDE>,<LONGITUDE>,
 *   <GNSS_ALT>,<SATS>,<TILT_X>,<TILT_Y>,<ROT_Z>,<SOFTWARE_STATE>
 *
 * Fields:
 *   TEAM_ID        — 4-digit numeric team identifier
 *   MISSION_TIME   — HH:MM:SS (elapsed from launch, not UTC)
 *   PACKET_COUNT   — monotonically increasing uint32
 *   ALTITUDE       — BMP585 altitude AGL (m), 2 dp
 *   PRESSURE       — BMP585 pressure (Pa), 1 dp
 *   TEMPERATURE    — BMP585 temperature (°C), 1 dp
 *   VOLTAGE        — INA260 bus voltage (V), 2 dp
 *   GNSS_TIME      — N-GS-01 UTC time string "HH:MM:SS"
 *   LATITUDE       — decimal degrees, 6 dp
 *   LONGITUDE      — decimal degrees, 6 dp
 *   GNSS_ALT       — GNSS altitude MSL (m), 2 dp
 *   SATS           — satellite count
 *   TILT_X         — IMU accel X (m/s²) or Euler pitch, 2 dp
 *   TILT_Y         — IMU accel Y (m/s²) or Euler roll,  2 dp
 *   ROT_Z          — Gyro Z (°/s), 2 dp
 *   SOFTWARE_STATE — integer phase code (2–7 per CAN-7USAT §3)
 *
 * @compliance CAN-7USAT 2026 §5.3
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
    // Raw sensor values (populated by the caller)
    float    altitude_m;
    float    pressure_pa;
    float    temperature_c;
    float    voltage_v;
    float    latitude_deg;
    float    longitude_deg;
    float    gnss_alt_msl_m;
    int      satellites;
    float    accel_x_mps2;
    float    accel_y_mps2;
    float    gyro_z_dps;
    char     gnss_time_str[12];   ///< "HH:MM:SS"
    uint8_t  software_state;       ///< 0–7
    uint32_t packet_count;
    uint32_t mission_time_s;       ///< Seconds elapsed since BOOST latch

    // Extended Mission Data (RF Mapping + P4 Camera)
    float    cc1101_freq_mhz;
    int16_t  cc1101_rssi_dbm;
    uint8_t  p4_recording;
    float    p4_sd_gb;
    uint8_t  p4_fps;
};

class TelemetryEncoder {
public:
    static constexpr size_t BUF_LEN = 384;

    TelemetryEncoder() noexcept = default;

    /**
     * @brief Build CSV telemetry fields from a TelemetryFrame.
     * 
     * NOTE: This produces the CSV string starting from MISSION_TIME. 
     * The TEAM_ID is prepended by the XBee link layer.
     * 
     * Returns length written (excluding null terminator).
     */
    int encode(const TelemetryFrame& frame, char* out, size_t out_len) const noexcept;

    /// Convenience: build frame from FlightComputer output + latest sensor data.
    static TelemetryFrame make_frame(
        const nav::FlightComputerOutput& fc_out,
        const drivers::BaroData&         baro,
        const drivers::GNSSData&         gnss,
        const drivers::IMUData&          imu,
        const drivers::PowerData&        pwr,
        const p4_link::P4Status&         p4,
        uint32_t freq_hz, int8_t rssi,
        uint32_t packet_count,
        uint32_t mission_time_s) noexcept;

private:
    static void format_mission_time(uint32_t seconds,
                                    char* buf, size_t len) noexcept;
};

} // namespace telemetry
