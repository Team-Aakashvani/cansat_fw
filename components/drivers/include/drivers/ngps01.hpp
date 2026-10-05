/**
 * @file ngps01.hpp
 * @brief N-GS-01 NavIC/IRNSS + GPS GNSS receiver driver (UART + NMEA).
 *
 * The N-GS-01 is a NavIC/IRNSS multi-constellation receiver outputting
 * NMEA-0183 sentences at 1Hz over UART at 115200 baud.
 *
 * Parsed sentences:
 *   $GPGGA — latitude, longitude, altitude (MSL), satellites, fix quality
 *   $GPRMC — lat, lon, speed, track, date/time
 *   $GPVTG — track and speed (for velocity estimation)
 *   $GPGSV — satellite visibility (for health monitoring)
 *
 * GNSS data is converted to ENU position/velocity for the EKF.
 * Reference point (origin) is set at first valid fix → relative ENU.
 *
 * @compliance CAN-7USAT §5.3: GNSS_TIME, LATITUDE, LONGITUDE, GNSS_ALT, SATS fields.
 */
#pragma once

#include "app_hal/uart_bus.hpp"
#include <cstdint>
#include <cmath>

namespace drivers {

struct GNSSData {
    // ENU position relative to first fix (m)
    double pos_e, pos_n, pos_u;
    // ENU velocity (m/s) estimated from GPVTG + differentiation
    double vel_e, vel_n, vel_u;
    // WGS-84 absolute position
    double lat_deg, lon_deg, alt_msl_m;
    // Telemetry fields
    double gnss_time_s;   ///< UTC seconds since midnight
    char   time_str[16];  ///< "HH:MM:SS"
    int    satellites;
    int    fix_quality;   ///< 0=no fix, 1=GPS, 2=DGPS/GAGAN, 4=NavIC
    bool   valid;
    bool   fix_3d;
    bool   gagan_active;  ///< True if GAGAN / SBAS differential corrections active (fix_quality == 2)
};

class NGPS01 {
public:
    NGPS01() noexcept = default;

    /// Initialise: configure UART, apply high-sensitivity settings, inject A-GPS seed.
    esp_err_t init(hal::UARTBus& uart, double est_lat = 0.0, double est_lon = 0.0, double est_alt = 0.0) noexcept;

    /// Configure receiver for maximum low-signal sensitivity (10 dB-Hz threshold, 5 deg mask, Auto-2D/3D).
    void configure_high_sensitivity(double est_lat = 0.0, double est_lon = 0.0, double est_alt = 0.0) noexcept;

    /// Inject AssistNow / A-GPS position seeding to collapse correlator search space by >95%.
    void seed_assisted_gps(double lat_deg, double lon_deg, double alt_m) noexcept;

    /// Configure SBAS / GAGAN differential corrections on the receiver.
    void configure_sbas_gagan() noexcept;

    /// Update barometric altitude aiding for 2.5D GNSS fix in low-signal conditions.
    void set_baro_alt_aiding(double baro_alt_m) noexcept { baro_alt_aiding_m_ = baro_alt_m; }

    /// Non-blocking poll: returns valid GNSSData if a new fix is available.
    GNSSData read() noexcept;

    /// Returns true if origin has been set (first valid fix received).
    bool has_origin() const noexcept { return origin_set_; }

    bool is_ready() const noexcept { return ready_; }

private:
    hal::UARTBus* uart_       = nullptr;
    bool          ready_      = false;
    bool          origin_set_ = false;
    double        baro_alt_aiding_m_ = 0.0;

    // Reference origin (set at first valid fix)
    double ref_lat_rad_ = 0.0;
    double ref_lon_rad_ = 0.0;
    double ref_alt_m_   = 0.0;

    // Previous velocity data for derivative
    double prev_time_s_   = -1.0;
    double prev_lat_deg_  = 0.0;
    double prev_lon_deg_  = 0.0;
    double prev_alt_msl_  = 0.0;

    // Parse NMEA sentences
    void process_sentence(const char* sentence) noexcept;
    bool parse_gga(const char* sentence, GNSSData& out) noexcept;
    bool parse_rmc(const char* sentence, GNSSData& out) noexcept;
    bool parse_vtg(const char* sentence, GNSSData& out) noexcept;
    bool parse_gsv(const char* sentence, GNSSData& out) noexcept;

    // Convert lat/lon to ENU
    void lla_to_enu(double lat_deg, double lon_deg, double alt_m,
                    double& e, double& n, double& u) const noexcept;

    // NMEA utilities
    static double nmea_lat(const char* val, const char dir) noexcept;
    static double nmea_lon(const char* val, const char dir) noexcept;
    static uint8_t nmea_checksum(const char* sentence) noexcept;
    static bool verify_checksum(const char* sentence) noexcept;

    GNSSData cached_{};
    char line_buf_[128] = {};
    int  line_idx_      = 0;
};

} // namespace drivers
