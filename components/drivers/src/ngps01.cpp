#include "drivers/ngps01.hpp"
#include "esp_log.h"
#include <cstring>
#include <cstdlib>
#include <cstdio>

static const char* TAG = "NGPS01";

static constexpr double DEG2RAD = 3.14159265358979323846 / 180.0;
static constexpr double EARTH_R = 6378137.0;  // WGS-84 semi-major axis (m)

namespace drivers {

esp_err_t NGPS01::init(hal::UARTBus& uart) noexcept {
    uart_  = &uart;
    cached_.valid = false;
    ready_ = true;
    ESP_LOGI(TAG, "N-GS-01 GNSS driver initialised");
    return ESP_OK;
}

GNSSData NGPS01::read() noexcept {
    GNSSData out = cached_;
    out.valid = false;
    if (!ready_ || !uart_) return out;

    int n = uart_->read_line(line_buf_, sizeof(line_buf_), 20);
    if (n <= 0) return out;

    // Route to parser based on sentence type
    if (strncmp(line_buf_, "$GPGGA", 6) == 0 ||
        strncmp(line_buf_, "$GNGGA", 6) == 0) {
        if (parse_gga(line_buf_, out)) {
            cached_ = out;
        }
    } else if (strncmp(line_buf_, "$GPRMC", 6) == 0 ||
               strncmp(line_buf_, "$GNRMC", 6) == 0) {
        parse_rmc(line_buf_, cached_);
        out = cached_;
    } else if (strncmp(line_buf_, "$GPVTG", 6) == 0 ||
               strncmp(line_buf_, "$GNVTG", 6) == 0) {
        parse_vtg(line_buf_, cached_);
        out = cached_;
    }
    return out;
}

// NMEA GPGGA: $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,q,nn,h.h,aa.a,M,g.g,M,z.z,ssss*hh
bool NGPS01::parse_gga(const char* s, GNSSData& out) noexcept {
    if (!verify_checksum(s)) return false;

    // Tokenise (modifying copy)
    char buf[128];
    strncpy(buf, s, 127); buf[127] = '\0';
    char* tok[20] = {};
    int n = 0;
    char* p = buf;
    while (n < 20 && p) { tok[n++] = p; p = strchr(p, ','); if (p) *p++ = '\0'; }
    if (n < 10) return false;

    // tok[1]=time, tok[2]=lat, tok[3]=N/S, tok[4]=lon, tok[5]=E/W
    // tok[6]=fix quality, tok[7]=satellites, tok[8]=HDOP, tok[9]=altitude
    double lat = nmea_lat(tok[2], tok[3][0]);
    double lon = nmea_lon(tok[4], tok[5][0]);
    double alt = strtod(tok[9], nullptr);
    int fix    = atoi(tok[6]);
    int sats   = atoi(tok[7]);

    if (fix == 0) { out.valid = false; return false; }

    // Set origin on first valid fix
    if (!origin_set_) {
        ref_lat_rad_ = lat * DEG2RAD;
        ref_lon_rad_ = lon * DEG2RAD;
        ref_alt_m_   = alt;
        origin_set_  = true;
        ESP_LOGI(TAG, "Origin set: lat=%.6f lon=%.6f alt=%.1f", lat, lon, alt);
    }

    double e, n2, u;
    lla_to_enu(lat, lon, alt, e, n2, u);

    out.lat_deg    = lat;
    out.lon_deg    = lon;
    out.alt_msl_m  = alt;
    out.pos_e      = e;
    out.pos_n      = n2;
    out.pos_u      = u;
    out.satellites = sats;
    out.fix_quality = fix;
    out.fix_3d     = (sats >= 4 && fix > 0);

    // Parse time (HHMMSS.ss)
    const char* tstr = tok[1];
    int hh = 0, mm = 0; double ss = 0.0;
    if (strlen(tstr) >= 6) {
        hh = (tstr[0]-'0')*10 + (tstr[1]-'0');
        mm = (tstr[2]-'0')*10 + (tstr[3]-'0');
        ss = strtod(tstr + 4, nullptr);
    }
    out.gnss_time_s = hh*3600 + mm*60 + ss;
    snprintf(out.time_str, sizeof(out.time_str), "%02d:%02d:%05.2f", hh, mm, ss);

    // Estimate vertical velocity from altitude derivative
    if (prev_time_s_ > 0.0) {
        double dt = out.gnss_time_s - prev_time_s_;
        if (dt > 0.5 && dt < 3.0) {
            out.vel_u = (alt - prev_alt_msl_) / dt;
        }
    }
    prev_time_s_  = out.gnss_time_s;
    prev_lat_deg_ = lat;
    prev_lon_deg_ = lon;
    prev_alt_msl_ = alt;

    out.valid = true;
    return true;
}

bool NGPS01::parse_rmc(const char* s, GNSSData& out) noexcept {
    if (!verify_checksum(s)) return false;
    // Minimal parse: just update valid flag from status field
    char buf[128]; strncpy(buf, s, 127); buf[127] = '\0';
    char* tok[20] = {}; int n = 0;
    char* p = buf;
    while (n < 20 && p) { tok[n++] = p; p = strchr(p, ','); if (p) *p++ = '\0'; }
    if (n < 3) return false;
    out.valid = (tok[2][0] == 'A');
    return out.valid;
}

bool NGPS01::parse_vtg(const char* s, GNSSData& out) noexcept {
    if (!verify_checksum(s)) return false;
    char buf[128]; strncpy(buf, s, 127); buf[127] = '\0';
    char* tok[20] = {}; int n = 0;
    char* p = buf;
    while (n < 20 && p) { tok[n++] = p; p = strchr(p, ','); if (p) *p++ = '\0'; }
    // tok[1]=true track (deg), tok[5]=speed (knots), tok[7]=speed (km/h)
    if (n < 8) return false;
    double speed_kph  = strtod(tok[7], nullptr);
    double track_deg  = strtod(tok[1], nullptr);
    double speed_mps  = speed_kph / 3.6;
    double track_rad  = track_deg * DEG2RAD;
    out.vel_e = speed_mps * sin(track_rad);
    out.vel_n = speed_mps * cos(track_rad);
    return true;
}

void NGPS01::lla_to_enu(double lat_deg, double lon_deg, double alt_m,
                         double& e, double& n, double& u) const noexcept {
    const double lat_r  = lat_deg * DEG2RAD;
    const double lon_r  = lon_deg * DEG2RAD;
    const double dlat   = lat_r  - ref_lat_rad_;
    const double dlon   = lon_r  - ref_lon_rad_;
    const double dalt   = alt_m  - ref_alt_m_;
    // Flat-Earth approximation (valid for << 100km missions)
    const double cos_lat = cos(ref_lat_rad_);
    n = dlat * EARTH_R;
    e = dlon * EARTH_R * cos_lat;
    u = dalt;
}

double NGPS01::nmea_lat(const char* val, const char dir) noexcept {
    if (!val || !val[0]) return 0.0;
    double ddmm = strtod(val, nullptr);
    double deg = (double)((int)(ddmm / 100));
    double min = ddmm - deg * 100.0;
    double lat = deg + min / 60.0;
    return (dir == 'S') ? -lat : lat;
}

double NGPS01::nmea_lon(const char* val, const char dir) noexcept {
    if (!val || !val[0]) return 0.0;
    double ddmm = strtod(val, nullptr);
    double deg = (double)((int)(ddmm / 100));
    double min = ddmm - deg * 100.0;
    double lon = deg + min / 60.0;
    return (dir == 'W') ? -lon : lon;
}

uint8_t NGPS01::nmea_checksum(const char* s) noexcept {
    uint8_t cs = 0;
    for (const char* p = s + 1; *p && *p != '*'; ++p) cs ^= (uint8_t)*p;
    return cs;
}

bool NGPS01::verify_checksum(const char* s) noexcept {
    const char* star = strchr(s, '*');
    if (!star || strlen(star) < 3) return false;
    uint8_t calc = nmea_checksum(s);
    uint8_t ref  = (uint8_t)strtol(star + 1, nullptr, 16);
    return calc == ref;
}

} // namespace drivers
