#include "drivers/ngps01.hpp"
#include "esp_log.h"
#include <cstring>
#include <cstdlib>
#include <cstdio>

static const char* TAG = "NGPS01";

static constexpr double DEG2RAD = 3.14159265358979323846 / 180.0;
static constexpr double EARTH_R = 6378137.0;  // WGS-84 semi-major axis (m)

namespace drivers {

esp_err_t NGPS01::init(hal::UARTBus& uart, double est_lat, double est_lon, double est_alt) noexcept {
    uart_  = &uart;
    cached_.valid = false;
    ready_ = true;
    configure_high_sensitivity(est_lat, est_lon, est_alt);
    configure_sbas_gagan();
    ESP_LOGI(TAG, "N-GS-01 / NEO-6M GNSS driver initialised with High-Sensitivity & GAGAN/SBAS support");
    return ESP_OK;
}

void NGPS01::seed_assisted_gps(double lat_deg, double lon_deg, double alt_m) noexcept {
    if (!uart_ || (lat_deg == 0.0 && lon_deg == 0.0)) return;

    // 1. Build UBX-AID-INI (Class 0x0B, ID 0x01, 48-byte payload) for u-blox NEO-6M
    uint8_t pkt[8 + 48] = { 0xB5, 0x62, 0x0B, 0x01, 0x30, 0x00 };
    uint8_t* payload = &pkt[6];

    int32_t lat_1e7   = static_cast<int32_t>(lat_deg * 1e7);
    int32_t lon_1e7   = static_cast<int32_t>(lon_deg * 1e7);
    int32_t alt_cm    = static_cast<int32_t>(alt_m * 100.0);
    uint32_t pos_acc  = 5000000; // 50 km accuracy estimate
    uint32_t flags    = (1 << 0) | (1 << 5); // pos valid (bit 0), LLA format (bit 5)

    memcpy(payload + 0,  &lat_1e7,  4);
    memcpy(payload + 4,  &lon_1e7,  4);
    memcpy(payload + 8,  &alt_cm,   4);
    memcpy(payload + 12, &pos_acc,  4);
    memcpy(payload + 44, &flags,    4);

    // Compute UBX Fletcher checksum
    uint8_t ck_a = 0, ck_b = 0;
    for (size_t i = 2; i < 6 + 48; ++i) {
        ck_a += pkt[i];
        ck_b += ck_a;
    }
    pkt[6 + 48]     = ck_a;
    pkt[6 + 48 + 1] = ck_b;

    uart_->write(pkt, sizeof(pkt));
    ESP_LOGI(TAG, "Injected A-GPS position seed to NEO-6M: lat=%.6f lon=%.6f alt=%.1fm (50km search radius)",
             lat_deg, lon_deg, alt_m);

    // 2. Also send PMTK auxiliary position seed for generic MTK receivers
    char pmtk_pos[64];
    int lat_deg_int = static_cast<int>(std::abs(lat_deg));
    double lat_min  = (std::abs(lat_deg) - lat_deg_int) * 60.0;
    int lon_deg_int = static_cast<int>(std::abs(lon_deg));
    double lon_min  = (std::abs(lon_deg) - lon_deg_int) * 60.0;
    char lat_dir    = lat_deg >= 0 ? 'N' : 'S';
    char lon_dir    = lon_deg >= 0 ? 'E' : 'W';
    snprintf(pmtk_pos, sizeof(pmtk_pos), "$PMTK740,%.4f,%c,%.4f,%c,%.1f*1F\r\n",
             lat_deg_int * 100.0 + lat_min, lat_dir,
             lon_deg_int * 100.0 + lon_min, lon_dir, alt_m);
    uart_->write(reinterpret_cast<const uint8_t*>(pmtk_pos), strlen(pmtk_pos));
}

void NGPS01::configure_high_sensitivity(double est_lat, double est_lon, double est_alt) noexcept {
    if (!uart_) return;

    // 1. Force continuous max-performance mode (UBX-CFG-RXM)
    static const uint8_t CMD_MAX_PERF[] = {
        0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x00, 0x00, 0x19, 0x81
    };
    uart_->write(CMD_MAX_PERF, sizeof(CMD_MAX_PERF));

    // 2. Disable heavy/unneeded NMEA sentences to prevent 9600-baud UART buffer congestion
    // GSV (rate=2, every 2s): outputs satellites-in-view without flooding 9600-baud UART
    static const uint8_t CMD_ENABLE_GSV_2S[] = {
        0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x02, 0xFF, 0x17
    };
    uart_->write(CMD_ENABLE_GSV_2S, sizeof(CMD_ENABLE_GSV_2S));

    // GSA (off): saves ~65 bytes/sec
    static const uint8_t CMD_DISABLE_GSA[] = {
        0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13
    };
    uart_->write(CMD_DISABLE_GSA, sizeof(CMD_DISABLE_GSA));

    // GLL (off): saves ~50 bytes/sec
    static const uint8_t CMD_DISABLE_GLL[] = {
        0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11
    };
    uart_->write(CMD_DISABLE_GLL, sizeof(CMD_DISABLE_GLL));

    // VTG (off): saves ~45 bytes/sec
    static const uint8_t CMD_DISABLE_VTG[] = {
        0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19
    };
    uart_->write(CMD_DISABLE_VTG, sizeof(CMD_DISABLE_VTG));

    // GGA (1Hz): primary position + sats + fix quality
    static const uint8_t CMD_ENABLE_GGA[] = {
        0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x01, 0xFB, 0x10
    };
    uart_->write(CMD_ENABLE_GGA, sizeof(CMD_ENABLE_GGA));

    // RMC (1Hz): primary time + velocity
    static const uint8_t CMD_ENABLE_RMC[] = {
        0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x01, 0xFF, 0x18
    };
    uart_->write(CMD_ENABLE_RMC, sizeof(CMD_ENABLE_RMC));

    // 3. Navigation Engine High-Sensitivity & Fast-Lock Tuning (UBX-CFG-NAV5)
    // - minElev = 5 deg (search lower in horizon for more satellites)
    // - cnoThresh = 10 dB-Hz (down from 15-20 dB-Hz, tracks weak indoor/occluded signals)
    // - cnoThreshNumSVs = 3 (compute fix with only 3 SVs)
    // - fixMode = 3 (Auto 2D/3D with 2D fallback on 3 satellites)
    // - pDop = 25.0 (wide initial gate for fast geometric fix)
    static const uint8_t CMD_HIGH_SENSITIVITY_NAV5[] = {
        0xB5, 0x62, 0x06, 0x24, 0x24, 0x00,
        0x17, 0x01, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x03, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xD3, 0x9C
    };
    uart_->write(CMD_HIGH_SENSITIVITY_NAV5, sizeof(CMD_HIGH_SENSITIVITY_NAV5));

    // 4. Inject Assisted-GPS position seed (collapses Doppler & satellite search space)
    if (est_lat != 0.0 || est_lon != 0.0) {
        seed_assisted_gps(est_lat, est_lon, est_alt);
    }

    // 5. SkyTraq and PMTK NMEA rate configuration fallback
    static const char PMTK_SET_NMEA_OUTPUT[] = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
    uart_->write(reinterpret_cast<const uint8_t*>(PMTK_SET_NMEA_OUTPUT), strlen(PMTK_SET_NMEA_OUTPUT));

    ESP_LOGI(TAG, "GNSS high-sensitivity fast-lock configuration dispatched (10dB-Hz threshold, 5 deg mask, Auto-2D/3D)");
}

void NGPS01::configure_sbas_gagan() noexcept {
    if (!uart_) return;

    // 1. SkyTraq / NavIC N-GS-01: Configure Output Message Format -> NMEA (Message ID 0x09)
    static const uint8_t CMD_ENABLE_NMEA[] = {
        0xA0, 0xA1, 0x00, 0x03, 0x09, 0x01, 0x01, 0x09, 0x0D, 0x0A
    };
    uart_->write(CMD_ENABLE_NMEA, sizeof(CMD_ENABLE_NMEA));

    // 2. SkyTraq / NavIC N-GS-01: Configure SBAS / GAGAN (Message ID 0x62 VENUS8_EXT2, Sub-ID 0x01)
    // Payload: [0x62, 0x01, Enable=1, Nav=1, Range=8, Corr=1, Channels=3, Subsystem=7(WAAS/EGNOS/MSAS/GAGAN), Save=1]
    static const uint8_t CMD_ENABLE_SKYTRAQ_SBAS[] = {
        0xA0, 0xA1, 0x00, 0x09, 0x62, 0x01, 0x01, 0x01, 0x08, 0x01, 0x03, 0x07, 0x01, 0x6F, 0x0D, 0x0A
    };
    uart_->write(CMD_ENABLE_SKYTRAQ_SBAS, sizeof(CMD_ENABLE_SKYTRAQ_SBAS));

    // 3. u-blox NEO-6M / 7M / M8N: UBX-CFG-SBAS (Class 0x06, ID 0x16, Len 8)
    // Payload: mode=1 (Enable), usage=7 (Ranging + DiffCorr + Integrity), maxSBAS=3, scanmode=0 (Auto-scan GAGAN PRN 127/128/132)
    // Checksum: CK_A=0x2F, CK_B=0xD5
    static const uint8_t CMD_ENABLE_UBX_SBAS[] = {
        0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x01, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2F, 0xD5
    };
    uart_->write(CMD_ENABLE_UBX_SBAS, sizeof(CMD_ENABLE_UBX_SBAS));

    // 4. u-blox NEO-6M: UBX-CFG-CFG Save Configuration to Flash/BBR (Class 0x06, ID 0x09)
    static const uint8_t CMD_SAVE_UBX_CFG[] = {
        0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x52, 0xE7
    };
    uart_->write(CMD_SAVE_UBX_CFG, sizeof(CMD_SAVE_UBX_CFG));

    // 5. Fallback NMEA PMTK commands for generic SBAS/DGPS receivers (MediaTek MT3339 etc.)
    static const char PMTK_SBAS[] = "$PMTK313,1*2E\r\n";
    static const char PMTK_DGPS[] = "$PMTK301,2*2E\r\n";
    uart_->write(reinterpret_cast<const uint8_t*>(PMTK_SBAS), strlen(PMTK_SBAS));
    uart_->write(reinterpret_cast<const uint8_t*>(PMTK_DGPS), strlen(PMTK_DGPS));

    ESP_LOGI(TAG, "Sent GAGAN / SBAS activation packets (SkyTraq + u-blox UBX + PMTK) to GNSS receiver");
}

void NGPS01::process_sentence(const char* s) noexcept {
    const char* p = strchr(s, '$');
    if (!p) return;

    if (strstr(p, "GGA") != nullptr) {
        parse_gga(p, cached_);
    } else if (strstr(p, "RMC") != nullptr) {
        parse_rmc(p, cached_);
    } else if (strstr(p, "GSV") != nullptr) {
        parse_gsv(p, cached_);
    } else if (strstr(p, "VTG") != nullptr) {
        parse_vtg(p, cached_);
    }
}

GNSSData NGPS01::read() noexcept {
    if (!ready_ || !uart_) return cached_;

    uint8_t buf[128];
    int n;
    while ((n = uart_->read(buf, sizeof(buf), 0)) > 0) {
        for (int i = 0; i < n; ++i) {
            char ch = static_cast<char>(buf[i]);
            if (ch == '$') {
                line_idx_ = 0;
                line_buf_[line_idx_++] = '$';
            } else if (line_idx_ > 0 && line_idx_ < (int)sizeof(line_buf_) - 1) {
                line_buf_[line_idx_++] = ch;
                if (ch == '\n' || ch == '\r') {
                    line_buf_[line_idx_] = '\0';
                    process_sentence(line_buf_);
                    line_idx_ = 0;
                }
            }
        }
    }
    return cached_;
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

    // Parse time (HHMMSS.ss)
    const char* tstr = tok[1];
    int hh = 0, mm = 0; double ss = 0.0;
    if (strlen(tstr) >= 6) {
        hh = (tstr[0]-'0')*10 + (tstr[1]-'0');
        mm = (tstr[2]-'0')*10 + (tstr[3]-'0');
        ss = strtod(tstr + 4, nullptr);
        out.gnss_time_s = hh*3600 + mm*60 + ss;
        snprintf(out.time_str, sizeof(out.time_str), "%02d:%02d:%02d", hh, mm, static_cast<int>(ss));
    }

    // tok[2]=lat, tok[3]=N/S, tok[4]=lon, tok[5]=E/W
    // tok[6]=fix quality, tok[7]=satellites, tok[8]=HDOP, tok[9]=altitude
    double lat = nmea_lat(tok[2], tok[3][0]);
    double lon = nmea_lon(tok[4], tok[5][0]);
    double alt = strtod(tok[9], nullptr);
    int fix    = atoi(tok[6]);
    int sats   = atoi(tok[7]);

    out.satellites   = sats;
    out.fix_quality  = fix;
    out.gagan_active = (fix == 2);
    if (out.gagan_active) {
        ESP_LOGI(TAG, "GAGAN/SBAS differential correction ACTIVE (Fix quality 2, Sats: %d)", sats);
    }

    if (lat != 0.0 || lon != 0.0) {
        out.lat_deg   = lat;
        out.lon_deg   = lon;

        // 2.5D Baro-Assisted GNSS:
        // When in 2D fix (sats == 3) or GPS altitude is 0, fuse with barometric altitude aiding
        if ((alt == 0.0 || sats < 4) && baro_alt_aiding_m_ != 0.0) {
            out.alt_msl_m = baro_alt_aiding_m_;
        } else {
            out.alt_msl_m = alt;
        }

        out.fix_3d    = (sats >= 4 && fix > 0);
        out.valid     = (fix > 0 || sats >= 3);

        // Set origin on first valid fix
        if (!origin_set_ && out.valid) {
            ref_lat_rad_ = lat * DEG2RAD;
            ref_lon_rad_ = lon * DEG2RAD;
            ref_alt_m_   = out.alt_msl_m;
            origin_set_  = true;
            ESP_LOGI(TAG, "Origin set: lat=%.6f lon=%.6f alt=%.1f (sats=%d, fix=%d)",
                     lat, lon, out.alt_msl_m, sats, fix);
        }

        double e, n2, u;
        lla_to_enu(lat, lon, out.alt_msl_m, e, n2, u);
        out.pos_e = e;
        out.pos_n = n2;
        out.pos_u = u;

        // Estimate vertical velocity from altitude derivative
        if (prev_time_s_ > 0.0) {
            double dt = out.gnss_time_s - prev_time_s_;
            if (dt > 0.5 && dt < 3.0) {
                out.vel_u = (out.alt_msl_m - prev_alt_msl_) / dt;
            }
        }
        prev_time_s_  = out.gnss_time_s;
        prev_lat_deg_ = lat;
        prev_lon_deg_ = lon;
        prev_alt_msl_ = out.alt_msl_m;
        return true;
    } else {
        out.valid  = false;
        out.fix_3d = false;
        return true;
    }
}

// NMEA GPRMC: $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,,,a*hh
bool NGPS01::parse_rmc(const char* s, GNSSData& out) noexcept {
    if (!verify_checksum(s)) return false;
    char buf[128]; strncpy(buf, s, 127); buf[127] = '\0';
    char* tok[20] = {}; int n = 0;
    char* p = buf;
    while (n < 20 && p) { tok[n++] = p; p = strchr(p, ','); if (p) *p++ = '\0'; }
    if (n < 7) return false;

    // Parse time
    const char* tstr = tok[1];
    int hh = 0, mm = 0; double ss = 0.0;
    if (strlen(tstr) >= 6) {
        hh = (tstr[0]-'0')*10 + (tstr[1]-'0');
        mm = (tstr[2]-'0')*10 + (tstr[3]-'0');
        ss = strtod(tstr + 4, nullptr);
        out.gnss_time_s = hh*3600 + mm*60 + ss;
        snprintf(out.time_str, sizeof(out.time_str), "%02d:%02d:%02d", hh, mm, static_cast<int>(ss));
    }

    if (tok[2][0] == 'A') {
        double lat = nmea_lat(tok[3], tok[4][0]);
        double lon = nmea_lon(tok[5], tok[6][0]);
        if (lat != 0.0 || lon != 0.0) {
            out.lat_deg = lat;
            out.lon_deg = lon;
            out.valid   = true;
            if (out.satellites < 3) out.satellites = 3;
            if (out.fix_quality == 0) out.fix_quality = 1;
            if (baro_alt_aiding_m_ != 0.0 && out.alt_msl_m == 0.0) {
                out.alt_msl_m = baro_alt_aiding_m_;
            }
            return true;
        }
    }
    return true;
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

bool NGPS01::parse_gsv(const char* s, GNSSData& out) noexcept {
    if (!verify_checksum(s)) return false;
    char buf[128]; strncpy(buf, s, 127); buf[127] = '\0';
    char* tok[20] = {}; int n = 0;
    char* p = buf;
    while (n < 20 && p) { tok[n++] = p; p = strchr(p, ','); if (p) *p++ = '\0'; }
    if (n < 4) return false;
    int sats_in_view = atoi(tok[3]);
    if (sats_in_view > 0 && out.fix_quality == 0) {
        out.satellites = sats_in_view;
    }
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
