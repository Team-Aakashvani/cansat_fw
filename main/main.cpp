/**
 * @file main.cpp
 * @brief CAN-7USAT 2026 CanSat — ESP32-S3 Flight Software Entry Point.
 */

#include "nav/config.hpp"
#include "nav/vertical_kf.hpp"
#include "nav/mission.hpp"
#include "nav/guidance.hpp"

#include "app_hal/i2c_bus.hpp"
#include "app_hal/spi_bus.hpp"
#include "app_hal/uart_bus.hpp"

#include "drivers/bno085.hpp"
#include "drivers/bmp585.hpp"
#include "drivers/ngps01.hpp"
#include "drivers/ina260.hpp"
#include "drivers/max17048.hpp"
#include "drivers/sdp31.hpp"
#include "drivers/sht4x.hpp"
#include "drivers/sgp41.hpp"

#include "control/steer_controller.hpp"
#include "drivers/dshot.hpp"
#include "drivers/servo.hpp"

#include "telemetry/encoder.hpp"

#include "comms/xbee_link.hpp"
#include "comms/command_parser.hpp"
#include "comms/ota_service.hpp"

#include "cli/console.hpp"
#include "logging/sd_logger.hpp"
#include "logging/event_log.hpp"
#include "logging/coredump_exporter.hpp"
#include "logging/flight_recorder.hpp"
#include "comms/ble_link.hpp"
#include "esp_core_dump.h"
#include "esp_heap_caps.h"

#include "power/power_manager.hpp"
#include "config_mgr/nvs_config.hpp"
#include "watchdog/watchdog.hpp"
#include "bit/built_in_test.hpp"
#include "rf_mapping/rf_mapper.hpp"

#include "cli/test_suite.hpp"
#include "driver/gpio.h"
#include "esp_cpu.h"
#include "soc/gpio_struct.h"
#include "esp_rom_sys.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_attr.h"

#include <cmath>
#include <cstring>
#include <cstdlib>
#include <cctype>
#include <cstdio>
#include <atomic>

extern void main_gcs();

static const char* TAG = "main";

// ===========================================================================
// FreeRTOS task priorities
// ===========================================================================
#define PRI_IMU_TASK     (configMAX_PRIORITIES - 1)
#define PRI_NAV_TASK     (configMAX_PRIORITIES - 2)
#define PRI_CTRL_TASK    (configMAX_PRIORITIES - 3)
#define PRI_SENSOR_TASK  (configMAX_PRIORITIES - 3)
#define PRI_TELEM_TASK   5
#define PRI_LOGGING_TASK 4
#define PRI_POWER_TASK   3

#define STK_IMU      6144
#define STK_NAV      8192     // was 32 KB for the IMM; measured use ~1.5 KB
#define STK_CTRL     10240
#define STK_SENSOR   10240
#define STK_TELEM    12288
#define STK_LOGGING  8192
#define STK_POWER    4096

#define EVT_BIT_PASS        (1u << 0)

// ===========================================================================
// Global subsystem instances
// ===========================================================================

static hal::I2CBus    i2c0;
static hal::I2CBus    i2c1;
static hal::UARTBus   xbee_uart;
static hal::SPIBus    spi;
static hal::UARTBus   uart;

static drivers::BNO085   imu_drv;
static drivers::BMP585   baro_drv;
static drivers::NGPS01   gnss_drv;
static drivers::SDP31    sdp31_drv;
static drivers::SHT4x    sht4x_drv;
static drivers::SGP41    sgp41_drv;
static drivers::CC1101   scan_drv;

// ---- Flight stack: vertical filter + mission supervisor + guidance + steering ----
static nav::VerticalKF          vkf;          ///< nav_task (under fc_mutex)
static nav::MissionSupervisor   mission;      ///< nav_task / commands (under fc_mutex)
static nav::SiteEstimator       site;         ///< nav_task (under fc_mutex)
static nav::ReturnGuidance      guidance;     ///< control_task only
static control::SteerController steer;        ///< control_task only
static drivers::DShot           esc;          ///< control_task only (after boot)
static drivers::ServoPair       latch_servos; ///< control_task / commands

/// Survives brownout / watchdog / panic resets (not power-on): mid-air resume
RTC_NOINIT_ATTR static nav::MissionPersist g_persist;

/// Published by nav_task, read by control / telemetry / indicator tasks (fc_mutex)
struct NavSnapshot {
    nav::MissionSupervisor::Outputs msn{};
    nav::VerticalKF::Output         vk{};
    bool   site_valid = false;
    double site_lat = 0.0, site_lon = 0.0;
    bool   valid = false;
};
static NavSnapshot g_nav{};

/// Published by control_task for telemetry (fc_mutex)
struct CtrlSnapshot {
    nav::ReturnGuidance::Output guide{};
    float heading_enu_deg = 0.0f;
    uint8_t esc_state = 0;            ///< 0 silent, 1 arming, 2 spin-up, 3 running, 4 stopped
    float throttle[4] = {0, 0, 0, 0};
};
static CtrlSnapshot g_ctrl{};

static std::atomic<double> latest_gnss_rx_s{-1.0};     ///< MCU time a new GNSS fix arrived
static logging::FlightRecorder recorder;               ///< 12 MB flash flight log
static comms::BleLink          ble;                    ///< Bluetooth LE health / command link
static QueueHandle_t           g_ble_cmd_q = nullptr;  ///< BLE lines -> ble_cmd_task
static std::atomic<float>      g_imu_hz{0.0f};
static std::atomic<uint32_t>   g_bit_flags{0};
static std::atomic<bool>       g_crash_present{false};
static std::atomic<float>  g_heading_offset_deg{0.0f}; ///< NORTH calibration (NVS)
static std::atomic<bool>   g_manual_unlatch{false};    ///< CHUTE command: open the arms now

/// Bench actuator tests (PAD only, never once flight has started)
struct BenchRequest { int mode = 0; int motor = -1; float throttle = 0.0f; int64_t until_us = 0; };
static BenchRequest g_bench{};                          ///< under fc_mutex
static bool         g_resumed_in_flight = false;

static comms::XBeeLink      xbee;
static comms::CommandParser cmd_parser;
static comms::OTAService    ota_svc;

static telemetry::TelemetryEncoder telem_enc;
static logging::SDLogger   sd_logger;
static logging::EventLog   event_log;
static rf_mapping::RFMapper rf_mapper;
static power::PowerManager pwr_mgr;
static config_mgr::NVSConfig nvs_cfg;
static watchdog::Watchdog    wdg;
static cli::Console console(nvs_cfg, cmd_parser);

static SemaphoreHandle_t fc_mutex     = nullptr;
static EventGroupHandle_t evt_group   = nullptr;
static SemaphoreHandle_t sensor_mutex = nullptr;

static std::atomic<uint32_t> packet_count{0};
// Mission time is derived from the MCU's own clock (esp_timer), never from loop
// counting, so the telemetry timestamp always matches real elapsed time.
// ST,<hh:mm:ss> moves the epoch; it does not stop the clock.
static std::atomic<int64_t>  mission_epoch_us{0};
static std::atomic<bool>     telem_enabled{true};   ///< USB / Bluetooth stream (paused during log dumps)
/// XBee downlink. Guidelines: no telemetry transmission until the ground station commands it (CX,ON)
static std::atomic<bool>     radio_enabled{false};

static drivers::BaroData       latest_baro{};
static drivers::GNSSData       latest_gnss{};
static drivers::IMUData        latest_imu_snap{};
static uint32_t                latest_imu_seq = 0;   ///< Bumped per new IMU sample (under sensor_mutex)
static drivers::PowerData      latest_pwr{};
static drivers::HumidData      latest_sht4x{};
static drivers::AirQualityData latest_sgp41{};

static inline double now_s() noexcept {
    return (double)esp_timer_get_time() * 1.0e-6;
}

static inline uint32_t mission_time_now_s() noexcept {
    const int64_t dt_us = esp_timer_get_time() - mission_epoch_us.load();
    return dt_us > 0 ? (uint32_t)(dt_us / 1000000) : 0u;
}

/// World-up component of the vehicle-frame specific force (m/s^2), minus g
static inline float accel_up_minus_g(const drivers::IMUData& m) noexcept {
    const double w = m.quat_w, x = m.quat_x, y = m.quat_y, z = m.quat_z;   // vehicle -> world
    const double up = 2.0*(x*z - w*y) * m.acc_x + 2.0*(y*z + w*x) * m.acc_y
                    + (1.0 - 2.0*(x*x + y*y)) * m.acc_z;
    return (float)(up - nav::G0_MPS2);
}

/// Angle between vehicle +Z and world up (deg)
static inline float tilt_from_quat_deg(const drivers::IMUData& m) noexcept {
    const double c = 1.0 - 2.0*(m.quat_x*m.quat_x + m.quat_y*m.quat_y);
    return (float)(std::acos(std::max(-1.0, std::min(1.0, c))) * 57.29578);
}

/// Heading of vehicle +X in the fusion world frame (rad, CCW), before NORTH offset
static inline float heading_world_rad(const drivers::IMUData& m) noexcept {
    const double w = m.qw_world, x = m.qx_world, y = m.qy_world, z = m.qz_world;
    return (float)std::atan2(2.0*(x*y + w*z), 1.0 - 2.0*(y*y + z*z));
}

/// 100 Hz IMU acquisition + attitude reference (highest priority on core 0).
static void imu_task(void* /*arg*/) {
    watchdog::Watchdog::register_task();
    const TickType_t period = pdMS_TO_TICKS(10);
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t n_ok = 0, n_fail = 0;
    int64_t rd_sum = 0, rd_max = 0, win_t0 = esp_timer_get_time();

    while (true) {
        watchdog::Watchdog::ping();
        const int64_t rd_t0 = esp_timer_get_time();
        drivers::IMUData imu = imu_drv.read();
        const int64_t rd = esp_timer_get_time() - rd_t0;

        if (imu.valid) {
            xSemaphoreTake(sensor_mutex, portMAX_DELAY);
            latest_imu_snap = imu;
            ++latest_imu_seq;
            xSemaphoreGive(sensor_mutex);
        }

        // IMU health: sample rate and bus read time, logged every 5 s
        (imu.valid ? n_ok : n_fail)++; rd_sum += rd; if (rd > rd_max) rd_max = rd;
        if (rd_t0 - win_t0 >= 5000000) {
            const double win_s = (double)(rd_t0 - win_t0) * 1e-6;
            g_imu_hz.store((float)(n_ok / win_s));
            ESP_LOGI("IMU", "%.1f Hz (%lu fail), read avg %.2f ms max %.2f ms",
                     n_ok / win_s, (unsigned long)n_fail,
                     (double)rd_sum / (double)(n_ok + n_fail) * 1e-3, (double)rd_max * 1e-3);
            n_ok = n_fail = 0; rd_sum = rd_max = 0; win_t0 = rd_t0;
        }
        vTaskDelayUntil(&last_wake, period);
    }
}

/// Vertical filter + mission supervisor (100 Hz on core 0, single actuator authority)
static void nav_task(void* /*arg*/) {
    watchdog::Watchdog::register_task();
    const TickType_t period = pdMS_TO_TICKS(10);
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t seen_imu_seq = 0;
    double last_imu_t = -1.0, last_baro_t = -1.0, last_step_t = -1.0, last_fix_rx = -1.0;
    bool locked_logged = false;
    bool last_clipped = false;
    int  ble_state = -1;

    while (true) {
        watchdog::Watchdog::ping();
        const double t = now_s();

        drivers::IMUData imu; drivers::BaroData baro; drivers::GNSSData gnss; uint32_t seq;
        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        imu = latest_imu_snap; seq = latest_imu_seq; baro = latest_baro; gnss = latest_gnss;
        xSemaphoreGive(sensor_mutex);

        xSemaphoreTake(fc_mutex, portMAX_DELAY);

        // ---- IMU: predict + launch/release evidence (every new sample) ----------
        if (imu.valid && seq != seen_imu_seq) {
            seen_imu_seq = seq;
            const float dt = (last_imu_t > 0.0) ? (float)(imu.timestamp_s - last_imu_t) : 0.0f;
            last_imu_t = imu.timestamp_s;
            const float fmag = (float)std::sqrt(imu.acc_x*imu.acc_x + imu.acc_y*imu.acc_y + imu.acc_z*imu.acc_z);
            const bool clipped = std::max({std::fabs(imu.acc_x), std::fabs(imu.acc_y), std::fabs(imu.acc_z)})
                                 >= nav::VERT_CFG.accel_clip_mps2;
            last_clipped = clipped;
            if (imu.quat_valid && dt > 0.0f) vkf.predict(accel_up_minus_g(imu), dt, clipped);
            mission.ingest_accel(imu.timestamp_s, fmag);
        }

        // ---- Baro (50 Hz): update + mission step --------------------------------
        if (baro.valid && baro.timestamp_s != last_baro_t) {
            last_baro_t = baro.timestamp_s;
            const nav::MissionPhase ph = mission.phase();
            const nav::VerticalConfig& V = nav::VERT_CFG;
            switch (ph) {
                case nav::MissionPhase::PAD:      vkf.set_noise(V.sigma_a_pad, V.baro_sigma_m); break;
                case nav::MissionPhase::ASCENT:   vkf.set_noise(V.sigma_a_ascent, V.baro_sigma_m); break;
                case nav::MissionPhase::STEERING: vkf.set_noise(V.sigma_a_steering, V.baro_sigma_steering_m); break;
                default:                          vkf.set_noise(V.sigma_a_descent, V.baro_sigma_m); break;
            }
            vkf.update_baro((float)baro.altitude_agl_m, baro.timestamp_s);
            const nav::VerticalKF::Output vk = vkf.output();

            const float rate = (float)std::sqrt(imu.gyr_x*imu.gyr_x + imu.gyr_y*imu.gyr_y + imu.gyr_z*imu.gyr_z);
            nav::MissionSupervisor::Inputs in{ t, vk.h_m, vk.v_mps,
                                               imu.quat_valid ? tilt_from_quat_deg(imu) : 180.0f,
                                               rate, imu.quat_valid };
            const float dt_step = (last_step_t > 0.0) ? (float)(t - last_step_t) : 0.02f;
            last_step_t = t;
            const nav::MissionSupervisor::Outputs msn = mission.step(in, dt_step);

            if (msn.event) {
                ESP_LOGW("MISSION", "[%s] %s | AGL %.1f m  v %.1f m/s  peak %.1f m",
                         nav::mission_phase_name(msn.phase), msn.event, (double)msn.agl_m,
                         (double)vk.v_mps, (double)msn.h_max_m);
                char ev[96];
                snprintf(ev, sizeof(ev), "%s @%.0fm", msn.event, (double)msn.agl_m);
                recorder.log_event((uint8_t)msn.phase, ev);
                logging::EventCode code = logging::EventCode::CUSTOM;
                if (strncmp(msn.event, "LAUNCH", 6) == 0)        code = logging::EventCode::LAUNCH_DETECT;
                else if (strncmp(msn.event, "RELEASED", 8) == 0) code = logging::EventCode::APOGEE;
                else if (strncmp(msn.event, "ARMS", 4) == 0)     code = logging::EventCode::DRONE_DEPLOY;
                else if (strncmp(msn.event, "LANDED", 6) == 0)   code = logging::EventCode::LANDED;
                event_log.log_event(code, mission_time_now_s(), (uint8_t)msn.phase, 0, 0, msn.event);
                char bl[128];
                snprintf(bl, sizeof(bl), "%03u,EVT,%s,%s", (unsigned)nav::TELEM_CFG.team_id,
                         nav::mission_phase_name(msn.phase), ev);
                ble.send_line(bl);
            }

            // Flight data record (50 Hz)
            {
                logging::NavRecord r{};
                r.phase = (uint8_t)msn.phase;
                r.t_ms = (uint32_t)(t * 1000.0);
                r.agl_m = msn.agl_m; r.vz_mps = vk.v_mps; r.baro_alt_m = (float)baro.altitude_agl_m;
                r.q[0] = (int16_t)(imu.quat_w * 30000.0); r.q[1] = (int16_t)(imu.quat_x * 30000.0);
                r.q[2] = (int16_t)(imu.quat_y * 30000.0); r.q[3] = (int16_t)(imu.quat_z * 30000.0);
                auto c16 = [](double v) { return (int16_t)std::max(-32767.0, std::min(32767.0, v)); };
                r.acc[0] = c16(imu.acc_x * 100.0); r.acc[1] = c16(imu.acc_y * 100.0); r.acc[2] = c16(imu.acc_z * 100.0);
                r.gyr[0] = c16(imu.gyr_x * 1000.0); r.gyr[1] = c16(imu.gyr_y * 1000.0); r.gyr[2] = c16(imu.gyr_z * 1000.0);
                r.lat_e7 = (int32_t)(gnss.lat_deg * 1e7); r.lon_e7 = (int32_t)(gnss.lon_deg * 1e7);
                r.gnss_alt_dm = c16(gnss.alt_msl_m * 10.0);
                r.sats = (uint8_t)std::max(0, std::min(255, gnss.satellites));
                r.flags = (uint8_t)((msn.arms_unlatched ? 1 : 0) | (vk.baro_held ? 2 : 0) | (last_clipped ? 4 : 0)
                                  | (msn.motors_suspended ? 8 : 0) | ((g_ctrl.esc_state & 7) << 4));
                for (int i = 0; i < 4; ++i) r.thr[i] = (uint8_t)std::max(0.0f, std::min(255.0f, g_ctrl.throttle[i] * 255.0f));
                r.tilt_sp[0] = (int8_t)std::max(-90.0f, std::min(90.0f, g_ctrl.guide.tilt_x_rad * 57.29578f));
                r.tilt_sp[1] = (int8_t)std::max(-90.0f, std::min(90.0f, g_ctrl.guide.tilt_y_rad * 57.29578f));
                recorder.log_nav(r);
            }

            // Flash erase stalls both cores: no background erase while flying.
            // BLE (2.4 GHz, shared with the XBee) off during a real flight, back on after landing.
            const bool flying = msn.flight_started && msn.phase != nav::MissionPhase::LANDED;
            recorder.set_flight_active(flying);
            const int want_ble = (flying && !mission.lift_test()) ? 0 : 1;
            if (want_ble != ble_state) { ble_state = want_ble; ble.set_enabled(want_ble != 0); }
            if (msn.flight_started) {
                site.lock();
                imu_drv.attitude_ref().set_locked(true);   // reference frozen for the flight
                if (!locked_logged) { locked_logged = true;
                    ESP_LOGW("MISSION", "Flight latched: launch site %s, IMU reference locked",
                             site.valid() ? "captured" : "NOT captured (no GNSS) - steering will hold level"); }
            }

            // Persist everything a mid-air reset needs
            mission.save(g_persist);
            g_persist.baro_p0_pa     = baro_drv.get_sea_level_pressure();
            g_persist.site_valid     = site.valid();
            g_persist.site_lat_deg   = site.lat();
            g_persist.site_lon_deg   = site.lon();
            g_persist.site_alt_off_m = site.alt_offset();
            g_persist.seal();

            g_nav.msn = msn; g_nav.vk = vk; g_nav.valid = true;
            g_nav.site_valid = site.valid(); g_nav.site_lat = site.lat(); g_nav.site_lon = site.lon();
        }

        // ---- GNSS (~1 Hz): launch-site averaging on the pad, weak altitude aiding ---
        const double fix_rx = latest_gnss_rx_s.load();
        if (gnss.valid && fix_rx != last_fix_rx) {
            const float dt_fix = (last_fix_rx > 0.0) ? (float)(fix_rx - last_fix_rx) : 1.0f;
            last_fix_rx = fix_rx;
            if (!mission.flight_started())
                site.add(gnss.lat_deg, gnss.lon_deg, gnss.alt_msl_m, g_nav.vk.h_m, gnss.satellites, dt_fix);
            else if (site.valid() && gnss.satellites >= nav::VERT_CFG.gnss_min_sats)
                vkf.update_gnss_alt((float)(gnss.alt_msl_m - site.alt_offset()));
        }

        xSemaphoreGive(fc_mutex);
        vTaskDelayUntil(&last_wake, period);
    }
}

static void sensor_task(void* /*arg*/) {
    const TickType_t period = pdMS_TO_TICKS(20);

    while (true) {
        drivers::BaroData baro = baro_drv.read();
        if (baro.valid) {
            baro.timestamp_s = now_s();     // MCU time of the sample (vertical filter needs it)
            gnss_drv.set_baro_alt_aiding(baro.altitude_agl_m + nvs_cfg.get_ground_alt_m());
        }
        drivers::GNSSData gnss = gnss_drv.read();
        {
            static double s_last_fix_utc = -1.0;
            if (gnss.valid && gnss.gnss_time_s != s_last_fix_utc) {
                s_last_fix_utc = gnss.gnss_time_s;
                latest_gnss_rx_s.store(now_s());
            }
        }

        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        if (baro.valid) {
            latest_baro = baro;
        }

        // Always reflect live satellite count, fix quality, and UTC time in telemetry
        latest_gnss.satellites   = gnss.satellites;
        latest_gnss.fix_quality  = gnss.fix_quality;
        latest_gnss.gagan_active = gnss.gagan_active;
        if (gnss.gnss_time_s > 0.0) {
            latest_gnss.gnss_time_s = gnss.gnss_time_s;
            memcpy(latest_gnss.time_str, gnss.time_str, sizeof(latest_gnss.time_str));
        }

        if (gnss.valid || gnss.lat_deg != 0.0 || gnss.lon_deg != 0.0) {
            latest_gnss = gnss;
            // Cache valid position to NVS (throttled to once per session)
            static bool nvs_pos_saved = false;
            if (!nvs_pos_saved && gnss.lat_deg != 0.0 && gnss.lon_deg != 0.0) {
                nvs_cfg.set_last_pos(static_cast<float>(gnss.lat_deg), static_cast<float>(gnss.lon_deg));
                nvs_pos_saved = true;
            }
        }
        xSemaphoreGive(sensor_mutex);

        // Sample environmental sensors (SHT4x & SGP41) at 1 Hz (every 50 iterations of 20ms)
        static uint32_t env_div = 0;
        if (++env_div >= 50) {
            env_div = 0;
            drivers::HumidData sht = sht4x_drv.read();
            double t_comp = sht.valid ? sht.temperature_c : (baro.valid ? baro.temperature_c : 25.0);
            double rh_comp = sht.valid ? sht.humidity_pct : 50.0;
            drivers::AirQualityData sgp = sgp41_drv.read(t_comp, rh_comp);

            xSemaphoreTake(sensor_mutex, portMAX_DELAY);
            if (sht.valid) latest_sht4x = sht;
            if (sgp.valid) latest_sgp41 = sgp;
            xSemaphoreGive(sensor_mutex);

            if (sht.valid || sgp.valid) {
                ESP_LOGI("ENV", "SHT4x: T=%.2f°C RH=%.1f%% | SGP41: VOC_idx=%u NOx_idx=%u",
                         sht.valid ? sht.temperature_c : 0.0,
                         sht.valid ? sht.humidity_pct : 0.0,
                         sgp.valid ? sgp.voc_index : 100,
                         sgp.valid ? sgp.nox_index : 1);
                printf("%03u,ENV,%.2f,%.1f,%u,%u\n", (unsigned)nav::TELEM_CFG.team_id,
                       sht.valid ? sht.temperature_c : 0.0,
                       sht.valid ? sht.humidity_pct : 0.0,
                       sgp.valid ? sgp.voc_index : 100,
                       sgp.valid ? sgp.nox_index : 1);
                fflush(stdout);
                char el[64];
                snprintf(el, sizeof(el), "%03u,ENV,%.2f,%.1f,%u,%u", (unsigned)nav::TELEM_CFG.team_id,
                         sht.valid ? sht.temperature_c : 0.0, sht.valid ? sht.humidity_pct : 0.0,
                         sgp.valid ? sgp.voc_index : 100, sgp.valid ? sgp.nox_index : 1);
                ble.send_line(el);
            }
        }

        vTaskDelay(period);
    }
}

/// Actuators: arm-latch servos + DShot motors (100 Hz on core 0).
/// Frames are only sent from here, so a stalled task means no frames and the ESC's own
/// signal-loss failsafe stops the motors.
static void control_task(void* /*arg*/) {
    watchdog::Watchdog::register_task();
    const TickType_t period = pdMS_TO_TICKS(10);
    TickType_t last_wake = xTaskGetTickCount();
    const nav::ActuatorConfig& A = nav::ACT_CFG;

    enum EscState : uint8_t { SILENT = 0, ARMING = 1, SPINUP = 2, RUNNING = 3, STOPPED = 4 };
    EscState st = SILENT;
    int64_t st_t0 = 0;
    bool unlatched_logged = false;

    while (true) {
        watchdog::Watchdog::ping();
        const int64_t now_us = esp_timer_get_time();
        const double t = now_us * 1e-6;
        const float dt = (float)nav::CONTROL_CFG.pid_dt_s;

        drivers::IMUData imu; drivers::GNSSData gnss;
        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        imu = latest_imu_snap; gnss = latest_gnss;
        xSemaphoreGive(sensor_mutex);
        NavSnapshot nv; BenchRequest bench;
        xSemaphoreTake(fc_mutex, portMAX_DELAY);
        nv = g_nav; bench = g_bench;
        if (bench.mode != 0 && (now_us > bench.until_us || nv.msn.flight_started)) { g_bench.mode = 0; bench.mode = 0; }
        xSemaphoreGive(fc_mutex);

        // ---- Arm-latch servos (latching: once open, stay open) ------------------
        const bool unlatch = nv.msn.arms_unlatched || g_manual_unlatch.load();
        latch_servos.set_us(unlatch ? A.servo_unlatch_us : A.servo_lock_us);
        if (unlatch && !unlatched_logged) { unlatched_logged = true; ESP_LOGW("ACT", "Arm latches -> UNLATCH"); }

        // ---- Motors -------------------------------------------------------------
        const bool flight_spin = nv.valid && nv.msn.motors_enabled;
        const bool bench_spin  = bench.mode != 0;
        const bool want_spin   = flight_spin || bench_spin;

        switch (st) {
            case SILENT:
                if (want_spin) {
                    st = ARMING; st_t0 = now_us; steer.reset(); guidance.reset();
                    ESP_LOGW("ACT", "ESC arming (DShot zero throttle)");
                }
                break;
            case ARMING:
                if (!want_spin) { st = STOPPED; st_t0 = now_us; }
                else if (now_us - st_t0 > (int64_t)(A.arm_zero_s * 1e6f)) { st = SPINUP; st_t0 = now_us; }
                break;
            case SPINUP:
                if (!want_spin) { st = STOPPED; st_t0 = now_us; }
                else if (now_us - st_t0 > (int64_t)(A.spinup_s * 1e6f)) { st = RUNNING; ESP_LOGW("ACT", "Motors running"); }
                break;
            case RUNNING:
                if (!want_spin) { st = STOPPED; st_t0 = now_us; ESP_LOGW("ACT", "Motors stopped"); }
                break;
            case STOPPED:
                if (want_spin) { st = SPINUP; st_t0 = now_us; steer.reset(); }
                // Not flying (bench / landed): drop the signal after 1 s so the ESC disarms
                else if (!nv.msn.steering_active && now_us - st_t0 > 1000000) { st = SILENT; }
                break;
        }

        uint16_t dshot[4] = {0, 0, 0, 0};
        float thr[4] = {0, 0, 0, 0};
        nav::ReturnGuidance::Output go{};
        float heading_enu = heading_world_rad(imu) + g_heading_offset_deg.load() * 0.01745329f;

        if (st == SPINUP || st == RUNNING) {
            const float ramp = (st == SPINUP) ? std::min(1.0f, (float)(now_us - st_t0) * 1e-6f / A.spinup_s) : 1.0f;
            if (bench.mode == 1) {                                   // single / all motor test
                for (int i = 0; i < 4; ++i)
                    thr[i] = (bench.motor < 0 || bench.motor == i) ? std::max(A.idle_throttle, bench.throttle * ramp) : 0.0f;
            } else {
                float sp_x = 0.0f, sp_y = 0.0f;
                if (flight_spin && nv.site_valid && imu.quat_valid) {
                    double se = 0.0, sn = 0.0;
                    nav::geo_to_en(gnss.lat_deg, gnss.lon_deg, nv.site_lat, nv.site_lon, se, sn);
                    const double rx = latest_gnss_rx_s.load();
                    const float age = (gnss.valid && rx > 0.0) ? (float)(t - rx) : 1e9f;
                    go = guidance.update((float)se, (float)sn, (float)gnss.vel_e, (float)gnss.vel_n, age, heading_enu, dt);
                    sp_x = go.tilt_x_rad * ramp; sp_y = go.tilt_y_rad * ramp;
                }
                const float coll = bench_spin ? std::min(bench.throttle, 0.30f) : A.base_throttle;
                const float collective = A.idle_throttle + (coll - A.idle_throttle) * ramp;
                const auto out = steer.update(sp_x, sp_y,
                                              (float)(imu.euler_pitch_deg * 0.01745329), (float)(imu.euler_roll_deg * 0.01745329),
                                              (float)imu.gyr_x, (float)imu.gyr_y, (float)imu.gyr_z, collective, dt);
                for (int i = 0; i < 4; ++i) thr[i] = out.m[i];
            }
            for (int i = 0; i < 4; ++i) dshot[i] = drivers::DShot::from_throttle(thr[i]);
        }
        if (st != SILENT) esc.write(dshot);                           // SILENT: no signal at all

        xSemaphoreTake(fc_mutex, portMAX_DELAY);
        g_ctrl.guide = go;
        g_ctrl.heading_enu_deg = heading_enu * 57.29578f;
        g_ctrl.esc_state = (uint8_t)st;
        for (int i = 0; i < 4; ++i) g_ctrl.throttle[i] = thr[i];
        xSemaphoreGive(fc_mutex);

        vTaskDelayUntil(&last_wake, period);
    }
}

static void telem_task(void* /*arg*/) {
    // 50 Hz tick: compact attitude line (ATT) every tick for the live 3D view,
    // full CAN-7USAT frame every FRAME_DIV ticks (25 Hz) to the console,
    // radio + SD decimated to 1 Hz.
    const TickType_t period = pdMS_TO_TICKS(20);
    constexpr uint32_t FRAME_DIV = 2;
    constexpr uint32_t RADIO_DIV = 25;   // frames per radio packet (25 Hz / 25 = 1 Hz)
    TickType_t last_wake = xTaskGetTickCount();
    char csv_buf[telemetry::TelemetryEncoder::BUF_LEN];
    uint32_t radio_div = 0;
    uint32_t frame_div = 0;

    while (true) {
        if (telem_enabled.load()) {
            drivers::IMUData att_snap;
            xSemaphoreTake(sensor_mutex, portMAX_DELAY);
            att_snap = latest_imu_snap;
            xSemaphoreGive(sensor_mutex);
            // ATT,<mcu_ms at IMU sample>,<qw>,<qx>,<qy>,<qz>,<tilt_x>,<tilt_y>,<rot_z>,<mount>,<ref_count>
            // q = vehicle -> heading-tared world (Three.js ZXY Euler alongside for display)
            printf("%03u,ATT,%lu,%.5f,%.5f,%.5f,%.5f,%.2f,%.2f,%.2f,%u,%lu\n",
                   (unsigned)nav::TELEM_CFG.team_id,
                   (unsigned long)(att_snap.timestamp_s * 1000.0),
                   att_snap.quat_w, att_snap.quat_x, att_snap.quat_y, att_snap.quat_z,
                   att_snap.euler_pitch_deg, att_snap.euler_roll_deg, att_snap.euler_yaw_deg,
                   att_snap.quat_valid ? (unsigned)att_snap.mount : 0u,
                   (unsigned long)imu_drv.attitude_ref().reference_count());
            static uint32_t ble_att_div = 0;
            if (ble.connected() && ++ble_att_div >= 5) {
                ble_att_div = 0;
                char al[140];
                snprintf(al, sizeof(al), "%03u,ATT,%lu,%.5f,%.5f,%.5f,%.5f,%.2f,%.2f,%.2f,%u,%lu",
                         (unsigned)nav::TELEM_CFG.team_id, (unsigned long)(att_snap.timestamp_s * 1000.0),
                         att_snap.quat_w, att_snap.quat_x, att_snap.quat_y, att_snap.quat_z,
                         att_snap.euler_pitch_deg, att_snap.euler_roll_deg, att_snap.euler_yaw_deg,
                         att_snap.quat_valid ? (unsigned)att_snap.mount : 0u,
                         (unsigned long)imu_drv.attitude_ref().reference_count());
                ble.send_line(al);
            }
        }
        if (telem_enabled.load() && ++frame_div >= FRAME_DIV) {
            frame_div = 0;
            drivers::BaroData baro_snap; drivers::GNSSData gnss_snap; drivers::IMUData imu_snap;
            drivers::PowerData pwr_snap; drivers::HumidData sht_snap; drivers::AirQualityData sgp_snap;
            NavSnapshot nav_snap; CtrlSnapshot ctrl_snap;
            bool lift_mode = false;
            {
                xSemaphoreTake(sensor_mutex, portMAX_DELAY);
                baro_snap = latest_baro; gnss_snap = latest_gnss; imu_snap = latest_imu_snap; pwr_snap = latest_pwr;
                sht_snap = latest_sht4x; sgp_snap = latest_sgp41;
                xSemaphoreGive(sensor_mutex);
            }
            if (sht_snap.valid) {
                baro_snap.temperature_c = (float)sht_snap.temperature_c;
            }
            {
                xSemaphoreTake(fc_mutex, portMAX_DELAY);
                nav_snap = g_nav; ctrl_snap = g_ctrl; lift_mode = mission.lift_test();
                xSemaphoreGive(fc_mutex);
            }
            const uint32_t pkt_cnt = ++packet_count;
            char state_buf[24];
            snprintf(state_buf, sizeof(state_buf), "%s%s", lift_mode ? "LIFT-" : "",
                     nav_snap.valid ? nav::mission_phase_name(nav_snap.msn.phase) : "BOOT");
            telemetry::TelemetryFrame frame = telemetry::TelemetryEncoder::make_frame(
                state_buf, nav_snap.msn.agl_m, baro_snap, gnss_snap, imu_snap, pwr_snap,
                sht_snap.valid ? (float)sht_snap.humidity_pct : 0.0f,
                sgp_snap.valid ? sgp_snap.voc_index : 0, sgp_snap.valid ? sgp_snap.nox_index : 0,
                pkt_cnt, now_s());

            // Health line (1 Hz): what you check on the pad / over Bluetooth
            static uint32_t hlt_div = 0, ble_frame_div = 0;
            if (++hlt_div >= 25) {
                hlt_div = 0;
                const logging::FlightRecorder::Status rs = recorder.status();
                char hl[200];
                snprintf(hl, sizeof(hl), "%03u,HLT,%lu,%.0f,%u,%d,%.2f,%u,%lu,%lu,%.1f,%08lX,%u,%u,%u,%lu",
                         (unsigned)nav::TELEM_CFG.team_id, (unsigned long)(esp_timer_get_time() / 1000000),
                         (double)g_imu_hz.load(), (unsigned)baro_snap.valid, gnss_snap.satellites,
                         (double)pwr_snap.voltage_v,
                         (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
                         (unsigned long)(rs.session_bytes / 1024), (unsigned long)(rs.erased_ahead / 1024),
                         (double)baro_snap.temperature_c, (unsigned long)g_bit_flags.load(),
                         (unsigned)ble.connected(), (unsigned)imu_snap.mount, (unsigned)g_crash_present.load(),
                         (unsigned long)rs.dropped);
                printf("%s\n", hl);
                ble.send_line(hl);
            }
            if (ble.connected() && ++ble_frame_div >= 25) {      // 1 Hz frame over BLE
                ble_frame_div = 0;
                char cb[telemetry::TelemetryEncoder::BUF_LEN];
                if (telem_enc.encode(frame, cb, sizeof(cb)) > 0) ble.send_line(cb);
            }

            // Mission line (5 Hz): phase, filtered AGL / speed / peak, actuators, guidance
            static uint32_t msn_div = 0;
            if (++msn_div >= 5) {
                msn_div = 0;
                char phase_buf[24];
                snprintf(phase_buf, sizeof(phase_buf), "%s%s", lift_mode ? "LIFT-" : "",
                         nav::mission_phase_name(nav_snap.msn.phase));
                printf("%03u,MSN,%s,%.1f,%.2f,%.1f,%u,%u,%.1f,%.0f,%d,%u,%.1f,%.1f,%.0f\n",
                       (unsigned)nav::TELEM_CFG.team_id, phase_buf,
                       (double)nav_snap.msn.agl_m, (double)nav_snap.vk.v_mps, (double)nav_snap.msn.h_max_m,
                       (unsigned)nav_snap.msn.arms_unlatched, (unsigned)ctrl_snap.esc_state,
                       (double)ctrl_snap.guide.dist_m, (double)ctrl_snap.guide.bearing_deg,
                       gnss_snap.satellites, (unsigned)nav_snap.vk.baro_held,
                       (double)(ctrl_snap.guide.tilt_x_rad * 57.29578f), (double)(ctrl_snap.guide.tilt_y_rad * 57.29578f),
                       (double)ctrl_snap.heading_enu_deg);
                if (ble.connected()) {
                    char ml[160];
                    snprintf(ml, sizeof(ml), "%03u,MSN,%s,%.1f,%.2f,%.1f,%u,%u,%.1f,%.0f,%d,%u,%.1f,%.1f,%.0f",
                             (unsigned)nav::TELEM_CFG.team_id, phase_buf,
                             (double)nav_snap.msn.agl_m, (double)nav_snap.vk.v_mps, (double)nav_snap.msn.h_max_m,
                             (unsigned)nav_snap.msn.arms_unlatched, (unsigned)ctrl_snap.esc_state,
                             (double)ctrl_snap.guide.dist_m, (double)ctrl_snap.guide.bearing_deg,
                             gnss_snap.satellites, (unsigned)nav_snap.vk.baro_held,
                             (double)(ctrl_snap.guide.tilt_x_rad * 57.29578f), (double)(ctrl_snap.guide.tilt_y_rad * 57.29578f),
                             (double)ctrl_snap.heading_enu_deg);
                    ble.send_line(ml);
                }
            }
            int n = telem_enc.encode(frame, csv_buf, sizeof(csv_buf));
            if (n > 0) {
                char sd_buf[telemetry::TelemetryEncoder::BUF_LEN + 16];
                int sn = snprintf(sd_buf, sizeof(sd_buf), "%s\n", csv_buf);
                if (sn > 0) {
                    // Radio and SD at 1 Hz; the radio only after CX,ON from the ground station
                    if (++radio_div >= RADIO_DIV) {
                        radio_div = 0;
                        if (radio_enabled.load()) xbee.enqueue_packet(csv_buf, (size_t)n);
                        sd_logger.write_line(sd_buf);
                    }
                    // Real-time 25 Hz frame stream to the USB console for the Dock GUI
                    printf("%s", sd_buf);
                }
            }
        }
        fflush(stdout);
        vTaskDelayUntil(&last_wake, period);
    }
}

static void logging_task(void* /*arg*/) {
    const TickType_t lora_period = pdMS_TO_TICKS(1000), flush_period = pdMS_TO_TICKS(5000);
    TickType_t lora_wake = xTaskGetTickCount(), flush_wake = xTaskGetTickCount();
    while (true) {
        TickType_t now = xTaskGetTickCount();
        if ((now - lora_wake) >= lora_period) { lora_wake = now; xbee.spin(); }
        if ((now - flush_wake) >= flush_period) { flush_wake = now; sd_logger.flush(); }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void power_task(void* /*arg*/) {
    const TickType_t period = pdMS_TO_TICKS(1000);
    while (true) {
        pwr_mgr.update();
        power::PowerState ps = pwr_mgr.get_state();
        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        latest_pwr.voltage_v = (double)ps.voltage_v; latest_pwr.current_a = (double)ps.current_a;
        latest_pwr.power_w = (double)ps.power_w; latest_pwr.valid = ps.valid;
        xSemaphoreGive(sensor_mutex);
        vTaskDelay(period);
    }
}

#include "esp_attr.h"

// =========================================================================
// ONBOARD WS2812 RGB LED (GPIO 48) & TEST SUITE IMPLEMENTATION
// =========================================================================
IRAM_ATTR static void set_onboard_rgb(uint8_t r, uint8_t g, uint8_t b, bool force = false) {
    static uint8_t s_last_r = 255, s_last_g = 255, s_last_b = 255;
    if (!force && r == s_last_r && g == s_last_g && b == s_last_b) {
        return; // Hardware latches color; avoid redundant bitbanging to eliminate glitches
    }
    s_last_r = r;
    s_last_g = g;
    s_last_b = b;

    const gpio_num_t pin = GPIO_NUM_48;
    static bool inited = false;
    if (!inited) {
        gpio_config_t io_conf{};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << pin);
        gpio_config(&io_conf);
        gpio_set_level(pin, 0);
        inited = true;
    }

    uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;
    const uint32_t mask = (1UL << (48 - 32)); // GPIO48 is in out1

    static portMUX_TYPE s_rgb_mux = portMUX_INITIALIZER_UNLOCKED;
    taskENTER_CRITICAL(&s_rgb_mux);

    for (int i = 23; i >= 0; --i) {
        if (grb & (1 << i)) {
            GPIO.out1_w1ts.val = mask;
            uint32_t t = esp_cpu_get_cycle_count();
            while ((esp_cpu_get_cycle_count() - t) < 170) {}
            GPIO.out1_w1tc.val = mask;
            t = esp_cpu_get_cycle_count();
            while ((esp_cpu_get_cycle_count() - t) < 140) {}
        } else {
            GPIO.out1_w1ts.val = mask;
            uint32_t t = esp_cpu_get_cycle_count();
            while ((esp_cpu_get_cycle_count() - t) < 80) {}
            GPIO.out1_w1tc.val = mask;
            t = esp_cpu_get_cycle_count();
            while ((esp_cpu_get_cycle_count() - t) < 200) {}
        }
    }

    GPIO.out1_w1tc.val = mask;
    taskEXIT_CRITICAL(&s_rgb_mux);
    esp_rom_delay_us(300);
}

static std::atomic<bool> g_test_alt_led_active{false};
static std::atomic<bool> g_test_sensors_active{false};

namespace test_suite {

void run_test_alt_led(bool enable) noexcept {
    g_test_alt_led_active.store(enable);
    if (enable) {
        printf("\n=======================================================\n");
        printf("  [ALT-LED FLIGHT MISSION TEST MONITOR] ACTIVATED\n");
        printf("  Real-time altitude tracking & LED indicator status:\n");
        printf("    1. PAD           : Desk level   -> Calm Emerald Green (Standby)\n");
        printf("    2. ASCENT        : Lift >=0.35m -> Solid ORANGE LED\n");
        printf("    3. APOGEE        : Apex drop    -> PURPLE flash + Parachute Deploy\n");
        printf("    4. DRONE_DESCENT : Lower board  -> Solid CYAN LED\n");
        printf("    5. TOUCHDOWN     : On desk 1.2s -> Flashing RED Recovery Beacon\n");
        printf("=======================================================\n\n");
    } else {
        printf("[TEST] Alt-LED console stream paused. LED flight supervisor continues.\n");
    }
}

void run_test_sensors(bool enable) noexcept {
    g_test_sensors_active.store(enable);
}

void run_test_bit() noexcept {
    bit::BuiltInTest bit_runner;
    bit_runner.run(i2c0, spi, uart, xbee_uart, &sd_logger, nvs_cfg);
}

void run_test_motor(float throttle_pct) noexcept {
    // PROPS OFF. Motor 1 only, max 25 %, 2.5 s; refused once flight has started.
    throttle_pct = std::max(0.0f, std::min(25.0f, throttle_pct));
    xSemaphoreTake(fc_mutex, portMAX_DELAY);
    if (!mission.flight_started()) {
        g_bench.mode = 1; g_bench.motor = 0; g_bench.throttle = throttle_pct / 100.0f;
        g_bench.until_us = esp_timer_get_time() + 2500000;
    }
    xSemaphoreGive(fc_mutex);
    printf("[TEST] Motor 1 at %.0f %% for 2.5 s (DShot300). Props OFF!\n", (double)throttle_pct);
}

void run_test_led_scan() noexcept {
    printf("\n=======================================================\n");
    printf("   ESP32-S3 ONBOARD DISCRETE LED SCANNER (GPIO SCAN)   \n");
    printf("=======================================================\n");
    printf("Flashing candidate pins one by one (10 Hz toggle for 2.5s each).\n");
    printf("Look at the small green LED beside the red power LED:\n\n");

    const int test_pins[] = {43, 44, 2, 1, 10, 38, 21};
    const char* pin_names[] = {
        "GPIO 43 (UART0 TX / Standard TX LED)",
        "GPIO 44 (UART0 RX / Standard RX LED)",
        "GPIO 2  (Classic ESP32 Status LED)",
        "GPIO 1  (User Pin IO1)",
        "GPIO 10 (User Pin IO10)",
        "GPIO 38 (Alt RGB / Status)",
        "GPIO 21 (User Pin IO21)"
    };

    for (size_t i = 0; i < sizeof(test_pins)/sizeof(test_pins[0]); ++i) {
        int pin = test_pins[i];
        printf(">>> [TESTING PIN] %s ... <<<\n", pin_names[i]);
        gpio_reset_pin((gpio_num_t)pin);
        gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);

        // Flash 12 cycles (100ms LOW, 100ms HIGH)
        for (int k = 0; k < 12; ++k) {
            gpio_set_level((gpio_num_t)pin, 0); // Active-low ON
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level((gpio_num_t)pin, 1); // Active-low OFF / Active-high ON
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        gpio_set_level((gpio_num_t)pin, 0);
    }

    printf("\n[LED SCAN] Scan complete! Note which GPIO pin flashed the green LED.\n\n");
}

void stop_all_tests() noexcept {
    g_test_alt_led_active.store(false);
    g_test_sensors_active.store(false);
    xSemaphoreTake(fc_mutex, portMAX_DELAY); g_bench.mode = 0; xSemaphoreGive(fc_mutex);
    printf("[TEST] All tests stopped. System returned to STANDBY green heartbeat.\n");
}

} // namespace test_suite

static void test_task(void* /*arg*/) {
    uint32_t log_div = 0;
    while (true) {
        if (g_test_alt_led_active.load()) {
            drivers::BaroData baro;
            xSemaphoreTake(sensor_mutex, portMAX_DELAY);
            baro = latest_baro;
            xSemaphoreGive(sensor_mutex);

            if (baro.valid && ++log_div >= 2) {
                log_div = 0;
                NavSnapshot nv;
                xSemaphoreTake(fc_mutex, portMAX_DELAY); nv = g_nav; xSemaphoreGive(fc_mutex);
                printf("[ALT-LED] %-12s | Baro: %+6.2fm | AGL: %+6.2fm | v: %+5.2fm/s | Peak: %+6.2fm\n",
                       nav::mission_phase_name(nv.msn.phase), baro.altitude_agl_m,
                       (double)nv.msn.agl_m, (double)nv.vk.v_mps, (double)nv.msn.h_max_m);
            }
        }

        if (g_test_sensors_active.load()) {
            drivers::IMUData imu;
            drivers::BaroData baro;
            drivers::HumidData sht;
            drivers::AirQualityData sgp;
            xSemaphoreTake(sensor_mutex, portMAX_DELAY);
            imu = latest_imu_snap;
            baro = latest_baro;
            sht = latest_sht4x;
            sgp = latest_sgp41;
            xSemaphoreGive(sensor_mutex);

            printf("[SENSORS] IMU(R=%+5.1f P=%+5.1f Y=%+5.1f) | Alt=%+6.2fm P=%.0fPa | T=%.1fC RH=%.1f%% | VOC=%u NOx=%u\n",
                   imu.euler_roll_deg, imu.euler_pitch_deg, imu.euler_yaw_deg,
                   baro.altitude_agl_m, baro.pressure_pa,
                   sht.temperature_c, sht.humidity_pct,
                   sgp.voc_index, sgp.nox_index);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void cli_task(void* /*arg*/) { console.run(); }

// ---------------------------------------------------------------------------
// Boot sequence on the onboard RGB LED (main_fc only, before the tasks start)
//   white fade-in   power on            blue        sensors / buses coming up
//   violet breathe  IMU finding its mount (keep the CanSat still)
//   2x green        all checks passed   2x amber    warnings (e.g. power monitor not fitted)
//   3x red          IMU or barometer missing        1x magenta  resumed after an in-flight reset
// ---------------------------------------------------------------------------
static uint8_t s_led[3] = {0, 0, 0};

static void boot_led_set(uint8_t r, uint8_t g, uint8_t b) {
    s_led[0] = r; s_led[1] = g; s_led[2] = b;
    set_onboard_rgb(r, g, b, true);
}

static void boot_led_fade(uint8_t r, uint8_t g, uint8_t b, int ms) {
    const uint8_t r0 = s_led[0], g0 = s_led[1], b0 = s_led[2];
    const int steps = std::max(1, ms / 20);
    for (int i = 1; i <= steps; ++i) {
        const float k = (float)i / steps;
        boot_led_set((uint8_t)(r0 + (r - r0) * k), (uint8_t)(g0 + (g - g0) * k), (uint8_t)(b0 + (b - b0) * k));
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/// Breathing violet while the IMU aligns; call every loop iteration
static void boot_led_breathe() {
    const float ph = (float)(esp_timer_get_time() % 1600000) / 1600000.0f;
    const float k = 0.15f + 0.85f * 0.5f * (1.0f - std::cos(ph * 6.2831853f));
    boot_led_set((uint8_t)(70 * k), 0, (uint8_t)(110 * k));
}

static void boot_led_result(uint32_t bit_flags) {
    const uint32_t critical = bit::BIT_IMU_ABSENT | bit::BIT_BARO_ABSENT;
    uint8_t r = 0, g = 120, b = 30;  int n = 2;                     // pass: green
    if (bit_flags & critical)  { r = 160; g = 0;  b = 0;  n = 3; }   // missing sensor: red
    else if (bit_flags)        { r = 150; g = 80; b = 0; }           // warnings: amber
    boot_led_fade(0, 0, 0, 120);
    for (int i = 0; i < n; ++i) {
        boot_led_set(r, g, b); vTaskDelay(pdMS_TO_TICKS(170));
        boot_led_set(0, 0, 0); vTaskDelay(pdMS_TO_TICKS(170));
    }
}

/// Status indicator only (RGB LED + recovery buzzer). It never touches actuators:
/// the mission supervisor is the single authority for servos and motors.
static void beacon_task(void* /*arg*/) {
    const int BEACON_PIN = nav::PINS.beacon;
    gpio_reset_pin((gpio_num_t)BEACON_PIN);
    gpio_set_direction((gpio_num_t)BEACON_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)BEACON_PIN, 0);
    set_onboard_rgb(0, 40, 10, true);
    uint32_t tick = 0;

    while (true) {
        NavSnapshot nv;
        xSemaphoreTake(fc_mutex, portMAX_DELAY); nv = g_nav; xSemaphoreGive(fc_mutex);
        ++tick;
        bool buzz = false;
        switch (nv.msn.phase) {
            case nav::MissionPhase::PAD:         set_onboard_rgb(0, 40, 10); break;                 // green
            case nav::MissionPhase::ASCENT:      set_onboard_rgb(140, 60, 0); break;                // orange
            case nav::MissionPhase::DESCENT:     set_onboard_rgb(160, 0, 160); break;               // purple
            case nav::MissionPhase::ARMS_DEPLOY: set_onboard_rgb(0, 60, 160); break;                // blue
            case nav::MissionPhase::STEERING:    set_onboard_rgb(0, 100, 100); break;               // cyan
            case nav::MissionPhase::LANDED:
                buzz = (tick % 12) < 6;                                                            // 2 Hz
                set_onboard_rgb(buzz ? 180 : 0, 0, 0);
                break;
        }
        gpio_set_level((gpio_num_t)BEACON_PIN, buzz ? 1 : 0);
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

/// Lines received over Bluetooth: same console/command path as USB, but actuator and
/// bulk-flash commands are refused (anyone in radio range could otherwise send them).
static void ble_cmd_task(void* /*arg*/) {
    char line[160];
    while (true) {
        if (xQueueReceive(g_ble_cmd_q, line, portMAX_DELAY) != pdTRUE) continue;
        char up[160];
        size_t n = 0;
        for (; line[n] && n < sizeof(up) - 1; ++n) up[n] = (char)toupper((unsigned char)line[n]);
        up[n] = 0;
        static const char* const blocked[] = { ",MTR", ",MOTOR", ",PID", ",CHUTE", ",OTA",
                                                "LOG,DUMP", "LOG,ERASE", "TEST " };
        bool refuse = false;
        for (const char* b : blocked) if (strstr(up, b)) { refuse = true; break; }
        char reply[256];
        if (refuse) {
            snprintf(reply, sizeof(reply), "%03u,NAK,%s,not allowed over Bluetooth (use USB)",
                     (unsigned)nav::TELEM_CFG.team_id, line);
        } else {
            console.inject_line(line);
            snprintf(reply, sizeof(reply), "%03u,ACK,%s", (unsigned)nav::TELEM_CFG.team_id, line);
        }
        ble.send_line(reply);
    }
}

static void print_crash_summary() {
    size_t addr = 0, size = 0;
    if (esp_core_dump_image_get(&addr, &size) != ESP_OK) { printf("CRASH,none\n"); return; }
    esp_core_dump_summary_t* sum = (esp_core_dump_summary_t*)malloc(sizeof(esp_core_dump_summary_t));
    if (sum && esp_core_dump_get_summary(sum) == ESP_OK) {
        printf("CRASH,task %s,pc 0x%08lx,%lu bytes,backtrace", sum->exc_task, (unsigned long)sum->exc_pc, (unsigned long)size);
        for (uint32_t i = 0; i < sum->exc_bt_info.depth && i < 16; ++i)
            printf(" 0x%08lx", (unsigned long)sum->exc_bt_info.bt[i]);
        printf("%s\n", sum->exc_bt_info.corrupted ? " (corrupted)" : "");
        printf("CRASH,decode: xtensa-esp32s3-elf-addr2line -pfiaC -e build/cansat_fw.elf <addresses>\n");
    } else {
        printf("CRASH,present (%lu bytes) but summary unavailable\n", (unsigned long)size);
    }
    free(sum);
}

static void setup_command_handlers() {
    cmd_parser.on_cx([](bool enable) {
        radio_enabled.store(enable);
        ESP_LOGI("Cmd", "Radio telemetry (XBee) %s", enable ? "ON" : "OFF");
    });
    cmd_parser.on_st([](uint32_t t) {
        mission_epoch_us.store(esp_timer_get_time() - (int64_t)t * 1000000);
        ESP_LOGI("Cmd", "Mission time set to %u s", (unsigned)t);
    });
    cmd_parser.on_tare([]() {
        if (imu_drv.attitude_ref().locked()) { ESP_LOGW("Cmd", "TARE refused: IMU reference is locked for flight"); return; }
        imu_drv.request_attitude_tare();   // applied on the next IMU sample
    });
    cmd_parser.on_north([]() {
        // Vehicle +X is pointing at true north now: north = 90 deg CCW from east (ENU)
        drivers::IMUData m;
        xSemaphoreTake(sensor_mutex, portMAX_DELAY); m = latest_imu_snap; xSemaphoreGive(sensor_mutex);
        if (!m.quat_valid) { ESP_LOGW("Cmd", "NORTH refused: attitude not referenced yet"); return; }
        float off = 90.0f - heading_world_rad(m) * 57.29578f;
        off = std::fmod(off + 540.0f, 360.0f) - 180.0f;
        g_heading_offset_deg.store(off);
        nvs_cfg.set_heading_offset_deg(off);
        ESP_LOGW("Cmd", "NORTH calibrated: heading offset %.1f deg (stored)", (double)off);
    });
    cmd_parser.on_lift([](const char* arg) {
        // Lift test: whole mission sequence at building scale, motors hard-inhibited
        xSemaphoreTake(fc_mutex, portMAX_DELAY);
        if (mission.flight_started() && !mission.lift_test()) {
            xSemaphoreGive(fc_mutex);
            ESP_LOGW("Cmd", "LIFT refused: real flight in progress");
            return;
        }
        float deploy = 0.0f;
        if (strcasecmp(arg, "OFF") != 0) {
            deploy = (float)atof(arg);
            if (!(deploy >= 4.0f && deploy <= 300.0f)) deploy = 10.0f;
        }
        mission.set_lift_test(deploy);
        mission.reset();                       // back to PAD (keeps the lift setting)
        vkf = nav::VerticalKF{};
        imu_drv.attitude_ref().set_locked(false);
        g_manual_unlatch.store(false);
        xSemaphoreGive(fc_mutex);
        if (deploy > 0.0f)
            ESP_LOGW("Cmd", "LIFT TEST ON: launch = 4 m climb, release = 1.5 m sustained descent, "
                            "arms at %.1f m AGL, cut-off 3 m. MOTORS INHIBITED.", (double)deploy);
        else
            ESP_LOGW("Cmd", "LIFT TEST OFF: flight thresholds restored (mission reset to PAD)");
    });
    cmd_parser.on_log([](const char* arg) {
        char a[24]; snprintf(a, sizeof(a), "%s", arg ? arg : "");
        for (char* c = a; *c; ++c) *c = (char)toupper((unsigned char)*c);
        if (strncmp(a, "LIST", 4) == 0) {
            recorder.list_sessions();
        } else if (strncmp(a, "DUMP", 4) == 0) {
            const char* comma = strchr(a, ',');
            const int idx = comma ? atoi(comma + 1) : 1;
            const bool was = telem_enabled.exchange(false);   // quiet stream while dumping
            vTaskDelay(pdMS_TO_TICKS(60));
            recorder.dump_session(idx > 0 ? idx : 1);
            telem_enabled.store(was);
        } else if (strncmp(a, "ERASE", 5) == 0) {
            if (mission.flight_started() && !mission.lift_test()) { printf("LOG,ERASE refused in flight\n"); return; }
            recorder.request_erase_all();
            printf("LOG,ERASE started (about a minute)\n");
        } else if (strncmp(a, "CRASHCLR", 8) == 0) {
            esp_core_dump_image_erase(); g_crash_present.store(false);
            printf("CRASH,cleared\n");
        } else if (strncmp(a, "CRASH", 5) == 0) {
            print_crash_summary();
        } else {
            printf("LOG commands: LIST | DUMP[,n] | ERASE | CRASH | CRASHCLR\n");
        }
    });
    cmd_parser.on_cal([]() {
        xSemaphoreTake(fc_mutex, portMAX_DELAY);
        if (mission.flight_started() && !mission.lift_test()) {
            xSemaphoreGive(fc_mutex);
            ESP_LOGW("Cmd", "CAL refused: flight in progress");
            return;
        }
        drivers::BaroData snap;
        xSemaphoreTake(sensor_mutex, portMAX_DELAY); snap = latest_baro; xSemaphoreGive(sensor_mutex);
        if (snap.valid) {
            nvs_cfg.set_ground_alt_m(snap.altitude_agl_m);
            baro_drv.auto_calibrate_baseline(0.0, 10);
        }
        vkf = nav::VerticalKF{};
        mission.reset();
        imu_drv.attitude_ref().set_locked(false);
        g_manual_unlatch.store(false);
        xSemaphoreGive(fc_mutex);
        ESP_LOGI("Cmd", "Ground zero calibrated & mission reset to PAD.");
    });
    cmd_parser.on_sim([](const char* mode) {
        const bool en = (strcasecmp(mode, "ENABLE") == 0 || strcasecmp(mode, "ACTIVATE") == 0);
        if (!en && strcasecmp(mode, "DISABLE") != 0) return;
        xSemaphoreTake(fc_mutex, portMAX_DELAY);
        if (mission.flight_started() && !baro_drv.is_sim_mode()) {
            xSemaphoreGive(fc_mutex);
            ESP_LOGW("Cmd", "SIM refused: real flight in progress");
            return;
        }
        if (en) {
            // Start the simulation from the current real pressure: no step at the switch-over
            drivers::BaroData live;
            xSemaphoreTake(sensor_mutex, portMAX_DELAY); live = latest_baro; xSemaphoreGive(sensor_mutex);
            if (live.valid) baro_drv.inject_sim_pressure(live.pressure_pa);
        }
        baro_drv.set_sim_mode(en);
        vkf = nav::VerticalKF{};
        mission.reset();
        mission.set_sim_mode(en);
        imu_drv.attitude_ref().set_locked(false);
        xSemaphoreGive(fc_mutex);
        ESP_LOGI("Cmd", "Simulation Mode %s (mission reset to PAD)", en ? "ACTIVATED - send SIMP pressures" : "DISABLED");
    });
    cmd_parser.on_simp([](float pa) {
        baro_drv.inject_sim_pressure((double)pa);
        drivers::BaroData d = baro_drv.read();
        if (d.valid) d.timestamp_s = now_s();
        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        latest_baro = d;
        xSemaphoreGive(sensor_mutex);
    });
    cmd_parser.on_abort([]() {
        xSemaphoreTake(fc_mutex, portMAX_DELAY);
        mission.abort();
        g_bench.mode = 0;
        xSemaphoreGive(fc_mutex);
        ESP_LOGW("Cmd", "ABORT: motors cut for the rest of the flight (arms/servos unchanged)");
    });
    cmd_parser.on_chute([]() {
        // Manual arm unlatch (backup for the automatic 600 m deploy, or bench test)
        g_manual_unlatch.store(true);
        ESP_LOGW("Cmd", "MANUAL ARM UNLATCH commanded");
    });
    cmd_parser.on_motor([](int motor_id, float pct) {
        // Bench only, PROPS OFF: max 25 %, auto-stop after 3 s
        pct = std::max(0.0f, std::min(25.0f, pct));
        xSemaphoreTake(fc_mutex, portMAX_DELAY);
        if (mission.flight_started()) { xSemaphoreGive(fc_mutex); ESP_LOGW("Cmd", "MTR refused in flight"); return; }
        if (pct <= 0.0f) g_bench.mode = 0;
        else { g_bench.mode = 1; g_bench.motor = (motor_id >= 0 && motor_id < 4) ? motor_id : -1;
               g_bench.throttle = pct / 100.0f; g_bench.until_us = esp_timer_get_time() + 3000000; }
        xSemaphoreGive(fc_mutex);
        ESP_LOGI("Cmd", "MTR %s at %.0f %% (3 s, props OFF)", motor_id < 0 ? "ALL" : "single", (double)pct);
    });
    cmd_parser.on_pid([](bool start, float base_throttle) {
        // Bench stabilisation test (tethered / props-guarded), max 30 % collective, 20 s
        xSemaphoreTake(fc_mutex, portMAX_DELAY);
        if (mission.flight_started()) { xSemaphoreGive(fc_mutex); ESP_LOGW("Cmd", "PID test refused in flight"); return; }
        if (start) { g_bench.mode = 2; g_bench.throttle = std::min(base_throttle, 0.30f);
                     g_bench.until_us = esp_timer_get_time() + 20000000; }
        else g_bench.mode = 0;
        xSemaphoreGive(fc_mutex);
        ESP_LOGI("Cmd", "PID bench test %s", start ? "STARTED (20 s max)" : "STOPPED");
    });
    cmd_parser.on_rtl([]() {
        ESP_LOGI("Cmd", "RTL: return-to-launch is automatic once steering starts");
    });
    xbee.set_rx_callback(cmd_parser.make_rx_callback());
}

#include "system_init.hpp"

extern "C" void main_fc() {
    fc_mutex = xSemaphoreCreateMutex(); sensor_mutex = xSemaphoreCreateMutex(); evt_group = xEventGroupCreate();

    // Mid-air reset? Resume only after an abnormal reset with a valid in-flight record.
    {
        const esp_reset_reason_t rr = esp_reset_reason();
        const bool abnormal = (rr == ESP_RST_BROWNOUT || rr == ESP_RST_PANIC || rr == ESP_RST_INT_WDT ||
                               rr == ESP_RST_TASK_WDT || rr == ESP_RST_WDT);
        const bool in_flight = g_persist.valid() && g_persist.phase != (uint8_t)nav::MissionPhase::PAD &&
                               g_persist.phase != (uint8_t)nav::MissionPhase::LANDED;
        g_resumed_in_flight = abnormal && in_flight;
        if (!g_resumed_in_flight) std::memset(&g_persist, 0, sizeof(g_persist));
    }

    // USB-Serial-JTAG console (COM port of the native USB): buffered, interrupt-driven
    // driver so printf never busy-waits and stdin (CLI / dock commands) actually works.
    // With no host attached, writes time out after 50 ms once and are then dropped.
    {
        usb_serial_jtag_driver_config_t usb_cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
        usb_cfg.tx_buffer_size = 8192;
        usb_cfg.rx_buffer_size = 1024;
        if (usb_serial_jtag_driver_install(&usb_cfg) == ESP_OK) {
            usb_serial_jtag_vfs_use_driver();
            setvbuf(stdin, nullptr, _IONBF, 0);
        }
    }

    if (g_resumed_in_flight) {                         // in the air: one flash, no delay
        boot_led_set(140, 0, 120); vTaskDelay(pdMS_TO_TICKS(80)); boot_led_set(0, 0, 0);
    } else {
        boot_led_fade(60, 60, 60, 300);                // power on
        boot_led_fade(0, 30, 120, 200);                // sensors and buses
    }

    // 1. Bit-bang 10kHz slow scan to diagnose sensor ACKs even with weak pull-ups
    int sda = nav::PINS.i2c0_sda;
    int scl = nav::PINS.i2c0_scl;
    gpio_reset_pin((gpio_num_t)sda);
    gpio_reset_pin((gpio_num_t)scl);
    gpio_set_direction((gpio_num_t)sda, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction((gpio_num_t)scl, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_pullup_en((gpio_num_t)sda);
    gpio_pullup_en((gpio_num_t)scl);
    gpio_set_level((gpio_num_t)sda, 1);
    gpio_set_level((gpio_num_t)scl, 1);
    esp_rom_delay_us(500);

    auto bb_dly = []() { esp_rom_delay_us(50); };
    auto bb_start = [&]() {
        gpio_set_level((gpio_num_t)sda, 1); gpio_set_level((gpio_num_t)scl, 1); bb_dly();
        gpio_set_level((gpio_num_t)sda, 0); bb_dly();
        gpio_set_level((gpio_num_t)scl, 0); bb_dly();
    };
    auto bb_stop = [&]() {
        gpio_set_level((gpio_num_t)sda, 0); bb_dly();
        gpio_set_level((gpio_num_t)scl, 1); bb_dly();
        gpio_set_level((gpio_num_t)sda, 1); bb_dly();
    };
    auto bb_write_byte = [&](uint8_t byte) -> bool {
        for (int b = 7; b >= 0; --b) {
            gpio_set_level((gpio_num_t)sda, (byte >> b) & 1);
            bb_dly();
            gpio_set_level((gpio_num_t)scl, 1);
            bb_dly();
            gpio_set_level((gpio_num_t)scl, 0);
            bb_dly();
        }
        gpio_set_level((gpio_num_t)sda, 1);
        bb_dly();
        gpio_set_level((gpio_num_t)scl, 1);
        bb_dly();
        int ack = gpio_get_level((gpio_num_t)sda);
        gpio_set_level((gpio_num_t)scl, 0);
        bb_dly();
        return (ack == 0);
    };

    auto bb_read_reg = [&](uint8_t dev_addr, uint8_t reg) -> int {
        bb_start();
        if (!bb_write_byte(dev_addr << 1)) { bb_stop(); return -1; }
        if (!bb_write_byte(reg)) { bb_stop(); return -2; }
        bb_start();
        if (!bb_write_byte((dev_addr << 1) | 1)) { bb_stop(); return -3; }
        uint8_t data = 0;
        for (int b = 7; b >= 0; --b) {
            gpio_set_level((gpio_num_t)scl, 1); bb_dly();
            if (gpio_get_level((gpio_num_t)sda)) data |= (1 << b);
            gpio_set_level((gpio_num_t)scl, 0); bb_dly();
        }
        gpio_set_level((gpio_num_t)sda, 1); bb_dly();
        gpio_set_level((gpio_num_t)scl, 1); bb_dly();
        gpio_set_level((gpio_num_t)scl, 0); bb_dly();
        bb_stop();
        return data;
    };

    if (g_resumed_in_flight) ESP_LOGW("MAIN", "*** RESUMING MISSION AFTER IN-FLIGHT RESET (phase %u) ***", (unsigned)g_persist.phase);
    ESP_LOGI("BB_SCAN", "Starting slow Bit-Bang I2C scan (SDA=GPIO%d, SCL=GPIO%d)...", sda, scl);
    int bb_found = 0;
    uint8_t found_addrs[16] = {};
    for (uint8_t a = 1; a < 128 && !g_resumed_in_flight; ++a) {
        bb_start();
        bool ack = bb_write_byte(a << 1);
        bb_stop();
        if (ack) {
            if (bb_found < 16) found_addrs[bb_found] = a;
            ESP_LOGI("BB_SCAN", ">>> [FOUND] Sensor ACK at 0x%02X (%d)! <<<", a, a);
            bb_found++;
        }
    }
    ESP_LOGI("BB_SCAN", "Slow Bit-Bang scan complete. Found %d devices.", bb_found);

    // Identify discovered devices
    for (int i = 0; i < bb_found && i < 16; ++i) {
        uint8_t a = found_addrs[i];
        int r0 = bb_read_reg(a, 0x00);
        int r1 = bb_read_reg(a, 0x01);
        ESP_LOGI("BB_ID", "Addr 0x%02X: Reg[0x00]=0x%02X (%d), Reg[0x01]=0x%02X (%d)", a, r0, r0, r1, r1);
    }

    // 100 kHz: a 32-byte BNO055 burst takes ~3.5 ms (was ~14 ms at 25 kHz, which
    // stretched the 100 Hz nav loop to ~16 Hz and froze the live attitude).
    i2c0.init(I2C_NUM_0, nav::PINS.i2c0_sda, nav::PINS.i2c0_scl, 100000);
    if (nav::PINS.spi_devices_fitted) spi.init(nav::PINS.spi_mosi, nav::PINS.spi_miso, nav::PINS.spi_sck);

    // Auto-detect GNSS RX pin and baud rate across candidate pins & speeds (excluding I2C/IMU/PSRAM)
    // Only the GNSS RX pin: other candidates are now actuator pins (ESC / servos / XBee)
    static const int candidate_rx_pins[] = {nav::PINS.gnss_rx};
    static const int gnss_bauds[] = {9600, 115200};
    int locked_rx = nav::PINS.gnss_rx;
    int locked_baud = 9600;
    bool found_gnss = false;

    for (int rx : candidate_rx_pins) {
        if (found_gnss || g_resumed_in_flight) break;
        for (int b : gnss_bauds) {
            uart_driver_delete(UART_NUM_1);
            uart.init(UART_NUM_1, nav::PINS.gnss_tx, rx, b);
            uart.flush_rx();
            char nmea_test[128] = {};
            int n = uart.read_line(nmea_test, sizeof(nmea_test), 1100);
            if (n > 0) {
                ESP_LOGI("GNSS_SCAN", "GPIO %d @ %d baud rx %d bytes: '%.50s'", rx, b, n, nmea_test);
                if (strchr(nmea_test, '$') != nullptr || strstr(nmea_test, "G") != nullptr || n >= 10) {
                    locked_rx = rx;
                    locked_baud = b;
                    found_gnss = true;
                    ESP_LOGI("GNSS", "==> LOCKED GPS stream on GPIO %d at %d baud! <==", rx, b);
                    break;
                }
            }
        }
    }
    if (!found_gnss) {
        uart_driver_delete(UART_NUM_1);
        uart.init(UART_NUM_1, nav::PINS.gnss_tx, locked_rx, 9600);
        ESP_LOGW("GNSS", "No GPS NMEA detected on candidate RX pins");
    }

    xbee_uart.init(UART_NUM_2, nav::PINS.xbee_tx, nav::PINS.xbee_rx, nav::TELEM_CFG.xbee_baud);
    baro_drv.init(i2c0, 0x46);
    if (g_resumed_in_flight) baro_drv.set_sea_level_pressure(g_persist.baro_p0_pa);   // keep AGL continuous
    else                     baro_drv.auto_calibrate_baseline(0.0, 20);
    imu_drv.init(i2c0, 0x28);
    g_heading_offset_deg.store(nvs_cfg.get_heading_offset_deg());

    if (nav::PINS.spi_devices_fitted) scan_drv.init(spi, nav::PINS.cc1101_cs);
    float init_lat = nvs_cfg.get_last_lat();
    float init_lon = nvs_cfg.get_last_lon();
    float init_alt = nvs_cfg.get_ground_alt_m();
    gnss_drv.init(uart, init_lat, init_lon, init_alt);
    sdp31_drv.init(i2c0); sht4x_drv.init(i2c0); sgp41_drv.init(i2c0);
    xbee.init(xbee_uart); setup_command_handlers();
    if (nav::PINS.spi_devices_fitted) sd_logger.init(nav::PINS.sd_clk, nav::PINS.sd_cmd, nav::PINS.sd_d0);
    event_log.init(); pwr_mgr.init(i2c0);
    if (!g_resumed_in_flight) { bit::BuiltInTest bit_runner; g_bit_flags.store(bit_runner.run(i2c0, spi, uart, xbee_uart, &sd_logger, nvs_cfg).flags); }

    // Flight data recorder + crash dump from a previous session
    recorder.init(nvs_cfg.get_boot_count(), (float)baro_drv.get_sea_level_pressure(), g_resumed_in_flight);
    {
        size_t cd_addr = 0, cd_size = 0;
        if (esp_core_dump_image_get(&cd_addr, &cd_size) == ESP_OK) {
            g_crash_present.store(true);
            ESP_LOGW("MAIN", "Crash dump from a previous session present (%u bytes): send CMD,001,LOG,CRASH", (unsigned)cd_size);
            recorder.log_event(0, "CRASH DUMP PRESENT FROM PREVIOUS SESSION");
        }
    }
    // Mission state: fresh on the pad, or resumed after an in-flight reset
    if (g_resumed_in_flight) {
        mission.restore(g_persist, now_s());
        if (g_persist.site_valid) site.set(g_persist.site_lat_deg, g_persist.site_lon_deg, g_persist.site_alt_off_m);
        site.lock();
        imu_drv.request_attitude_tare();          // re-reference now (cannot wait for stillness mid-air)
        ESP_LOGW("MAIN", "Restored: phase %s, peak %.1f m, arms %s, site %s",
                 nav::mission_phase_name(mission.phase()), (double)g_persist.h_max_m,
                 g_persist.arms_unlatched ? "UNLATCHED" : "latched", g_persist.site_valid ? "valid" : "none");
    } else {
        // Boot alignment on the pad: let the IMU auto-identify the mount (up to ~4 s)
        drivers::IMUData imu_boot{};
        for (int i = 0; i < 400; ++i) {
            imu_boot = imu_drv.read();
            if (imu_drv.attitude_ref().referenced() && imu_boot.valid) break;
            boot_led_breathe();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        ESP_LOGI("MAIN", "Boot attitude: mount=%s", drivers::mount_class_name(imu_drv.current_mount()));
    }

    // Actuators: servos start in the persisted state (never re-latch arms mid-air)
    latch_servos.init(nav::PINS.servo_a, nav::PINS.servo_b,
                      g_persist.arms_unlatched ? nav::ACT_CFG.servo_unlatch_us : nav::ACT_CFG.servo_lock_us);
    if (g_persist.arms_unlatched) g_manual_unlatch.store(true);
    esc.init(nav::PINS.motor);

    // Bluetooth LE health / command link (off automatically during a real flight)
    g_ble_cmd_q = xQueueCreate(4, 160);
    {
        char ble_name[24];
        snprintf(ble_name, sizeof(ble_name), "AAKASHVANI-%03u", (unsigned)nav::TELEM_CFG.team_id);
        ble.init(ble_name, [](const char* l) {
            char buf[160]; snprintf(buf, sizeof(buf), "%s", l);
            xQueueSend(g_ble_cmd_q, buf, 0);
        });
    }

    if (!g_resumed_in_flight) boot_led_result(g_bit_flags.load());

    xEventGroupSetBits(evt_group, EVT_BIT_PASS);
    wdg.init(true);
    xTaskCreatePinnedToCore(imu_task, "imu", STK_IMU, nullptr, PRI_IMU_TASK, nullptr, 0);
    xTaskCreatePinnedToCore(nav_task, "nav", STK_NAV, nullptr, PRI_NAV_TASK, nullptr, 0);
    xTaskCreatePinnedToCore(control_task, "ctrl", STK_CTRL, nullptr, PRI_CTRL_TASK, nullptr, 0);
    xTaskCreatePinnedToCore(sensor_task, "sensor", STK_SENSOR, nullptr, PRI_SENSOR_TASK, nullptr, 1);
    xTaskCreatePinnedToCore(telem_task, "telem", STK_TELEM, nullptr, PRI_TELEM_TASK, nullptr, 1);
    xTaskCreatePinnedToCore(logging_task, "logging", STK_LOGGING, nullptr, PRI_LOGGING_TASK, nullptr, 1);
    xTaskCreatePinnedToCore(power_task, "power", STK_POWER, nullptr, PRI_POWER_TASK, nullptr, 1);
    xTaskCreatePinnedToCore(cli_task, "cli", 6144, nullptr, 1, nullptr, 1);
    xTaskCreatePinnedToCore(ble_cmd_task, "ble_cmd", 6144, nullptr, 1, nullptr, 1);   // 4 KB left 672 B headroom
    xTaskCreatePinnedToCore(test_task, "test_mgr", 4096, nullptr, 1, nullptr, 1);
    xTaskCreate(beacon_task, "beacon", 4096, nullptr, 2, nullptr);
}

extern "C" void app_main(void) {
    system_init::core_init();
#if defined(CONFIG_ROLE_FC)
    main_fc();
#elif defined(CONFIG_ROLE_GCS)
    extern void main_gcs(); main_gcs();
#endif
}
