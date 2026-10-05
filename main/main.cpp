/**
 * @file main.cpp
 * @brief CAN-7USAT 2026 CanSat — ESP32-S3 Flight Software Entry Point.
 */

#include "nav/config.hpp"
#include "nav/flight_computer.hpp"
#include "nav/nav_state.hpp"
#include "nav/frames.hpp"
#include "nav/supervisor.hpp"

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

#include "control/cascaded_pid.hpp"
#include "control/motor_mixer_x.hpp"
#include "control/motor_mixer.hpp"

#include "telemetry/encoder.hpp"

#include "comms/xbee_link.hpp"
#include "comms/command_parser.hpp"
#include "comms/ota_service.hpp"

#include "cli/console.hpp"
#include "logging/sd_logger.hpp"
#include "logging/event_log.hpp"
#include "logging/coredump_exporter.hpp"

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

#include <cmath>
#include <cstring>
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
#define STK_NAV      32768
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

static nav::FlightComputer fc;

static control::CascadedPID attitude_pid;
static control::PID         descent_pid;
static control::MotorMixerX mixer_x;
static control::MotorMixer  motors;

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
static std::atomic<bool>     telem_enabled{true};
static std::atomic<bool>     pid_test_active{false};
static std::atomic<float>    pid_test_throttle{0.25f};

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

static void configure_pids() noexcept {
    const nav::ControlConfig& C = nav::CONTROL_CFG;

    control::PIDGains angle_gains = { (float)C.kp_attitude, (float)C.ki_attitude, 0.0f, (float)C.anti_windup_limit_rad };
    control::PIDGains rate_gains = { (float)C.kp_attitude * 0.5f, (float)C.ki_attitude * 0.5f, (float)C.kd_attitude, 0.2f };
    control::PIDGains yaw_rate_gains = { 1.0f, 0.05f, 0.0f, 0.1f };

    attitude_pid.set_angle_gains(angle_gains, angle_gains);
    attitude_pid.set_rate_gains(rate_gains, rate_gains, yaw_rate_gains);

    control::PIDGains dsc_gains = { (float)C.kp_descent, (float)C.ki_descent, (float)C.kd_descent, 0.3f };
    descent_pid.set_gains(dsc_gains);
}

/// Shortest-arc quaternion (body->world) that rotates the measured gravity
/// reaction (vehicle frame) onto world +Z. Yaw is left at zero.
static nav::NavState level_nav_from_accel(const drivers::IMUData& imu) noexcept {
    nav::NavState ns{};
    ns.q(0) = 1.0; ns.q(1) = 0.0; ns.q(2) = 0.0; ns.q(3) = 0.0;
    if (!imu.valid) return ns;
    const double n = std::sqrt(imu.acc_x*imu.acc_x + imu.acc_y*imu.acc_y + imu.acc_z*imu.acc_z);
    if (n < 5.0 || n > 15.0) return ns;          // not at rest -> keep level default
    const double ax = imu.acc_x / n, ay = imu.acc_y / n, az = imu.acc_z / n;
    if (az < -0.999) { ns.q(0) = 0.0; ns.q(1) = 1.0; return ns; }   // upside down
    // q = [1 + a.z, a x z] normalised, with a x z = (ay, -ax, 0)
    double w = 1.0 + az, x = ay, y = -ax;
    const double qn = std::sqrt(w*w + x*x + y*y);
    ns.q(0) = w / qn; ns.q(1) = x / qn; ns.q(2) = y / qn; ns.q(3) = 0.0;
    return ns;
}

/// 100 Hz IMU acquisition + attitude reference. Kept separate from nav_task so the
/// live attitude stream never waits on the (software-double) IMM filter update.
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
            ESP_LOGI("IMU", "%.1f Hz (%lu fail), read avg %.2f ms max %.2f ms",
                     n_ok / win_s, (unsigned long)n_fail,
                     (double)rd_sum / (double)(n_ok + n_fail) * 1e-3, (double)rd_max * 1e-3);
            n_ok = n_fail = 0; rd_sum = rd_max = 0; win_t0 = rd_t0;
        }
        vTaskDelayUntil(&last_wake, period);
    }
}

static void nav_task(void* /*arg*/) {
    watchdog::Watchdog::register_task();
    const TickType_t period = pdMS_TO_TICKS(10);
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t baro_div = 0;
    uint32_t gnss_div = 0;
    uint32_t seen_ref_count = imu_drv.attitude_ref().reference_count();
    uint32_t seen_imu_seq = 0;

    while (true) {
        watchdog::Watchdog::ping();
        const double t = now_s();

        // Consume the newest IMU sample (filter runs at whatever rate it can sustain;
        // ingest_imu integrates over the true sample-to-sample dt)
        drivers::IMUData imu;
        uint32_t imu_seq;
        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        imu = latest_imu_snap;
        imu_seq = latest_imu_seq;
        xSemaphoreGive(sensor_mutex);

        if (imu.valid && imu_seq != seen_imu_seq) {
            seen_imu_seq = imu_seq;
            xSemaphoreTake(fc_mutex, portMAX_DELAY);
            const bool flying = fc.supervisor.latches.flight_started;
            // If the IMU re-identified its mount (boot reference or CMD,TARE on the pad),
            // the vehicle frame changed -> re-level the EKF attitude.
            const uint32_t rc = imu_drv.attitude_ref().reference_count();
            if (rc != seen_ref_count) {
                seen_ref_count = rc;
                if (!flying) {
                    nav::NavState ns = level_nav_from_accel(imu);
                    ns.p = fc.imm.fuse().nav.p;
                    fc.imm.set_initial_nav(ns);
                    ESP_LOGW("NAV", "IMU mount re-identified (%s) -> EKF attitude re-levelled",
                             drivers::mount_class_name(imu_drv.current_mount()));
                }
            }
            fc.ingest_imu(imu.timestamp_s, imu.acc_x, imu.acc_y, imu.acc_z, imu.gyr_x, imu.gyr_y, imu.gyr_z);
            xSemaphoreGive(fc_mutex);
        }

        // Ingest baro (every 20ms = 50Hz)
        if (++baro_div >= 2) {
            baro_div = 0;
            drivers::BaroData baro;
            xSemaphoreTake(sensor_mutex, portMAX_DELAY);
            baro = latest_baro;
            xSemaphoreGive(sensor_mutex);
            if (baro.valid) {
                xSemaphoreTake(fc_mutex, portMAX_DELAY);
                fc.ingest_baro(t, (double)baro.altitude_agl_m);
                xSemaphoreGive(fc_mutex);
            }
        }

        // Ingest GNSS (every 100ms = 10Hz)
        if (++gnss_div >= 10) {
            gnss_div = 0;
            drivers::GNSSData gnss;
            xSemaphoreTake(sensor_mutex, portMAX_DELAY);
            gnss = latest_gnss;
            xSemaphoreGive(sensor_mutex);
            if (gnss.valid) {
                xSemaphoreTake(fc_mutex, portMAX_DELAY);
                fc.ingest_gnss(t, gnss.pos_e, gnss.pos_n, gnss.pos_u, gnss.vel_e, gnss.vel_n, gnss.vel_u, gnss.gagan_active);
                xSemaphoreGive(fc_mutex);
            }
        }

        // Fixed 100 Hz: the period includes the loop's own work time (no drift)
        vTaskDelayUntil(&last_wake, period);
    }
}

static void sensor_task(void* /*arg*/) {
    const TickType_t period = pdMS_TO_TICKS(20);

    while (true) {
        drivers::BaroData baro = baro_drv.read();
        if (baro.valid) {
            gnss_drv.set_baro_alt_aiding(baro.altitude_agl_m + nvs_cfg.get_ground_alt_m());
        }
        drivers::GNSSData gnss = gnss_drv.read();

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
                printf("1234,ENV,%.2f,%.1f,%u,%u\n",
                       sht.valid ? sht.temperature_c : 0.0,
                       sht.valid ? sht.humidity_pct : 0.0,
                       sgp.valid ? sgp.voc_index : 100,
                       sgp.valid ? sgp.nox_index : 1);
                fflush(stdout);
            }
        }

        vTaskDelay(period);
    }
}

static void control_task(void* /*arg*/) {
    watchdog::Watchdog::register_task();
    const TickType_t period = pdMS_TO_TICKS(10);
    while (!nvs_cfg.get_bit_override() &&
           (xEventGroupGetBits(evt_group) & EVT_BIT_PASS) == 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    while (true) {
        watchdog::Watchdog::ping();
        drivers::BaroData baro;
        nav::FlightComputerOutput fc_out{};
        {
            xSemaphoreTake(sensor_mutex, portMAX_DELAY);
            baro = latest_baro;
            xSemaphoreGive(sensor_mutex);
            xSemaphoreTake(fc_mutex, portMAX_DELAY);
            fc_out = fc.last_output;
            bool valid = fc.output_valid;
            xSemaphoreGive(fc_mutex);
            if (!valid) { vTaskDelay(period); continue; }
        }

        const float altitude = baro.altitude_agl_m;
        const float dt       = (float)nav::CONTROL_CFG.pid_dt_s;

        // Live PID Test mode or autonomous State 6 (DRONE_HOVER):
        bool in_pid_test = pid_test_active.load();
        if (in_pid_test || fc_out.sup.state_code == 6) {
            if (!motors.is_armed()) motors.arm();
            nav::EulerAngles ea = nav::euler_from_quat(fc_out.imm.nav.q);
            float roll = (float)ea.roll_rad;
            float pitch = (float)ea.pitch_rad;
            if (!std::isfinite(roll)) roll = 0.0f;
            if (!std::isfinite(pitch)) pitch = 0.0f;

            drivers::IMUData imu;
            {
                xSemaphoreTake(sensor_mutex, portMAX_DELAY);
                imu = latest_imu_snap;
                xSemaphoreGive(sensor_mutex);
            }
            float gyr_x = (float)imu.gyr_x;
            float gyr_y = (float)imu.gyr_y;
            float gyr_z = (float)imu.gyr_z;
            if (!std::isfinite(gyr_x)) gyr_x = 0.0f;
            if (!std::isfinite(gyr_y)) gyr_y = 0.0f;
            if (!std::isfinite(gyr_z)) gyr_z = 0.0f;

            float base_thr = in_pid_test ? pid_test_throttle.load() : 0.10f;
            float throttle = base_thr;
            control::CascadedPID::Vector3 target_att = {0, 0, 0};
            control::CascadedPID::Vector3 current_att = {roll, pitch, 0};
            control::CascadedPID::Vector3 current_rates = {gyr_x, gyr_y, gyr_z};

            auto torque = attitude_pid.update(target_att, current_att, current_rates, dt);
            auto pwm = mixer_x.mix(throttle, torque.x, torque.y, torque.z);

            uint32_t m1_us = std::isfinite(pwm.m1) ? (uint32_t)pwm.m1 : 1000;
            uint32_t m2_us = std::isfinite(pwm.m2) ? (uint32_t)pwm.m2 : 1000;
            uint32_t m3_us = std::isfinite(pwm.m3) ? (uint32_t)pwm.m3 : 1000;
            uint32_t m4_us = std::isfinite(pwm.m4) ? (uint32_t)pwm.m4 : 1000;

            motors.set_motor_us(0, m1_us);
            motors.set_motor_us(1, m2_us);
            motors.set_motor_us(2, m3_us);
            motors.set_motor_us(3, m4_us);

            static uint32_t pid_diag_cnt = 0;
            if (in_pid_test && ++pid_diag_cnt >= 20) { // 5Hz log during live test
                pid_diag_cnt = 0;
                ESP_LOGI("PID_TEST", "Tilt [Roll: %+.1f°, Pitch: %+.1f°] -> Motors: M1=%lu, M2=%lu, M3=%lu, M4=%lu us",
                         (double)(roll * 180.0f / 3.14159f), (double)(pitch * 180.0f / 3.14159f),
                         (unsigned long)pwm.m1, (unsigned long)pwm.m2, (unsigned long)pwm.m3, (unsigned long)pwm.m4);
            }
        } else {
            if (fc_out.sup.state_code >= 4) {
                motors.servo_release();
            }
            if (motors.is_armed()) {
                motors.disarm();
            }
            attitude_pid.reset();
            descent_pid.reset();
        }
        vTaskDelay(period);
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
            printf("%u,ATT,%lu,%.5f,%.5f,%.5f,%.5f,%.2f,%.2f,%.2f,%u,%lu\n",
                   (unsigned)nav::TELEM_CFG.team_id,
                   (unsigned long)(att_snap.timestamp_s * 1000.0),
                   att_snap.quat_w, att_snap.quat_x, att_snap.quat_y, att_snap.quat_z,
                   att_snap.euler_pitch_deg, att_snap.euler_roll_deg, att_snap.euler_yaw_deg,
                   att_snap.quat_valid ? (unsigned)att_snap.mount : 0u,
                   (unsigned long)imu_drv.attitude_ref().reference_count());
        }
        if (telem_enabled.load() && ++frame_div >= FRAME_DIV) {
            frame_div = 0;
            drivers::BaroData baro_snap; drivers::GNSSData gnss_snap; drivers::IMUData imu_snap;
            drivers::PowerData pwr_snap; drivers::HumidData sht_snap; nav::FlightComputerOutput fc_snap;
            {
                xSemaphoreTake(sensor_mutex, portMAX_DELAY);
                baro_snap = latest_baro; gnss_snap = latest_gnss; imu_snap = latest_imu_snap; pwr_snap = latest_pwr;
                sht_snap = latest_sht4x;
                xSemaphoreGive(sensor_mutex);
            }
            if (sht_snap.valid) {
                baro_snap.temperature_c = (float)sht_snap.temperature_c;
            }
            {
                xSemaphoreTake(fc_mutex, portMAX_DELAY);
                fc_snap = fc.last_output;
                xSemaphoreGive(fc_mutex);
            }
            const uint32_t pkt_cnt = ++packet_count, mt_s = mission_time_now_s();
            telemetry::TelemetryFrame frame = telemetry::TelemetryEncoder::make_frame(fc_snap, baro_snap, gnss_snap, imu_snap, pwr_snap, scan_drv.get_frequency(), scan_drv.read_rssi_dbm(), pkt_cnt, mt_s);
            int n = telem_enc.encode(frame, csv_buf, sizeof(csv_buf));
            if (n > 0) {
                char sd_buf[telemetry::TelemetryEncoder::BUF_LEN + 16];
                int sn = snprintf(sd_buf, sizeof(sd_buf), "%u,%s\n", (unsigned)nav::TELEM_CFG.team_id, csv_buf);
                if (sn > 0) {
                    // Decimate RF radio and SD logging to standard 1 Hz
                    if (++radio_div >= RADIO_DIV) {
                        radio_div = 0;
                        xbee.enqueue_packet(csv_buf, (size_t)n);
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

enum class FlightMissionState : uint8_t {
    PAD = 0,
    ASCENT = 1,
    APOGEE = 2,
    DRONE_DESCENT = 3,
    TOUCHDOWN = 4
};

static std::atomic<FlightMissionState> g_flight_mission_state{FlightMissionState::PAD};
static std::atomic<float> g_mission_rel_alt{0.0f};
static std::atomic<float> g_mission_max_alt{0.0f};

static const char* get_flight_state_name(FlightMissionState s) {
    switch (s) {
        case FlightMissionState::PAD:           return "PAD (Standby)";
        case FlightMissionState::ASCENT:        return "ASCENT";
        case FlightMissionState::APOGEE:        return "APOGEE (Chute)";
        case FlightMissionState::DRONE_DESCENT: return "DRONE_DESCENT";
        case FlightMissionState::TOUCHDOWN:     return "TOUCHDOWN";
        default:                                return "UNKNOWN";
    }
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
    if (throttle_pct < 0.0f) throttle_pct = 0.0f;
    if (throttle_pct > 25.0f) throttle_pct = 25.0f;
    uint16_t pulse_us = 1000 + (uint16_t)(throttle_pct * 10.0f);
    motors.arm();
    motors.set_motor_us(0, pulse_us); // ESC1 on GPIO 14
    vTaskDelay(pdMS_TO_TICKS(2500));
    motors.set_motor_us(0, 1000);
    motors.disarm();
    printf("[TEST] HakRC ESC1 test complete. Disarmed.\n");
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
    motors.disarm();
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
                FlightMissionState st = g_flight_mission_state.load();
                printf("[ALT-LED] %-14s | Alt: %+5.2fm | Desk dH: %+5.2fm | Peak: %+5.2fm\n",
                       get_flight_state_name(st), baro.altitude_agl_m,
                       (double)g_mission_rel_alt.load(), (double)g_mission_max_alt.load());
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

static void beacon_task(void* /*arg*/) {
    const int BEACON_PIN = nav::PINS.beacon;
    gpio_set_direction((gpio_num_t)BEACON_PIN, GPIO_MODE_OUTPUT);

    FlightMissionState mission_state = FlightMissionState::PAD;
    double baseline_alt = 0.0;
    bool baseline_locked = false;
    double max_alt = 0.0;
    int64_t apogee_start_us = 0;
    int64_t touchdown_start_us = 0;
    int64_t ascent_abort_us = 0;
    uint32_t touchdown_counter = 0;
    int cal_samples = 0;
    double cal_acc = 0.0;
    int launch_confirm_count = 0;
    int apogee_confirm_count = 0;

    // Start with solid calm green on boot
    set_onboard_rgb(0, 40, 10, true);

    while (true) {
        drivers::BaroData baro;
        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        baro = latest_baro;
        xSemaphoreGive(sensor_mutex);

        // Check if Flight Computer supervisor has confirmed LANDED state
        nav::Phase fc_phase = nav::Phase::PRE_FLIGHT;
        xSemaphoreTake(fc_mutex, portMAX_DELAY);
        if (fc.output_valid) fc_phase = fc.last_output.sup.phase;
        xSemaphoreGive(fc_mutex);

        if (fc_phase == nav::Phase::LANDED && mission_state != FlightMissionState::PAD) {
            mission_state = FlightMissionState::TOUCHDOWN;
        }

        int64_t now_us = esp_timer_get_time();

        if (baro.valid) {
            if (!baseline_locked) {
                cal_acc += baro.altitude_agl_m;
                cal_samples++;
                if (cal_samples >= 40) { // ~1.6s on desk to settle IIR filter
                    baseline_alt = cal_acc / cal_samples;
                    baseline_locked = true;
                    ESP_LOGI("MISSION", "Pad baseline altitude permanently locked: %.2fm", baseline_alt);
                }
            } else if (mission_state == FlightMissionState::PAD) {
                // If stationary on pad, gently track slow environmental baro drift
                double raw_diff = baro.altitude_agl_m - baseline_alt;
                if (std::abs(raw_diff) < 0.20) {
                    baseline_alt = 0.98 * baseline_alt + 0.02 * baro.altitude_agl_m;
                }
            }
        }

        double rel_alt = 0.0;
        if (baseline_locked && baro.valid) {
            rel_alt = baro.altitude_agl_m - baseline_alt;
        }

        g_mission_rel_alt.store((float)rel_alt);
        g_mission_max_alt.store((float)max_alt);
        g_flight_mission_state.store(mission_state);

        switch (mission_state) {
            case FlightMissionState::PAD: {
                gpio_set_level((gpio_num_t)BEACON_PIN, 0);
                // Calm solid Emerald Green: indicates Armed & Ready
                set_onboard_rgb(0, 40, 10);

                // Launch Detection:
                // Desk noise floor is +/- 0.10m.
                // Require lifting >= 0.35m (35 cm / ~14 in) sustained for 3 consecutive cycles (120ms)
                if (baseline_locked && rel_alt >= 0.35) {
                    launch_confirm_count++;
                    if (launch_confirm_count >= 3) {
                        mission_state = FlightMissionState::ASCENT;
                        max_alt = rel_alt;
                        launch_confirm_count = 0;
                        ascent_abort_us = 0;
                        set_onboard_rgb(140, 60, 0); // Solid bright ORANGE
                        ESP_LOGW("MISSION", ">>> [ALT-LED] LAUNCH DETECTED! Phase: ASCENT | dH: %+5.2fm (LED: ORANGE) <<<", rel_alt);
                    }
                } else {
                    launch_confirm_count = 0;
                }
                break;
            }

            case FlightMissionState::ASCENT: {
                // Rocket Ascent: Solid bright ORANGE
                set_onboard_rgb(140, 60, 0);
                if (rel_alt > max_alt) max_alt = rel_alt;

                // Apogee Condition:
                // Must have climbed to at least 0.45m (45 cm)
                // AND must have dropped by >= 0.18m (18 cm) from the peak
                // AND sustained for 3 cycles (120ms) to ensure real descent, not hand vibration
                if (max_alt >= 0.45 && rel_alt <= (max_alt - 0.18)) {
                    apogee_confirm_count++;
                    if (apogee_confirm_count >= 3) {
                        mission_state = FlightMissionState::APOGEE;
                        apogee_start_us = now_us;
                        apogee_confirm_count = 0;
                        motors.servo_release(); // Deploy parachute latch
                        set_onboard_rgb(160, 0, 160); // Bright PURPLE
                        ESP_LOGW("MISSION", ">>> [ALT-LED] *** APOGEE DETECTED (Apex: %+5.2fm) *** Chute Deployed! (LED: PURPLE) <<<", max_alt);
                    }
                } else {
                    apogee_confirm_count = 0;
                }

                // Aborted lift / false bump: if board was set back down without reaching 0.45m
                if (rel_alt <= 0.15) {
                    if (ascent_abort_us == 0) ascent_abort_us = now_us;
                    else if ((now_us - ascent_abort_us) >= 1500000) { // 1.5s back on desk
                        mission_state = FlightMissionState::PAD;
                        ascent_abort_us = 0;
                        set_onboard_rgb(0, 40, 10);
                        ESP_LOGI("MISSION", ">>> Aborted climb. Returned to PAD Standby. <<<");
                    }
                } else {
                    ascent_abort_us = 0;
                }
                break;
            }

            case FlightMissionState::APOGEE: {
                // Bright PURPLE flash to signify parachute latch deployment
                set_onboard_rgb(160, 0, 160);
                // Stay in APOGEE purple flash for 1.2 seconds, then transition to controlled descent
                if ((now_us - apogee_start_us) >= 1200000) {
                    mission_state = FlightMissionState::DRONE_DESCENT;
                    set_onboard_rgb(0, 100, 100); // Bright CYAN
                    ESP_LOGW("MISSION", ">>> [ALT-LED] Phase: CONTROLLED DESCENT / DRONE HOVER (LED: CYAN) <<<");
                }
                break;
            }

            case FlightMissionState::DRONE_DESCENT: {
                // Drone Hover & Controlled Descent: Solid bright CYAN
                set_onboard_rgb(0, 100, 100);

                // Touchdown trigger: returned back near desk level (rel_alt <= 0.15m) for 1.2 seconds continuously
                if (rel_alt <= 0.15) {
                    if (touchdown_start_us == 0) {
                        touchdown_start_us = now_us;
                    } else if ((now_us - touchdown_start_us) >= 1200000) { // 1.2s confirmed stationary on desk
                        mission_state = FlightMissionState::TOUCHDOWN;
                        touchdown_counter = 0;
                        motors.disarm();
                        ESP_LOGW("MISSION", ">>> [ALT-LED] *** TOUCHDOWN CONFIRMED (LANDED)! *** (LED: FLASHING RED) <<<");
                    }
                } else {
                    touchdown_start_us = 0;
                }
                break;
            }

            case FlightMissionState::TOUCHDOWN: {
                // Flashing RED recovery beacon + buzzer (2 Hz)
                touchdown_counter++;
                if (touchdown_counter % 12 < 6) {
                    set_onboard_rgb(180, 0, 0); // Bright RED
                    gpio_set_level((gpio_num_t)BEACON_PIN, 1);
                } else {
                    set_onboard_rgb(0, 0, 0);
                    gpio_set_level((gpio_num_t)BEACON_PIN, 0);
                }

                // After 5.0 seconds on desk (125 * 40ms = 5000ms), smoothly transition back to PAD standby
                // WITHOUT wiping or corrupting the baseline reference!
                if (touchdown_counter >= 125) {
                    mission_state = FlightMissionState::PAD;
                    touchdown_start_us = 0;
                    touchdown_counter = 0;
                    max_alt = 0.0;
                    launch_confirm_count = 0;
                    apogee_confirm_count = 0;
                    gpio_set_level((gpio_num_t)BEACON_PIN, 0);
                    set_onboard_rgb(0, 40, 10);
                    ESP_LOGI("MISSION", ">>> Touchdown sequence complete. Returned to PAD Standby. Ready for next flight! <<<");
                }
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

static void setup_command_handlers() {
    cmd_parser.on_cx([](bool enable) {
        telem_enabled.store(enable);
        ESP_LOGI("Cmd", "Telemetry streaming %s", enable ? "ENABLED" : "DISABLED");
    });
    cmd_parser.on_st([](uint32_t t) {
        mission_epoch_us.store(esp_timer_get_time() - (int64_t)t * 1000000);
        ESP_LOGI("Cmd", "Mission time set to %u s", (unsigned)t);
    });
    cmd_parser.on_tare([]() {
        imu_drv.request_attitude_tare();   // applied on the next IMU sample in nav_task
    });
    cmd_parser.on_cal([]() {
        drivers::BaroData snap;
        xSemaphoreTake(sensor_mutex, portMAX_DELAY); snap = latest_baro; xSemaphoreGive(sensor_mutex);
        if (snap.valid) {
            nvs_cfg.set_ground_alt_m(snap.altitude_agl_m);
            baro_drv.auto_calibrate_baseline(0.0, 10);
        }
        xSemaphoreTake(fc_mutex, portMAX_DELAY);
        fc.supervisor.reset();
        xSemaphoreGive(fc_mutex);
        ESP_LOGI("Cmd", "Ground zero calibrated & Mission Supervisor reset to PAD.");
    });
    cmd_parser.on_sim([](const char* mode) {
        if (strcasecmp(mode, "ENABLE") == 0 || strcasecmp(mode, "ACTIVATE") == 0) {
            baro_drv.set_sim_mode(true);
            xSemaphoreTake(fc_mutex, portMAX_DELAY);
            fc.supervisor.set_sim_mode(true);
            fc.supervisor.reset();
            xSemaphoreGive(fc_mutex);
            ESP_LOGI("Cmd", "Simulation Mode ACTIVATED (SIMP active, Supervisor reset)");
        } else if (strcasecmp(mode, "DISABLE") == 0) {
            baro_drv.set_sim_mode(false);
            xSemaphoreTake(fc_mutex, portMAX_DELAY);
            fc.supervisor.set_sim_mode(false);
            fc.supervisor.reset();
            xSemaphoreGive(fc_mutex);
            ESP_LOGI("Cmd", "Simulation Mode DISABLED (Live sensor active)");
        }
    });
    cmd_parser.on_simp([](float pa) {
        baro_drv.inject_sim_pressure((double)pa);
        drivers::BaroData d = baro_drv.read();
        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        latest_baro = d;
        xSemaphoreGive(sensor_mutex);
        ESP_LOGI("Cmd", "SIMP Injected: %.1f Pa -> Alt: %.2f m", (double)pa, d.altitude_agl_m);
    });
    cmd_parser.on_abort([]() {
        pid_test_active.store(false);
        xSemaphoreTake(fc_mutex, portMAX_DELAY);
        fc.supervisor.emergency_abort();
        xSemaphoreGive(fc_mutex);
        motors.disarm();
        for(int i=0; i<4; ++i) motors.set_motor_us(i, 1000);
        motors.servo_release();
        ESP_LOGW("Cmd", "EMERGENCY ABORT EXECUTED — Motors cut, chute released");
    });
    cmd_parser.on_chute([]() {
        motors.servo_release();
        ESP_LOGW("Cmd", "MANUAL CHUTE DEPLOYMENT TRIGGERED");
    });
    cmd_parser.on_motor([](int motor_id, float pct) {
        pid_test_active.store(false);
        if (pct < 0.0f) pct = 0.0f;
        if (pct > 100.0f) pct = 100.0f;
        if (pct > 0.0f) {
            motors.arm();
        }
        uint32_t us = 1000 + (uint32_t)(pct * 10.0f);
        if (motor_id == -1) {
            for (int i = 0; i < 4; ++i) motors.set_motor_us(i, us);
            ESP_LOGI("Cmd", "ALL Motors commanded to %.1f%% (%lu us)", (double)pct, (unsigned long)us);
        } else if (motor_id >= 0 && motor_id < 4) {
            motors.set_motor_us(motor_id, us);
            ESP_LOGI("Cmd", "Motor %d commanded to %.1f%% (%lu us)", motor_id, (double)pct, (unsigned long)us);
        }
        if (pct == 0.0f) {
            motors.disarm();
        }
    });
    cmd_parser.on_pid([](bool start, float base_throttle) {
        pid_test_throttle.store(base_throttle);
        if (start) {
            mixer_x.reset_slew();
            attitude_pid.reset();
            descent_pid.reset();
            motors.arm();
            pid_test_active.store(true);
            ESP_LOGI("Cmd", ">>> LIVE PID ATTITUDE STABILISATION STARTED (Base Throttle: %.1f%%) <<<", (double)(base_throttle * 100.0f));
        } else {
            pid_test_active.store(false);
            motors.disarm();
            mixer_x.reset_slew();
            ESP_LOGI("Cmd", ">>> LIVE PID ATTITUDE STABILISATION STOPPED <<<");
        }
    });
    cmd_parser.on_rtl([]() {
        ESP_LOGI("Cmd", "RETURN TO LAUNCH TRIGGERED");
    });
    xbee.set_rx_callback(cmd_parser.make_rx_callback());
}

#include "system_init.hpp"

extern "C" void main_fc() {
    fc_mutex = xSemaphoreCreateMutex(); sensor_mutex = xSemaphoreCreateMutex(); evt_group = xEventGroupCreate();

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

    ESP_LOGI("BB_SCAN", "Starting slow Bit-Bang I2C scan (SDA=GPIO%d, SCL=GPIO%d)...", sda, scl);
    int bb_found = 0;
    uint8_t found_addrs[16] = {};
    for (uint8_t a = 1; a < 128; ++a) {
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
    spi.init(nav::PINS.spi_mosi, nav::PINS.spi_miso, nav::PINS.spi_sck);

    // Auto-detect GNSS RX pin and baud rate across candidate pins & speeds (excluding I2C/IMU/PSRAM)
    static const int candidate_rx_pins[] = {13, 16, 17, 4, 21};
    static const int gnss_bauds[] = {9600, 115200};
    int locked_rx = nav::PINS.gnss_rx;
    int locked_baud = 9600;
    bool found_gnss = false;

    for (int rx : candidate_rx_pins) {
        if (found_gnss) break;
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
    baro_drv.auto_calibrate_baseline(0.0, 20);
    imu_drv.init(i2c0, 0x28);

    scan_drv.init(spi, nav::PINS.cc1101_cs);
    float init_lat = nvs_cfg.get_last_lat();
    float init_lon = nvs_cfg.get_last_lon();
    float init_alt = nvs_cfg.get_ground_alt_m();
    gnss_drv.init(uart, init_lat, init_lon, init_alt);
    sdp31_drv.init(i2c0); sht4x_drv.init(i2c0); sgp41_drv.init(i2c0);
    xbee.init(xbee_uart); setup_command_handlers();
    sd_logger.init(nav::PINS.sd_clk, nav::PINS.sd_cmd, nav::PINS.sd_d0);
    event_log.init(); pwr_mgr.init(i2c0);
    bit::BuiltInTest bit_runner; bit_runner.run(i2c0, spi, uart, xbee_uart, &sd_logger, nvs_cfg);
    // Boot alignment: let the IMU driver auto-identify the mount (parallel /
    // perpendicular, any axis up) from gravity, then level the EKF from the
    // gravity vector expressed in the VEHICLE frame (vehicle +Z = up).
    drivers::IMUData imu_boot{};
    for (int i = 0; i < 400; ++i) {            // up to ~4 s (fusion convergence + fallback)
        imu_boot = imu_drv.read();
        if (imu_drv.attitude_ref().referenced() && imu_boot.valid) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    nav::NavState init_nav = level_nav_from_accel(imu_boot);
    ESP_LOGI("MAIN", "Boot attitude: mount=%s, levelled from gravity (q=[%.3f %.3f %.3f %.3f])",
             drivers::mount_class_name(imu_drv.current_mount()),
             init_nav.q(0), init_nav.q(1), init_nav.q(2), init_nav.q(3));
    fc.init(init_nav);
    xEventGroupSetBits(evt_group, EVT_BIT_PASS);
    configure_pids(); motors.init(); motors.servo_home(); wdg.init(true);
    xTaskCreatePinnedToCore(imu_task, "imu", STK_IMU, nullptr, PRI_IMU_TASK, nullptr, 0);
    xTaskCreatePinnedToCore(nav_task, "nav", STK_NAV, nullptr, PRI_NAV_TASK, nullptr, 0);
    xTaskCreatePinnedToCore(control_task, "ctrl", STK_CTRL, nullptr, PRI_CTRL_TASK, nullptr, 0);
    xTaskCreatePinnedToCore(sensor_task, "sensor", STK_SENSOR, nullptr, PRI_SENSOR_TASK, nullptr, 1);
    xTaskCreatePinnedToCore(telem_task, "telem", STK_TELEM, nullptr, PRI_TELEM_TASK, nullptr, 1);
    xTaskCreatePinnedToCore(logging_task, "logging", STK_LOGGING, nullptr, PRI_LOGGING_TASK, nullptr, 1);
    xTaskCreatePinnedToCore(power_task, "power", STK_POWER, nullptr, PRI_POWER_TASK, nullptr, 1);
    xTaskCreatePinnedToCore(cli_task, "cli", 4096, nullptr, 1, nullptr, 1);
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
