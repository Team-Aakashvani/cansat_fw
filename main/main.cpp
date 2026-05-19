/**
 * @file main.cpp
 * @brief CAN-7USAT 2026 CanSat — ESP32-S3 Flight Software Entry Point.
 *
 * System architecture (dual-core FreeRTOS):
 *
 *  Core 0 (RT) — fixed at highest priority:
 *    nav_task      : 100Hz IMU ingestion + EKF propagation (10ms period)
 *    control_task  : 100Hz attitude + descent-rate PID (10ms period)
 *
 *  Core 1 (SYS) — normal priority services:
 *    sensor_task   : 50Hz baro + 1Hz GNSS polling (20ms period)
 *    telem_task    : 1Hz telemetry encode + LoRa TX
 *    logging_task  : SD flush + event log
 *    power_task    : 1Hz power monitoring + low-battery callback
 *
 * Boot sequence:
 *   1. NVS init + config load
 *   2. HAL init (I2C0, I2C1, SPI, UART)
 *   3. Driver init (BNO085, BMP585, N-GS-01, SX1278, INA260, MAX17048)
 *   4. LoRa link init + command parser setup
 *   5. SD card mount + event log init
 *   6. BIT (Built-In Test) — halt on critical fail
 *   7. FlightComputer init (EKF, IMM, supervisor)
 *   8. Motor mixer init + servo home
 *   9. Watchdog arm
 *  10. Spawn FreeRTOS tasks
 *
 * @compliance CAN-7USAT India 2026 §3, §5, §6
 */

#include "nav/config.hpp"
#include "nav/flight_computer.hpp"
#include "nav/nav_state.hpp"
#include "nav/frames.hpp"
#include "nav/supervisor.hpp"

#include "hal/i2c_bus.hpp"
#include "hal/spi_bus.hpp"
#include "hal/uart_bus.hpp"

#include "drivers/bno085.hpp"
#include "drivers/bmp585.hpp"
#include "drivers/ngps01.hpp"
#include "drivers/sx1278.hpp"
#include "drivers/ina260.hpp"
#include "drivers/max17048.hpp"
#include "drivers/sdp31.hpp"
#include "drivers/sht4x.hpp"
#include "drivers/sgp41.hpp"

#include "control/pid.hpp"
#include "control/motor_mixer.hpp"

#include "telemetry/encoder.hpp"

#include "comms/lora_link.hpp"
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
#include "p4_link/p4_link.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "nvs_flash.h"

#include <cmath>
#include <cstring>
#include <cstdio>
#include <atomic>

extern void main_gcs();

static const char* TAG = "main";

// ===========================================================================
// FreeRTOS task priorities (higher = higher priority)
// ===========================================================================
#define PRI_NAV_TASK     (configMAX_PRIORITIES - 1)  ///< Highest — 100Hz EKF
#define PRI_CTRL_TASK    (configMAX_PRIORITIES - 2)  ///< 100Hz PID
#define PRI_SENSOR_TASK  (configMAX_PRIORITIES - 3)  ///< 50Hz baro + 1Hz GNSS
#define PRI_TELEM_TASK   5                            ///< 1Hz encode + TX
#define PRI_LOGGING_TASK 4                            ///< SD flush 1Hz
#define PRI_POWER_TASK   3                            ///< 1Hz power monitor

// Stack sizes (words = 4 bytes each)
#define STK_NAV      8192
#define STK_CTRL     4096
#define STK_SENSOR   4096
#define STK_TELEM    4096
#define STK_LOGGING  8192   ///< Larger for SD/FAT calls
#define STK_POWER    2048

// ===========================================================================
// Event bits
// ===========================================================================
#define EVT_BIT_PASS        (1u << 0)
#define EVT_TELEM_ENABLE    (1u << 1)
#define EVT_CALIBRATE       (1u << 2)

// ===========================================================================
// Global subsystem instances — statically allocated, zero init
// ===========================================================================

// HAL
static hal::I2CBus    i2c0;   ///< BNO085 + BMP585
static hal::I2CBus    i2c1;   ///< Environmental sensors + power monitors
static hal::SPIBus    spi;    ///< SX1278 LoRa
static hal::UARTBus   uart;   ///< N-GS-01 GNSS

// Sensor drivers
static drivers::BNO085   imu_drv;
static drivers::BMP585   baro_drv;
static drivers::NGPS01   gnss_drv;
static drivers::SDP31    sdp31_drv;
static drivers::SHT4x    sht4x_drv;
static drivers::SGP41    sgp41_drv;
static drivers::CC1101   scan_drv;

// Navigation
static nav::FlightComputer fc;

// Control
static control::PID         pitch_pid;
static control::PID         roll_pid;
static control::PID         descent_pid;
static control::MotorMixer  motors;

// Comms
static comms::LoRaLink      lora;
static comms::CommandParser cmd_parser;
static comms::OTAService    ota_svc;

// Telemetry
static telemetry::TelemetryEncoder telem_enc;

// Logging
static logging::SDLogger   sd_logger;
static logging::EventLog   event_log;

// Mapping
static rf_mapping::RFMapper rf_mapper;

// P4 Media Coprocessor Link
static hal::UARTBus         p4_uart;
static p4_link::P4Link      p4_link_drv;

// Power
static power::PowerManager pwr_mgr;

// Config + BIT
static config_mgr::NVSConfig nvs_cfg;
static watchdog::Watchdog    wdg;

// CLI
static cli::Console console(nvs_cfg, cmd_parser);

// Synchronisation
static SemaphoreHandle_t fc_mutex     = nullptr;   ///< Protects FlightComputer
static EventGroupHandle_t evt_group   = nullptr;
static std::atomic<uint32_t> packet_count{0};
static std::atomic<uint32_t> mission_time_s{0};
static std::atomic<bool>     telem_enabled{true};
static std::atomic<bool>     sim_mode{false};
static std::atomic<float>    sim_pressure_pa{101325.0f};
static struct {
    std::atomic<double> e, n, u;
    std::atomic<double> ve, vn, vu;
} sim_gnss;
static struct {
    std::atomic<double> ax, ay, az;
    std::atomic<double> gx, gy, gz;
} sim_imu;

// Latest sensor data (updated by sensor_task, read by nav_task / telem_task)
static SemaphoreHandle_t     sensor_mutex = nullptr;
static drivers::BaroData     latest_baro{};
static drivers::GNSSData     latest_gnss{};
static drivers::IMUData      latest_imu_snap{};  ///< For telem fallback only
static drivers::PowerData    latest_pwr{};

// ===========================================================================
// Helper: get elapsed time in seconds (double precision)
// ===========================================================================
static inline double now_s() noexcept {
    return (double)esp_timer_get_time() * 1.0e-6;
}

// ===========================================================================
// PID configuration helper — called once during init
// ===========================================================================
static void configure_pids() noexcept {
    const nav::ControlConfig& C = nav::CONTROL_CFG;

    control::PIDConfig att_cfg{};
    att_cfg.kp              = C.kp_attitude;
    att_cfg.ki              = C.ki_attitude;
    att_cfg.kd              = C.kd_attitude;
    att_cfg.dt_s            = C.pid_dt_s;
    att_cfg.output_min      = -1.0;
    att_cfg.output_max      =  1.0;
    att_cfg.integral_limit  = C.anti_windup_limit_rad;
    att_cfg.deadband        = C.attitude_deadband_rad;
    att_cfg.derivative_alpha= 0.3;

    pitch_pid.configure(att_cfg);
    roll_pid.configure(att_cfg);

    control::PIDConfig dsc_cfg{};
    dsc_cfg.kp              = C.kp_descent;
    dsc_cfg.ki              = C.ki_descent;
    dsc_cfg.kd              = C.kd_descent;
    dsc_cfg.dt_s            = C.pid_dt_s;
    dsc_cfg.output_min      = 0.0;
    dsc_cfg.output_max      = 1.0;
    dsc_cfg.integral_limit  = 0.3;
    dsc_cfg.deadband        = 0.1;
    dsc_cfg.derivative_alpha= 0.3;
    descent_pid.configure(dsc_cfg);
}

// ===========================================================================
// TASK: nav_task — Core 0, 100Hz
// Propagates IMU through IMM EKF. Calls ingest_baro (50Hz) via flag.
// ===========================================================================
static void nav_task(void* /*arg*/) {
    watchdog::Watchdog::register_task();
    ESP_LOGI(TAG, "nav_task started on core %d", xPortGetCoreID());

    const TickType_t period = pdMS_TO_TICKS(10);  // 100Hz
    TickType_t       wake   = xTaskGetTickCount();

    while (true) {
        watchdog::Watchdog::ping();

        const double t = now_s();

        // --- Read latest IMU ---
        drivers::IMUData imu = imu_drv.read();

        // Simulation mode override
        if (sim_mode.load()) {
            imu.acc_x = sim_imu.ax.load();
            imu.acc_y = sim_imu.ay.load();
            imu.acc_z = sim_imu.az.load();
            imu.gyr_x = sim_imu.gx.load();
            imu.gyr_y = sim_imu.gy.load();
            imu.gyr_z = sim_imu.gz.load();
            imu.valid = true;
        }

        if (imu.valid) {
            xSemaphoreTake(sensor_mutex, portMAX_DELAY);
            latest_imu_snap = imu;
            xSemaphoreGive(sensor_mutex);

            xSemaphoreTake(fc_mutex, portMAX_DELAY);
            fc.ingest_imu(t,
                          imu.acc_x, imu.acc_y, imu.acc_z,
                          imu.gyr_x, imu.gyr_y, imu.gyr_z);
            xSemaphoreGive(fc_mutex);
        }

        if (imu.mag_valid) {
            xSemaphoreTake(fc_mutex, portMAX_DELAY);
            fc.ingest_mag(t, imu.mag_x, imu.mag_y, imu.mag_z);
            xSemaphoreGive(fc_mutex);
        }

        // Increment mission time every 1000 ticks (1s at 100Hz)
        static uint32_t tick_counter = 0;
        if (++tick_counter >= 100) {
            tick_counter = 0;
            ++mission_time_s;
        }

        vTaskDelayUntil(&wake, period);
    }
}

// ===========================================================================
// TASK: sensor_task — Core 1, 50Hz baro + 1Hz GNSS
// ===========================================================================
static void sensor_task(void* /*arg*/) {
    ESP_LOGI(TAG, "sensor_task started on core %d", xPortGetCoreID());

    const TickType_t period = pdMS_TO_TICKS(20);  // 50Hz
    TickType_t       wake   = xTaskGetTickCount();
    uint32_t         gnss_divider = 0;

    while (true) {
        const double t = now_s();

        // --- Barometer (50Hz) ---
        drivers::BaroData baro = baro_drv.read();

        // Simulation mode override
        if (sim_mode.load()) {
            baro.pressure_pa    = sim_pressure_pa.load();
            // ISA altitude - ground altitude calibration = AGL
            baro.altitude_agl_m = (float)(
                nav::isa_pressure_to_altitude((double)baro.pressure_pa)
                - (double)nvs_cfg.get_ground_alt_m());
            baro.valid = true;
        }

        {
            xSemaphoreTake(sensor_mutex, portMAX_DELAY);
            latest_baro = baro;
            xSemaphoreGive(sensor_mutex);
        }

        if (baro.valid) {
            xSemaphoreTake(fc_mutex, portMAX_DELAY);
            fc.ingest_baro(t, (double)baro.altitude_agl_m);
            xSemaphoreGive(fc_mutex);
        }

        // --- GNSS (1Hz) ---
        if (++gnss_divider >= 50) {
            gnss_divider = 0;
            drivers::GNSSData gnss = gnss_drv.read();

            if (sim_mode.load()) {
                gnss.pos_e = (float)sim_gnss.e.load();
                gnss.pos_n = (float)sim_gnss.n.load();
                gnss.pos_u = (float)sim_gnss.u.load();
                gnss.vel_e = (float)sim_gnss.ve.load();
                gnss.vel_n = (float)sim_gnss.vn.load();
                gnss.vel_u = (float)sim_gnss.vu.load();
                gnss.valid = true;
            }

            xSemaphoreTake(sensor_mutex, portMAX_DELAY);
            latest_gnss = gnss;
            xSemaphoreGive(sensor_mutex);

            if (gnss.valid) {
                xSemaphoreTake(fc_mutex, portMAX_DELAY);
                // ENU: East, North, Up  |  velocities already in ENU convention
                fc.ingest_gnss(t,
                               gnss.pos_e, gnss.pos_n, gnss.pos_u,
                               gnss.vel_e, gnss.vel_n, gnss.vel_u);
                xSemaphoreGive(fc_mutex);
            }
        }

        vTaskDelayUntil(&wake, period);
    }
}

// ===========================================================================
// TASK: control_task — Core 0, 100Hz
// Attitude + descent-rate PID; mix to motors during DRONE_HOVER phase.
// ===========================================================================
static void control_task(void* /*arg*/) {
    watchdog::Watchdog::register_task();
    ESP_LOGI(TAG, "control_task started on core %d", xPortGetCoreID());

    const TickType_t period = pdMS_TO_TICKS(10);
    TickType_t       wake   = xTaskGetTickCount();

    // Wait for BIT to pass before arming
    xEventGroupWaitBits(evt_group, EVT_BIT_PASS, pdFALSE, pdTRUE, portMAX_DELAY);

    while (true) {
        watchdog::Watchdog::ping();

        nav::FlightComputerOutput fc_out{};
        {
            xSemaphoreTake(fc_mutex, portMAX_DELAY);
            fc_out = fc.last_output;
            bool valid = fc.output_valid;
            xSemaphoreGive(fc_mutex);
            if (!valid) { vTaskDelayUntil(&wake, period); continue; }
        }

        const nav::Phase phase = fc_out.sup.phase;
        const double dt        = (double)nav::CONTROL_CFG.pid_dt_s;

        // P4 Media Coprocessor Triggers
        static nav::Phase last_phase = nav::Phase::PRE_FLIGHT;
        if (phase != last_phase) {
            if (last_phase == nav::Phase::PRE_FLIGHT && phase == nav::Phase::BOOST) {
                p4_link_drv.send_command("RECORD");
            } else if (phase == nav::Phase::LANDED) {
                p4_link_drv.send_command("HALT_FLUSH");
            }
            last_phase = phase;
        }

        if (phase == nav::Phase::DRONE_HOVER) {
            if (!motors.is_armed()) {
                motors.arm();
                pitch_pid.reset();
                roll_pid.reset();
                descent_pid.reset();
                event_log.log_event(logging::EventCode::DRONE_DEPLOY,
                                    mission_time_s.load(), 0, 0, 0, "Motor arm");
            }

            // Extract Euler angles from fused nav state
            nav::EulerAngles ea = nav::euler_from_quat(fc_out.imm.nav.q);
            const double pitch  = ea.pitch_rad;
            const double roll   = ea.roll_rad;
            const double vel_z  = fc_out.imm.nav.v(2);

            // PID updates
            const double pitch_cmd   = pitch_pid.update(0.0, pitch, dt);
            const double roll_cmd    = roll_pid.update(0.0,  roll,  dt);
            const double descent_cmd = descent_pid.update(
                -nav::MISSION.descent_rate_target_mps, vel_z, dt);

            // Battery compensation
            power::PowerState ps = pwr_mgr.get_state();
            double bat_factor = 1.0;
            if (ps.valid && ps.voltage_v > 0.1) {
                bat_factor = std::min(1.0, (double)ps.voltage_v
                                     / (double)nav::POWER_CFG.bat_nominal_v);
            }
            if (pwr_mgr.is_critical()) bat_factor *= nav::CONTROL_CFG.max_throttle_brownout;

            motors.mix_and_set(descent_cmd, pitch_cmd, roll_cmd, 0.0, bat_factor);

        } else if (phase == nav::Phase::PARACHUTE && fc_out.sup.latches.chute_deployed
                   && !fc_out.sup.latches.drone_deployed) {
            // Deploy parachute servo
            motors.servo_release();
            event_log.log_event(logging::EventCode::PARACHUTE_DEPLOY,
                                mission_time_s.load(), 0, 0, 0, "Servo release");

        } else if (phase == nav::Phase::LANDED || phase == nav::Phase::PRE_FLIGHT) {
            if (motors.is_armed()) {
                motors.disarm();
            }
        }

        vTaskDelayUntil(&wake, period);
    }
}

// ===========================================================================
// TASK: telem_task — Core 1, 1Hz
// Encodes telemetry CSV and enqueues for LoRa TX.
// ===========================================================================
static void telem_task(void* /*arg*/) {
    ESP_LOGI(TAG, "telem_task started on core %d", xPortGetCoreID());

    const TickType_t period = pdMS_TO_TICKS(1000);  // 1Hz
    TickType_t       wake   = xTaskGetTickCount();

    char csv_buf[telemetry::TelemetryEncoder::BUF_LEN];

    while (true) {
        if (!telem_enabled.load()) {
            vTaskDelayUntil(&wake, period);
            continue;
        }

        // Snapshot sensors + FC output
        drivers::BaroData  baro_snap{};
        drivers::GNSSData  gnss_snap{};
        drivers::IMUData   imu_snap{};
        drivers::PowerData pwr_snap{};
        nav::FlightComputerOutput fc_snap{};
        p4_link::P4Status p4_snap{};

        {
            xSemaphoreTake(sensor_mutex, portMAX_DELAY);
            baro_snap = latest_baro;
            gnss_snap = latest_gnss;
            imu_snap  = latest_imu_snap;
            pwr_snap  = latest_pwr;
            xSemaphoreGive(sensor_mutex);
        }
        {
            xSemaphoreTake(fc_mutex, portMAX_DELAY);
            fc_snap = fc.last_output;
            xSemaphoreGive(fc_mutex);
        }
        p4_snap = p4_link_drv.get_status();

        // Build and encode frame
        const uint32_t pkt_cnt = ++packet_count;
        const uint32_t mt_s    = mission_time_s.load();

        telemetry::TelemetryFrame frame =
            telemetry::TelemetryEncoder::make_frame(
                fc_snap, baro_snap, gnss_snap, imu_snap, pwr_snap,
                p4_snap, scan_drv.get_frequency(), scan_drv.read_rssi_dbm(),
                pkt_cnt, mt_s);

        int n = telem_enc.encode(frame, csv_buf, sizeof(csv_buf));
        if (n > 0) {
            // 1. Send via LoRa (LoRaLink will prepend TEAM_ID and append CRC+\n)
            lora.enqueue_packet(csv_buf, (size_t)n);

            // 2. Log to SD (Must include TEAM_ID and \n for compliance)
            char sd_buf[telemetry::TelemetryEncoder::BUF_LEN + 16];
            int sn = snprintf(sd_buf, sizeof(sd_buf), "%u,%s\n", 
                              (unsigned)nav::TELEM_CFG.team_id, csv_buf);
            if (sn > 0) sd_logger.write_line(sd_buf);

            // Brief debug log (first field only for rate check)
            ESP_LOGD(TAG, "TELEM[%lu] %d bytes", (unsigned long)pkt_cnt, n);
        }

        vTaskDelayUntil(&wake, period);
    }
}

// ===========================================================================
// TASK: logging_task — Core 1
// Drives LoRa spin() at 1Hz and flushes SD + event log periodically.
// ===========================================================================
static void logging_task(void* /*arg*/) {
    ESP_LOGI(TAG, "logging_task started on core %d", xPortGetCoreID());

    const TickType_t lora_period  = pdMS_TO_TICKS(1000);  // 1Hz LoRa spin
    const TickType_t flush_period = pdMS_TO_TICKS(5000);  // 5s SD flush
    TickType_t       lora_wake    = xTaskGetTickCount();
    TickType_t       flush_wake   = xTaskGetTickCount();

    while (true) {
        TickType_t now = xTaskGetTickCount();

        // LoRa spin (drives TX/RX at 1Hz)
        if ((now - lora_wake) >= lora_period) {
            lora_wake = now;
            lora.spin();
        }

        // SD flush
        if ((now - flush_wake) >= flush_period) {
            flush_wake = now;
            sd_logger.flush();
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Yield CPU; wake up at 10Hz to check
    }
}

// ===========================================================================
// TASK: power_task — Core 1, 1Hz
// ===========================================================================
static void power_task(void* /*arg*/) {
    ESP_LOGI(TAG, "power_task started on core %d", xPortGetCoreID());

    const TickType_t period = pdMS_TO_TICKS(1000);
    TickType_t       wake   = xTaskGetTickCount();

    while (true) {
        pwr_mgr.update();
        {
            power::PowerState ps = pwr_mgr.get_state();
            xSemaphoreTake(sensor_mutex, portMAX_DELAY);
            latest_pwr.voltage_v  = (double)ps.voltage_v;
            latest_pwr.current_a  = (double)ps.current_a;
            latest_pwr.power_w    = (double)ps.power_w;
            latest_pwr.valid      = ps.valid;
            xSemaphoreGive(sensor_mutex);
        }
        vTaskDelayUntil(&wake, period);
    }
}

// ===========================================================================
// TASK: p4_link_task — Core 1
// Drives the reliable UART link to the media coprocessor.
// ===========================================================================
static void p4_link_task(void* /*arg*/) {
    ESP_LOGI(TAG, "p4_link_task started on core %d", xPortGetCoreID());
    const TickType_t period = pdMS_TO_TICKS(100); // 10Hz
    while (true) {
        p4_link_drv.spin();
        vTaskDelay(period);
    }
}

// ===========================================================================
// TASK: cli_task — Core 1
// USB/Serial console for configuration and debugging.
// ===========================================================================
static void cli_task(void* /*arg*/) {
    ESP_LOGI(TAG, "cli_task started on core %d", xPortGetCoreID());
    console.run();
}

// ===========================================================================
// Beacon task — drives recovery audio beacon during LANDED phase
// ===========================================================================
static void beacon_task(void* /*arg*/) {
    const int BEACON_PIN = nav::PINS.beacon;
    gpio_set_direction((gpio_num_t)BEACON_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)BEACON_PIN, 0);

    while (true) {
        nav::Phase phase = nav::Phase::PRE_FLIGHT;
        {
            xSemaphoreTake(fc_mutex, portMAX_DELAY);
            if (fc.output_valid) phase = fc.last_output.sup.phase;
            xSemaphoreGive(fc_mutex);
        }

        if (phase == nav::Phase::LANDED) {
            // 1Hz 50% duty cycle, 92dB spec @ 1m (driven by external transistor)
            gpio_set_level((gpio_num_t)BEACON_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_set_level((gpio_num_t)BEACON_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
        } else {
            gpio_set_level((gpio_num_t)BEACON_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}

// ===========================================================================
// Command parser handlers — called from LoRa task via RxCallback
// ===========================================================================
static void setup_command_handlers() {
    cmd_parser.on_cx([](bool enable) {
        telem_enabled.store(enable);
        event_log.log_event(enable ? logging::EventCode::CMD_CX_ON
                                   : logging::EventCode::CMD_CX_OFF,
                            mission_time_s.load());
        ESP_LOGI(TAG, "CX: telemetry %s", enable ? "ON" : "OFF");
    });

    cmd_parser.on_st([](uint32_t t) {
        mission_time_s.store(t);
        event_log.log_event(logging::EventCode::CMD_ST,
                            t, 0, 0, 0, "Mission time override");
        ESP_LOGI(TAG, "ST: mission time set to %lu s", (unsigned long)t);
    });

    cmd_parser.on_cal([]() {
        // Capture current baro altitude as ground reference
        drivers::BaroData snap{};
        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        snap = latest_baro;
        xSemaphoreGive(sensor_mutex);
        if (snap.valid) {
            nvs_cfg.set_ground_alt_m(snap.altitude_agl_m);
            ESP_LOGI(TAG, "CAL: ground_alt = %.2f m", (double)snap.altitude_agl_m);
        }
        event_log.log_event(logging::EventCode::CMD_CAL, mission_time_s.load());
    });

    cmd_parser.on_sim([](const char* mode) {
        if (strcmp(mode, "ENABLE") == 0 || strcmp(mode, "ACTIVATE") == 0) {
            sim_mode.store(true);
            ESP_LOGI(TAG, "SIM: activated");
        } else if (strcmp(mode, "DISABLE") == 0) {
            sim_mode.store(false);
            ESP_LOGI(TAG, "SIM: disabled");
        }
    });

    cmd_parser.on_simp([](float pa) {
        sim_pressure_pa.store(pa);
        ESP_LOGD(TAG, "SIMP: %.1f Pa", (double)pa);
    });

    cmd_parser.on_simg([](double e, double n, double u, double ve, double vn, double vu) {
        sim_gnss.e.store(e);
        sim_gnss.n.store(n);
        sim_gnss.u.store(u);
        sim_gnss.ve.store(ve);
        sim_gnss.vn.store(vn);
        sim_gnss.vu.store(vu);
    });

    cmd_parser.on_simi([](double ax, double ay, double az, double gx, double gy, double gz) {
        sim_imu.ax.store(ax);
        sim_imu.ay.store(ay);
        sim_imu.az.store(az);
        sim_imu.gx.store(gx);
        sim_imu.gy.store(gy);
        sim_imu.gz.store(gz);
    });

    cmd_parser.on_abort([]() {
        ESP_LOGW(TAG, "CMD: ABORT triggered");
        xSemaphoreTake(fc_mutex, portMAX_DELAY);
        fc.supervisor.emergency_abort();
        xSemaphoreGive(fc_mutex);
        motors.disarm();
        event_log.log_event(logging::EventCode::ERROR_FDIR, mission_time_s.load(), 0, 0, 0, "Manual Abort");
    });

    cmd_parser.on_chute([]() {
        ESP_LOGW(TAG, "CMD: Manual CHUTE deployment");
        motors.servo_release();
    });

    cmd_parser.on_rtl([]() {
        ESP_LOGI(TAG, "CMD: RTL triggered (Controlled Descent)");
        // Trigger RTL sequence (e.g., controlled altitude descent)
    });

    cmd_parser.on_mapping([]() {
        if (rf_mapper.is_running()) {
            rf_mapper.stop();
            ESP_LOGI(TAG, "CMD: RF Mapping stopped");
        } else {
            rf_mapper.start();
            ESP_LOGI(TAG, "CMD: RF Mapping started (Secondary Mission)");
        }
    });

    cmd_parser.on_ota([](const char* arg) {
        if (strcmp(arg, "START") == 0) {
            ota_svc.start();
        } else if (strncmp(arg, "CHUNK,", 6) == 0) {
            const char* hex = arg + 6;
            size_t hex_len = strlen(hex);
            uint8_t bin[32];
            size_t bin_len = 0;
            for (size_t i = 0; i < hex_len && bin_len < sizeof(bin); i += 2) {
                char b[3] = { hex[i], hex[i+1], '\0' };
                bin[bin_len++] = (uint8_t)strtol(b, nullptr, 16);
            }
            ota_svc.write_chunk(bin, bin_len);
        } else if (strcmp(arg, "FINISH") == 0) {
            if (ota_svc.finish() == ESP_OK) {
                esp_restart();
            }
        } else if (strcmp(arg, "ABORT") == 0) {
            ota_svc.abort();
        }
    });

    lora.set_rx_callback(cmd_parser.make_rx_callback());
}

// ===========================================================================
// app_main
// ===========================================================================



#include "system_init.hpp"

// ... (other includes unchanged)

extern "C" void main_fc() {
    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, "   AAKASHVANI — CAN-7USAT india 2026            ");
    ESP_LOGI(TAG, "   (C) 2026 SVNIT. All Rights Reserved.         ");
    ESP_LOGI(TAG, "   [ Flight Software v1.0 ]                     ");
    ESP_LOGI(TAG, "================================================");
    
    // ------------------------------------------------------------------
    // 1. Synchronisation primitives
    // ------------------------------------------------------------------
    fc_mutex     = xSemaphoreCreateMutex();
    sensor_mutex = xSemaphoreCreateMutex();
    evt_group    = xEventGroupCreate();
    configASSERT(fc_mutex && sensor_mutex && evt_group);

    // ------------------------------------------------------------------
    // 2. HAL init
    // ------------------------------------------------------------------
    ESP_LOGI(TAG, "Initialising HAL...");
    ESP_ERROR_CHECK(i2c0.init(I2C_NUM_0, nav::PINS.i2c0_sda, nav::PINS.i2c0_scl, 400000));
    ESP_ERROR_CHECK(i2c1.init(I2C_NUM_1, nav::PINS.i2c1_sda, nav::PINS.i2c1_scl, 400000));
    ESP_ERROR_CHECK(spi.init(nav::PINS.spi_mosi, nav::PINS.spi_miso, nav::PINS.spi_sck));
    ESP_ERROR_CHECK(uart.init(UART_NUM_1, nav::PINS.gnss_tx, nav::PINS.gnss_rx, 115200));
    ESP_ERROR_CHECK(p4_uart.init(UART_NUM_2, nav::PINS.p4_tx, nav::PINS.p4_rx, 921600));
    p4_link_drv.init(p4_uart);

    // ------------------------------------------------------------------
    // 3. Driver init
    // ------------------------------------------------------------------
    ESP_LOGI(TAG, "Initialising sensors...");
    ESP_ERROR_CHECK(imu_drv.init(i2c0));
    ESP_ERROR_CHECK(baro_drv.init(i2c0));
    ESP_ERROR_CHECK(scan_drv.init(spi, nav::PINS.cc1101_cs));
    gnss_drv.init(uart);
    sdp31_drv.init(i2c1);
    sht4x_drv.init(i2c1);
    sgp41_drv.init(i2c1);

    // ------------------------------------------------------------------
    // 4. Comms init
    // ------------------------------------------------------------------
    ESP_LOGI(TAG, "Initialising LoRa link...");
    ESP_ERROR_CHECK(lora.init(spi, nav::PINS.lora_cs,
                              nav::PINS.lora_rst, nav::PINS.lora_irq));
    setup_command_handlers();

    // ------------------------------------------------------------------
    // 5. Logging init
    // ------------------------------------------------------------------
    ESP_LOGI(TAG, "Initialising logging...");
    esp_err_t sd_ret = sd_logger.init(nav::PINS.sd_clk,
                                       nav::PINS.sd_cmd,
                                       nav::PINS.sd_d0);
    if (sd_ret != ESP_OK) {
        ESP_LOGW(TAG, "SD card not available — logging to RAM only");
    } else {
        // SD is available, check for coredump from previous crash
        logging::CoredumpExporter::check_and_export();
    }
    ESP_ERROR_CHECK(event_log.init());
    
    // Note: boot_cnt handled in shared core_init
    event_log.log_event(logging::EventCode::BOOT, 0, 0, 0, 0, "system boot");

    // ------------------------------------------------------------------
    // 6. Power manager
    // ------------------------------------------------------------------
    ESP_ERROR_CHECK(pwr_mgr.init(i2c1));
    pwr_mgr.set_low_power_callback([](power::PowerStatus s) {
        event_log.log_event(logging::EventCode::POWER_LOW,
                            mission_time_s.load(),
                            (uint8_t)s, 0, 0, "Low power");
        if (s == power::PowerStatus::CRITICAL) {
            if (motors.is_armed()) motors.disarm();
        }
    });

    // ------------------------------------------------------------------
    // 7. Built-In Test
    // ------------------------------------------------------------------
    ESP_LOGI(TAG, "Running BIT...");
    bit::BuiltInTest bit_runner;
    bit::BITResult bit_res = bit_runner.run(i2c0, spi, uart,
                                             &sd_logger, nvs_cfg);
    event_log.log_event(bit_res.pass() ? logging::EventCode::BIT_PASS
                                       : logging::EventCode::BIT_FAIL,
                        0, (uint8_t)(bit_res.flags & 0xFF),
                        (uint8_t)((bit_res.flags >> 8) & 0xFF), 0,
                        bit_res.pass() ? "BIT pass" : "BIT FAIL");

    if (!bit_res.pass()) {
        ESP_LOGE(TAG, "CRITICAL BIT FAILURE (0x%08X) — halting",
                 (unsigned)bit_res.flags);
        gpio_set_direction((gpio_num_t)nav::PINS.led_status, GPIO_MODE_OUTPUT);
        while (true) {
            gpio_set_level((gpio_num_t)nav::PINS.led_status, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level((gpio_num_t)nav::PINS.led_status, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    xEventGroupSetBits(evt_group, EVT_BIT_PASS);

    // ------------------------------------------------------------------
    // 8. FlightComputer init
    // ------------------------------------------------------------------
    ESP_LOGI(TAG, "Initialising flight computer...");
    {
        nav::NavState initial_nav;
        initial_nav.p(0) = 0.0; initial_nav.p(1) = 0.0; initial_nav.p(2) = 0.0;
        initial_nav.v(0) = 0.0; initial_nav.v(1) = 0.0; initial_nav.v(2) = 0.0;
        initial_nav.q(0) = nav::ESTIMATOR_CFG.initial_q_w;
        initial_nav.q(1) = nav::ESTIMATOR_CFG.initial_q_x;
        initial_nav.q(2) = nav::ESTIMATOR_CFG.initial_q_y;
        initial_nav.q(3) = nav::ESTIMATOR_CFG.initial_q_z;
        for (int i = 0; i < 3; ++i) { initial_nav.ba(i) = 0.0; initial_nav.bg(i) = 0.0; }
        fc.init(initial_nav);
    }

    // ------------------------------------------------------------------
    // 9. Control init
    // ------------------------------------------------------------------
    configure_pids();
    ESP_ERROR_CHECK(motors.init());
    motors.servo_home();
    ESP_LOGI(TAG, "Motor mixer ready. Servo homed.");

    // ------------------------------------------------------------------
    // 10. RF Mapper init
    // ------------------------------------------------------------------
    rf_mapper.init(fc, scan_drv, motors, sd_logger, fc_mutex);

    // ------------------------------------------------------------------
    // 11. Watchdog
    // ------------------------------------------------------------------
    ESP_ERROR_CHECK(wdg.init(true));

    // ------------------------------------------------------------------
    // 12. Spawn tasks
    // ------------------------------------------------------------------
    ESP_LOGI(TAG, "Spawning tasks...");
    xTaskCreatePinnedToCore(nav_task,     "nav",     STK_NAV,     nullptr, PRI_NAV_TASK,     nullptr, 0);
    xTaskCreatePinnedToCore(control_task, "ctrl",    STK_CTRL,    nullptr, PRI_CTRL_TASK,    nullptr, 0);
    xTaskCreatePinnedToCore(sensor_task,  "sensor",  STK_SENSOR,  nullptr, PRI_SENSOR_TASK,  nullptr, 1);
    xTaskCreatePinnedToCore(telem_task,   "telem",   STK_TELEM,   nullptr, PRI_TELEM_TASK,   nullptr, 1);
    xTaskCreatePinnedToCore(logging_task, "logging", STK_LOGGING, nullptr, PRI_LOGGING_TASK, nullptr, 1);
    xTaskCreatePinnedToCore(power_task,   "power",   STK_POWER,   nullptr, PRI_POWER_TASK,   nullptr, 1);
    xTaskCreatePinnedToCore(p4_link_task, "p4_link", 2048,      nullptr, 3,                nullptr, 1);
    xTaskCreatePinnedToCore(cli_task,     "cli",     4096,      nullptr, 1,                nullptr, 1);
    xTaskCreate(beacon_task, "beacon", 1024, nullptr, 2, nullptr);

    ESP_LOGI(TAG, "Flight software running.");
}

extern "C" void app_main(void) {
    // 1. Core System Init (Shared)
    system_init::core_init();

    // 2. Role-based Entry
#if defined(CONFIG_ROLE_FC)
    main_fc();
#elif defined(CONFIG_ROLE_GCS)
    extern void main_gcs();
    main_gcs();
#elif defined(CONFIG_ROLE_P4)
    extern void main_p4();
    main_p4();
#else
    ESP_LOGW("main", "No hardware role defined! Halted.");
#endif
}
