/**
 * @file main.cpp
 * @brief CAN-7USAT 2026 CanSat — ESP32-S3 Flight Software Entry Point.
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
// FreeRTOS task priorities
// ===========================================================================
#define PRI_NAV_TASK     (configMAX_PRIORITIES - 1)
#define PRI_CTRL_TASK    (configMAX_PRIORITIES - 2)
#define PRI_SENSOR_TASK  (configMAX_PRIORITIES - 3)
#define PRI_TELEM_TASK   5
#define PRI_LOGGING_TASK 4
#define PRI_POWER_TASK   3

#define STK_NAV      8192
#define STK_CTRL     4096
#define STK_SENSOR   4096
#define STK_TELEM    4096
#define STK_LOGGING  8192
#define STK_POWER    2048

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
static std::atomic<uint32_t> mission_time_s{0};
static std::atomic<bool>     telem_enabled{true};

static drivers::BaroData     latest_baro{};
static drivers::GNSSData     latest_gnss{};
static drivers::IMUData      latest_imu_snap{};
static drivers::PowerData    latest_pwr{};

static inline double now_s() noexcept {
    return (double)esp_timer_get_time() * 1.0e-6;
}

static void configure_pids() noexcept {
    const nav::ControlConfig& C = nav::CONTROL_CFG;

    control::PIDGains angle_gains = { (float)C.kp_attitude, (float)C.ki_attitude, 0.0f, (float)C.anti_windup_limit_rad };
    control::PIDGains rate_gains = { (float)C.kp_attitude * 0.5f, (float)C.ki_attitude * 0.5f, (float)C.kd_attitude, 0.0f };
    control::PIDGains yaw_rate_gains = { 1.0f, 0.05f, 0.0f, 0.0f };

    attitude_pid.set_angle_gains(angle_gains, angle_gains);
    attitude_pid.set_rate_gains(rate_gains, rate_gains, yaw_rate_gains);

    control::PIDGains dsc_gains = { (float)C.kp_descent, (float)C.ki_descent, (float)C.kd_descent, 0.3f };
    descent_pid.set_gains(dsc_gains);
}

static void nav_task(void* /*arg*/) {
    watchdog::Watchdog::register_task();
    const TickType_t period = pdMS_TO_TICKS(10);
    TickType_t       wake   = xTaskGetTickCount();

    while (true) {
        watchdog::Watchdog::ping();
        const double t = now_s();
        drivers::IMUData imu = imu_drv.read();

        if (imu.valid) {
            xSemaphoreTake(sensor_mutex, portMAX_DELAY);
            latest_imu_snap = imu;
            xSemaphoreGive(sensor_mutex);

            xSemaphoreTake(fc_mutex, portMAX_DELAY);
            fc.ingest_imu(t, imu.acc_x, imu.acc_y, imu.acc_z, imu.gyr_x, imu.gyr_y, imu.gyr_z);
            xSemaphoreGive(fc_mutex);
        }

        static uint32_t tick_counter = 0;
        if (++tick_counter >= 100) {
            tick_counter = 0;
            ++mission_time_s;
        }
        vTaskDelayUntil(&wake, period);
    }
}

static void sensor_task(void* /*arg*/) {
    const TickType_t period = pdMS_TO_TICKS(20);
    TickType_t       wake   = xTaskGetTickCount();
    uint32_t         gnss_divider = 0;

    while (true) {
        const double t = now_s();
        drivers::BaroData baro = baro_drv.read();

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

        if (++gnss_divider >= 50) {
            gnss_divider = 0;
            drivers::GNSSData gnss = gnss_drv.read();
            xSemaphoreTake(sensor_mutex, portMAX_DELAY);
            latest_gnss = gnss;
            xSemaphoreGive(sensor_mutex);

            if (gnss.valid) {
                xSemaphoreTake(fc_mutex, portMAX_DELAY);
                fc.ingest_gnss(t, gnss.pos_e, gnss.pos_n, gnss.pos_u, gnss.vel_e, gnss.vel_n, gnss.vel_u);
                xSemaphoreGive(fc_mutex);
            }
        }
        vTaskDelayUntil(&wake, period);
    }
}

static void control_task(void* /*arg*/) {
    watchdog::Watchdog::register_task();
    const TickType_t period = pdMS_TO_TICKS(10);
    TickType_t       wake   = xTaskGetTickCount();
    xEventGroupWaitBits(evt_group, EVT_BIT_PASS, pdFALSE, pdTRUE, portMAX_DELAY);

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
            if (!valid) { vTaskDelayUntil(&wake, period); continue; }
        }

        const float altitude = baro.altitude_agl_m;
        const float dt       = (float)nav::CONTROL_CFG.pid_dt_s;

        if (altitude > 600.0f) {
            motors.servo_release();
            for (int i = 0; i < 4; ++i) motors.set_motor_us(i, 1000);
            attitude_pid.reset();
            descent_pid.reset();
        } else {
            motors.servo_home();
            nav::EulerAngles ea = nav::euler_from_quat(fc_out.imm.nav.q);
            const float roll = (float)ea.roll_rad, pitch = (float)ea.pitch_rad, vel_z = (float)fc_out.imm.nav.v(2);
            drivers::IMUData imu;
            {
                xSemaphoreTake(sensor_mutex, portMAX_DELAY);
                imu = latest_imu_snap;
                xSemaphoreGive(sensor_mutex);
            }
            float throttle = descent_pid.update(-2.0f, vel_z, dt);
            control::CascadedPID::Vector3 target_att = {0, 0, 0}, current_att = {roll, pitch, 0}, current_rates = {(float)imu.gyr_x, (float)imu.gyr_y, (float)imu.gyr_z};
            auto torque = attitude_pid.update(target_att, current_att, current_rates, dt);
            auto pwm = mixer_x.mix(throttle, torque.x, torque.y, torque.z);
            motors.set_motor_us(0, (uint32_t)pwm.m1); motors.set_motor_us(1, (uint32_t)pwm.m2);
            motors.set_motor_us(2, (uint32_t)pwm.m3); motors.set_motor_us(3, (uint32_t)pwm.m4);
        }
        vTaskDelayUntil(&wake, period);
    }
}

static void telem_task(void* /*arg*/) {
    const TickType_t period = pdMS_TO_TICKS(1000);
    TickType_t       wake   = xTaskGetTickCount();
    char csv_buf[telemetry::TelemetryEncoder::BUF_LEN];

    while (true) {
        if (telem_enabled.load()) {
            drivers::BaroData baro_snap; drivers::GNSSData gnss_snap; drivers::IMUData imu_snap;
            drivers::PowerData pwr_snap; nav::FlightComputerOutput fc_snap;
            {
                xSemaphoreTake(sensor_mutex, portMAX_DELAY);
                baro_snap = latest_baro; gnss_snap = latest_gnss; imu_snap = latest_imu_snap; pwr_snap = latest_pwr;
                xSemaphoreGive(sensor_mutex);
            }
            {
                xSemaphoreTake(fc_mutex, portMAX_DELAY);
                fc_snap = fc.last_output;
                xSemaphoreGive(fc_mutex);
            }
            const uint32_t pkt_cnt = ++packet_count, mt_s = mission_time_s.load();
            telemetry::TelemetryFrame frame = telemetry::TelemetryEncoder::make_frame(fc_snap, baro_snap, gnss_snap, imu_snap, pwr_snap, scan_drv.get_frequency(), scan_drv.read_rssi_dbm(), pkt_cnt, mt_s);
            int n = telem_enc.encode(frame, csv_buf, sizeof(csv_buf));
            if (n > 0) {
                xbee.enqueue_packet(csv_buf, (size_t)n);
                char sd_buf[telemetry::TelemetryEncoder::BUF_LEN + 16];
                int sn = snprintf(sd_buf, sizeof(sd_buf), "%u,%s\n", (unsigned)nav::TELEM_CFG.team_id, csv_buf);
                if (sn > 0) sd_logger.write_line(sd_buf);
            }
        }
        vTaskDelayUntil(&wake, period);
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
    TickType_t       wake   = xTaskGetTickCount();
    while (true) {
        pwr_mgr.update();
        power::PowerState ps = pwr_mgr.get_state();
        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        latest_pwr.voltage_v = (double)ps.voltage_v; latest_pwr.current_a = (double)ps.current_a;
        latest_pwr.power_w = (double)ps.power_w; latest_pwr.valid = ps.valid;
        xSemaphoreGive(sensor_mutex);
        vTaskDelayUntil(&wake, period);
    }
}

static void cli_task(void* /*arg*/) { console.run(); }

static void beacon_task(void* /*arg*/) {
    const int BEACON_PIN = nav::PINS.beacon;
    gpio_set_direction((gpio_num_t)BEACON_PIN, GPIO_MODE_OUTPUT);
    while (true) {
        nav::Phase phase = nav::Phase::PRE_FLIGHT;
        xSemaphoreTake(fc_mutex, portMAX_DELAY);
        if (fc.output_valid) phase = fc.last_output.sup.phase;
        xSemaphoreGive(fc_mutex);
        if (phase == nav::Phase::LANDED) {
            gpio_set_level((gpio_num_t)BEACON_PIN, 1); vTaskDelay(pdMS_TO_TICKS(500));
            gpio_set_level((gpio_num_t)BEACON_PIN, 0); vTaskDelay(pdMS_TO_TICKS(500));
        } else {
            gpio_set_level((gpio_num_t)BEACON_PIN, 0); vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}

static void setup_command_handlers() {
    cmd_parser.on_cx([](bool enable) { telem_enabled.store(enable); });
    cmd_parser.on_st([](uint32_t t) { mission_time_s.store(t); });
    cmd_parser.on_cal([]() {
        drivers::BaroData snap;
        xSemaphoreTake(sensor_mutex, portMAX_DELAY); snap = latest_baro; xSemaphoreGive(sensor_mutex);
        if (snap.valid) nvs_cfg.set_ground_alt_m(snap.altitude_agl_m);
    });
    cmd_parser.on_abort([]() {
        xSemaphoreTake(fc_mutex, portMAX_DELAY); fc.supervisor.emergency_abort(); xSemaphoreGive(fc_mutex);
        for(int i=0; i<4; ++i) motors.set_motor_us(i, 1000);
    });
    cmd_parser.on_chute([]() { motors.servo_release(); });
    xbee.set_rx_callback(cmd_parser.make_rx_callback());
}

#include "system_init.hpp"

extern "C" void main_fc() {
    fc_mutex = xSemaphoreCreateMutex(); sensor_mutex = xSemaphoreCreateMutex(); evt_group = xEventGroupCreate();
    i2c0.init(I2C_NUM_0, nav::PINS.i2c0_sda, nav::PINS.i2c0_scl, 400000);
    i2c1.init(I2C_NUM_1, nav::PINS.i2c1_sda, nav::PINS.i2c1_scl, 400000);
    spi.init(nav::PINS.spi_mosi, nav::PINS.spi_miso, nav::PINS.spi_sck);
    uart.init(UART_NUM_1, nav::PINS.gnss_tx, nav::PINS.gnss_rx, 115200);
    xbee_uart.init(UART_NUM_2, nav::PINS.xbee_tx, nav::PINS.xbee_rx, nav::TELEM_CFG.xbee_baud);
    imu_drv.init(i2c0); baro_drv.init(i2c0); scan_drv.init(spi, nav::PINS.cc1101_cs);
    gnss_drv.init(uart); sdp31_drv.init(i2c1); sht4x_drv.init(i2c1); sgp41_drv.init(i2c1);
    xbee.init(xbee_uart); setup_command_handlers();
    sd_logger.init(nav::PINS.sd_clk, nav::PINS.sd_cmd, nav::PINS.sd_d0);
    event_log.init(); pwr_mgr.init(i2c1);
    bit::BuiltInTest bit_runner; bit_runner.run(i2c0, spi, uart, xbee_uart, &sd_logger, nvs_cfg);
    xEventGroupSetBits(evt_group, EVT_BIT_PASS);
    nav::NavState init_nav{}; init_nav.q(0) = nav::ESTIMATOR_CFG.initial_q_w; init_nav.q(2) = nav::ESTIMATOR_CFG.initial_q_y; fc.init(init_nav);
    configure_pids(); motors.init(); motors.servo_home(); wdg.init(true);
    xTaskCreatePinnedToCore(nav_task, "nav", STK_NAV, nullptr, PRI_NAV_TASK, nullptr, 0);
    xTaskCreatePinnedToCore(control_task, "ctrl", STK_CTRL, nullptr, PRI_CTRL_TASK, nullptr, 0);
    xTaskCreatePinnedToCore(sensor_task, "sensor", STK_SENSOR, nullptr, PRI_SENSOR_TASK, nullptr, 1);
    xTaskCreatePinnedToCore(telem_task, "telem", STK_TELEM, nullptr, PRI_TELEM_TASK, nullptr, 1);
    xTaskCreatePinnedToCore(logging_task, "logging", STK_LOGGING, nullptr, PRI_LOGGING_TASK, nullptr, 1);
    xTaskCreatePinnedToCore(power_task, "power", STK_POWER, nullptr, PRI_POWER_TASK, nullptr, 1);
    xTaskCreatePinnedToCore(cli_task, "cli", 4096, nullptr, 1, nullptr, 1);
    xTaskCreate(beacon_task, "beacon", 1024, nullptr, 2, nullptr);
}

extern "C" void app_main(void) {
    system_init::core_init();
#if defined(CONFIG_ROLE_FC)
    main_fc();
#elif defined(CONFIG_ROLE_GCS)
    extern void main_gcs(); main_gcs();
#endif
}
