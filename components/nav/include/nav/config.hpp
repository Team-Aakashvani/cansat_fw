/**
 * @file config.hpp
 * @brief CAN-7USAT 2026 — Centralized aerospace-grade configuration constants.
 *
 * This file is the single source of truth for every numerical parameter
 * in the flight software stack. No magic numbers exist anywhere else.
 *
 * Units: SI throughout, with suffixes: _m (metres), _mps (m/s),
 *        _mps2 (m/s²), _radps (rad/s), _hz (Hz), _s (seconds),
 *        _pa (Pascals), _k (Kelvin), _deg (degrees as noted).
 *
 * Migration: Every struct here maps 1-to-1 to a Python dataclass in
 * config.py. Numerical values are identical to the simulation reference.
 *
 * @compliance CAN-7USAT India 2026 Guidelines §3 (Mission Requirements)
 * @compliance Can7usat Aerospace Doctrine V1 §1–§4
 */
#pragma once

#include <cstdint>

namespace nav {

// ===========================================================================
// Universal physical constants  (WGS-84 / ISA)
// ===========================================================================

constexpr double G0_MPS2        = 9.80665;       ///< Standard gravity (m/s²)
constexpr double R_AIR_JPKGK    = 287.05287;     ///< Dry-air gas constant (J/kg/K)
constexpr double T0_K           = 288.15;         ///< ISA sea-level temperature (K)
constexpr double P0_PA          = 101325.0;       ///< ISA sea-level pressure (Pa)
constexpr double ISA_LAPSE_KPM  = 0.0065;         ///< ISA troposphere lapse rate (K/m)
constexpr double RHO0_KGPM3     = 1.225;          ///< ISA sea-level density (kg/m³)
constexpr double PI             = 3.14159265358979323846;

// ===========================================================================
// EKF state dimensions
// ===========================================================================

constexpr int N_NAV = 16;  ///< Nav state size: [p(3) v(3) q(4) ba(3) bg(3)]
constexpr int N_ERR = 15;  ///< Error state size: [δp δv δθ δba δbg] all ×3

// Error-state slice offsets (0-indexed)
constexpr int EIDX_P_0  = 0;   constexpr int EIDX_P_END  = 3;
constexpr int EIDX_V_0  = 3;   constexpr int EIDX_V_END  = 6;
constexpr int EIDX_TH_0 = 6;   constexpr int EIDX_TH_END = 9;
constexpr int EIDX_BA_0 = 9;   constexpr int EIDX_BA_END = 12;
constexpr int EIDX_BG_0 = 12;  constexpr int EIDX_BG_END = 15;

// Nav-state slice offsets
constexpr int IDX_P_0 = 0;   constexpr int IDX_P_END = 3;
constexpr int IDX_V_0 = 3;   constexpr int IDX_V_END = 6;
constexpr int IDX_Q_0 = 6;   constexpr int IDX_Q_END = 10;
constexpr int IDX_BA_0 = 10; constexpr int IDX_BA_END = 13;
constexpr int IDX_BG_0 = 13; constexpr int IDX_BG_END = 16;

// ===========================================================================
// IMM regime indices  (must match IMMConfig::model_names order)
// ===========================================================================

constexpr int REGIME_BOOST       = 0;
constexpr int REGIME_BALLISTIC   = 1;
constexpr int REGIME_PARACHUTE   = 2;
constexpr int REGIME_DRONE_HOVER = 3;
constexpr int REGIME_LANDED      = 4;
constexpr int N_REGIMES          = 5;

// ===========================================================================
// Sensor identifiers (used as indices into health/FDIR arrays)
// ===========================================================================

constexpr int SENSOR_IMU   = 0;
constexpr int SENSOR_BARO  = 1;
constexpr int SENSOR_GNSS  = 2;
constexpr int SENSOR_MAG   = 3;
constexpr int N_SENSORS    = 4;

// ===========================================================================
// Vehicle parameters  (VehicleParams equivalent)
// ===========================================================================

struct VehicleConfig {
    double mass_kg                   = 0.50;    ///< CAN-7USAT max <1kg
    double reference_area_m2         = 0.01767; ///< π·(0.075)² m² (150mm dia)
    double drag_coeff_freefall       = 0.55;
    double parachute_terminal_mps    = 5.0;     ///< Target ≤ 5m/s per rules
    double drone_terminal_mps        = 2.0;     ///< Active hover target
    double tether_length_m           = 0.15;    ///< Canopy tether length
};
static constexpr VehicleConfig VEHICLE{};

// ===========================================================================
// Mission profile  (MissionProfile equivalent)
// ===========================================================================

struct MissionConfig {
    double apogee_altitude_m         = 1000.0;  ///< CAN-7USAT target altitude
    double deployment_altitude_m     = 600.0;   ///< Secondary deploy @ 600m ±10m
    double deployment_tolerance_m    = 10.0;    ///< Allowed error band
    double boost_duration_s          = 1.0;
    double boost_net_thrust_mps2     = 40.0;
    double min_mission_time_s        = 5.0;     ///< Minimum before any deploy
    double drone_hover_alt_m         = 22.0;    ///< Hand-over altitude (ground)
    double descent_rate_target_mps   = 2.0;     ///< Nominal 1–3m/s
    double descent_rate_max_mps      = 3.0;
    double descent_rate_min_mps      = 1.0;
};
static constexpr MissionConfig MISSION{};

// ===========================================================================
// IMU parameters  (IMUParams equivalent — BNO085)
// ===========================================================================

struct IMUConfig {
    double rate_hz                   = 100.0;   ///< Minimum per rules; driver runs 100Hz
    double accel_vrw_mps2_sqrthz     = 0.04;    ///< Velocity random walk density
    double accel_bivs_mps3_sqrthz    = 5.0e-4;  ///< Bias instability
    double accel_bias_init_std_mps2  = 0.05;
    double accel_scale_factor_ppm    = 200.0;
    double accel_misalignment_mrad   = 1.0;
    double accel_saturation_mps2     = 156.96;  ///< BNO085 ±16g range
    double gyro_arw_radps_sqrthz     = 1.7e-3;  ///< Angular random walk
    double gyro_bivs_radps2_sqrthz   = 1.0e-5;
    double gyro_bias_init_std_radps  = 0.01;
    double gyro_saturation_radps     = 34.9;    ///< BNO085 ±2000 deg/s
};
static constexpr IMUConfig IMU_CFG{};

// ===========================================================================
// Barometer parameters  (BarometerParams equivalent — BMP585)
// ===========================================================================

struct BaroConfig {
    double rate_hz                   = 50.0;    ///< BMP585 at 50Hz ODR
    double pressure_noise_std_pa     = 3.0;     ///< BMP585 noise spec ~0.065Pa RMS
    double thermal_drift_pa_per_s    = 0.05;
    double quantization_pa           = 0.5;
    double bias_init_std_pa          = 20.0;
    double sigma_h_floor_m           = 0.1;     ///< 0.1m altitude resolution
};
static constexpr BaroConfig BARO_CFG{};

// ===========================================================================
// GNSS parameters  (GPSParams equivalent — N-GS-01 NavIC)
// ===========================================================================

struct GNSSConfig {
    double rate_hz                   = 1.0;     ///< 1Hz PVT minimum per rules
    double horizontal_pos_std_m      = 2.5;     ///< NavIC CEP ≈ 5m; σ ≈ 2.5m
    double vertical_pos_std_m        = 5.0;
    double horizontal_vel_std_mps    = 0.3;
    double latency_s                 = 0.1;     ///< Estimated UART parse latency
    double dropout_alt_min_m         = 10.0;    ///< Below this, GNSS unreliable
};
static constexpr GNSSConfig GNSS_CFG{};

// ===========================================================================
// Estimator parameters  (EstimatorParams equivalent)
// ===========================================================================

struct EstimatorConfig {
    // Initial covariance diagonal
    double pos_init_std_m            = 5.0;
    double vel_init_std_mps          = 1.0;
    double att_init_std_rad          = 0.10;
    double ba_init_std_mps2          = 0.05;
    double bg_init_std_radps         = 0.01;

    // Initial attitude: body-x (nose) aligned with world +z (upward launch rail)
    // q = [w, x, y, z] = [cos(45°), 0, -sin(45°), 0] = [0.7071, 0, -0.7071, 0]
    double initial_q_w = 0.7071067811865476;
    double initial_q_x = 0.0;
    double initial_q_y = -0.7071067811865476;
    double initial_q_z = 0.0;

    // Adaptive R
    double adaptive_alpha            = 0.05;
    int    adaptive_window_samples   = 50;
    double adaptive_R_min_factor     = 0.25;
    double adaptive_R_max_factor     = 16.0;
    bool   adaptive_enabled          = true;

    // FDIR / innovation gating
    double chi2_per_channel_alpha    = 0.01;  ///< 99% confidence z-score = 2.576
    double sprt_log_likelihood_thr   = 4.6;   ///< ln(100) ≈ 4.6
    int    sprt_window_samples       = 10;
    double analytical_redundancy_tol_mps = 6.0;

    // Sensor health
    double health_smoothing_tau_s    = 1.0;
    double health_floor              = 0.02;

    // Covariance ceiling
    double cov_ceiling               = 1.0e6;
};
static constexpr EstimatorConfig ESTIMATOR_CFG{};

// ===========================================================================
// IMM parameters  (IMMParams equivalent)
// ===========================================================================

struct IMMConfig {
    // Process-noise spectral densities per regime
    // [Boost, Ballistic, Parachute, DroneHover, Landed]
    double sigma_a[N_REGIMES] = {10.0, 2.5, 3.0, 1.0, 0.1};
    double sigma_w[N_REGIMES] = {0.30, 0.08, 0.50, 0.05, 0.005};

    // Bias-RW densities (sensor property, shared across all regimes)
    double sigma_ba           = 1.0e-3;
    double sigma_bg           = 1.0e-4;

    // Initial regime probabilities (uniform)
    double initial_probs[N_REGIMES] = {0.2, 0.2, 0.2, 0.2, 0.2};

    // Base Markov transition matrix (row-stochastic)
    double Pi[N_REGIMES][N_REGIMES] = {
        {0.95, 0.05, 0.00, 0.00, 0.00},  // Boost
        {0.00, 0.90, 0.10, 0.00, 0.00},  // Ballistic
        {0.00, 0.00, 0.85, 0.10, 0.05},  // Parachute
        {0.00, 0.00, 0.00, 0.90, 0.10},  // DroneHover
        {0.00, 0.00, 0.00, 0.02, 0.98},  // Landed
    };

    // EMA smoothing on the regime posterior
    double mu_alpha = 0.35;
};
static constexpr IMMConfig IMM_CFG{};

// ===========================================================================
// Supervisor parameters  (SupervisorParams equivalent)
// ===========================================================================

struct SupervisorConfig {
    // Deployment posteriors
    double deploy_posterior_threshold  = 0.75;
    double deploy_posterior_off_thr    = 0.45;
    double deploy_confirm_s            = 0.25;
    double deploy_safety_alt_max_m     = 1500.0;
    double deploy_safety_alt_min_m     = 30.0;
    double landed_posterior_threshold  = 0.95;
    double landed_confirm_s            = 1.00;

    // Launch detection (independent multi-channel)
    double launch_baseline_s           = 1.0;
    double launch_specific_force_excess_mps2 = 9.0;  ///< ~0.92g above bias
    double launch_altitude_rise_m      = 2.0;
    double launch_vertical_vel_mps     = 3.0;
    double launch_persist_s            = 0.15;        ///< 150ms per channel
    double launch_hard_specific_force_mps2 = 25.0;   ///< No-doubt override

    // VS-IMM soft gating
    double vs_gate_soft_floor          = 0.05;

    // Landed-column hysteresis
    double landed_gate_alt_on_m        = 7.0;
    double landed_gate_alt_off_m       = 3.0;
    double landed_post_on              = 0.60;
    double landed_post_off             = 0.35;
    double landed_velocity_sigma_mps   = 8.0;
    double landed_altitude_sigma_m     = 2.5;

    // Descent-velocity hysteresis
    double descent_on_mps              = -3.0;
    double descent_off_mps             = -1.0;

    // BOOST → BALLISTIC transition
    double boost_to_ballistic_mu_on    = 0.50;
    double boost_tipover_vel_mps       = 1.0;
    double boost_burnout_fmag_band_mps2 = 3.0;
    double ascent_sign_vel_min_mps     = 1.0;
    double ascent_sign_votes_required  = 5.0;

    // Chute-deploy posterior logistic
    double chute_descent_mid_mps       = 5.0;
    double chute_descent_slope         = 1.5;

    // Drone activation altitude window
    double drone_alt_min_m             = 1.0;
    double drone_alt_max_m             = 30.0;
    double drone_alt_center_m          = 15.0;
    double drone_alt_sigma_m           = 8.0;

    // CAN-7USAT deployment target altitude
    double target_deploy_alt_m         = 600.0;
    double deploy_alt_tolerance_m      = 10.0;
};
static constexpr SupervisorConfig SUPERVISOR_CFG{};

// ===========================================================================
// Control parameters  (PID + motor mixer)
// ===========================================================================

struct ControlConfig {
    // Pitch / Roll PID
    double kp_attitude               = 2.5;
    double ki_attitude               = 0.1;
    double kd_attitude               = 0.8;
    double anti_windup_limit_rad     = 0.5;   ///< Integrator clamp ±0.5 rad
    double pid_dt_s                  = 0.01;  ///< 100Hz control loop

    // Deadband: only activate if |pitch| or |roll| > 5°
    double attitude_deadband_rad     = 0.0873; ///< 5° in radians

    // Descent rate PID (z-axis)
    double kp_descent                = 3.0;
    double ki_descent                = 0.05;
    double kd_descent                = 0.5;

    // Motor parameters
    int    n_motors                  = 4;
    double motor_min_pwm_us          = 1000;   ///< ESC: 1000–2000μs
    double motor_max_pwm_us          = 2000;
    double motor_idle_pwm_us         = 1050;   ///< Idle to keep ESCs alive
    double motor_arm_pwm_us          = 1000;   ///< Arming signal
    double max_throttle_brownout     = 0.6;    ///< Throttle limit during low battery

    // Slew rate limit (μs/step at 100Hz = μs/10ms)
    double motor_slew_rate_us_per_s  = 200.0;

    // Stabilisation delay after deployment (ms)
    uint32_t stabilise_delay_ms      = 500;
};
static constexpr ControlConfig CONTROL_CFG{};

// ===========================================================================
// GPIO pin mapping  (ESP32-S3 WROOM-1)
// ===========================================================================

struct PinConfig {
    // I2C Bus 0 (BNO085 + BMP585)
    int i2c0_sda = 8;
    int i2c0_scl = 9;

    // I2C Bus 1 (SDP31 + SGP41 + SHT4x + INA260 + MAX17048)
    int i2c1_sda = 10;
    int i2c1_scl = 11;

    // SPI Bus (Shared — CC1101)
    int spi_mosi = 35;
    int spi_miso = 37;
    int spi_sck  = 36;
    int cc1101_cs = 21;  ///< RF scanner CS

    // UART3 (XBee Pro Radio)
    int xbee_tx  = 34;
    int xbee_rx  = 32;

    // UART1 (N-GS-01 NavIC GNSS)
    int gnss_tx  = 17;
    int gnss_rx  = 18;

    // UART2 (ESP32-P4 Media Coprocessor)
    int p4_tx    = 26;
    int p4_rx    = 27;

    // SDMMC (SD card logging)
    int sd_clk   = 14;
    int sd_cmd   = 15;
    int sd_d0    = 2;
    int sd_d1    = 4;
    int sd_d2    = 12;
    int sd_d3    = 13;

    // PWM motors (LEDC channels 0-3)
    int motor[4] = {5, 6, 7, 16};

    // Servo release (LEDC channel 4)
    int servo    = 38;

    // Recovery beacon (GPIO output to transistor-driven buzzer)
    int beacon   = 39;

    // External power switch (active-high input)
    int power_switch = 40;

    // LED indicator
    int led_status = 48;  ///< ESP32-S3 onboard RGB
};
static constexpr PinConfig PINS{};

// ===========================================================================
// Telemetry configuration  (CAN-7USAT format compliance)
// ===========================================================================

struct TelemetryConfig {
    uint16_t team_id             = 1234;     ///< REPLACE with actual team ID
    uint32_t xbee_baud           = 115200;
    uint8_t  xbee_pan_id         = 0x12;     ///< Set to Team ID lower byte per guidelines

    double   telemetry_rate_hz   = 1.0;      ///< 1Hz mandatory per rules
    uint32_t telemetry_buf_size  = 256;      ///< Max CSV line length

    // Command uplink
    uint16_t cmd_crc_poly        = 0x1021;   ///< CRC-16/CCITT
    uint16_t cmd_max_age_s       = 5;        ///< Anti-replay window
    uint8_t  cmd_max_retries     = 3;
};
static constexpr TelemetryConfig TELEM_CFG{};

// ===========================================================================
// Logging configuration
// ===========================================================================

struct LoggingConfig {
    uint32_t sd_buf_size_bytes   = 4096;     ///< Write buffer (power-loss safe)
    uint32_t sd_flush_period_ms  = 1000;     ///< Flush interval
    uint32_t max_log_files       = 100;      ///< File rotation
    bool     binary_events       = false;    ///< CSV events (set true for prod)
};
static constexpr LoggingConfig LOGGING_CFG{};

// ===========================================================================
// Power management thresholds
// ===========================================================================

struct PowerConfig {
    double bat_nominal_v         = 7.4;      ///< 2S LiPo nominal
    double bat_low_v             = 6.8;      ///< Low battery warning
    double bat_critical_v        = 6.4;      ///< Throttle motors
    double bat_cutoff_v          = 6.0;      ///< Emergency beacon only
    double current_limit_a       = 10.0;     ///< INA260 overcurrent limit
    double voltage_sag_thresh_v  = 0.3;      ///< Sag detection (V drop from idle)
};
static constexpr PowerConfig POWER_CFG{};

// ===========================================================================
// Watchdog periods
// ===========================================================================

struct WatchdogConfig {
    uint32_t rt_task_period_ms   = 20;       ///< RT loop must pet every 20ms
    uint32_t sys_task_period_ms  = 200;      ///< System services heartbeat
    uint32_t hw_wdg_period_ms    = 5000;     ///< Hardware TWDT timeout = 10s (above)
    uint32_t sensor_timeout_ms   = 500;      ///< Sensor dropout declaration
};
static constexpr WatchdogConfig WDG_CFG{};

} // namespace nav
