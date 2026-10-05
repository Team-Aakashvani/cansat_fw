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
constexpr int MAX_MEAS_DIM = 6; ///< Maximum measurement vector dimension

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

    // Innovation gating (state protection): a sample whose NIS = νᵀS⁻¹ν exceeds
    // the gate is NOT fused, so it cannot overwrite the last valid state.
    double baro_gate_chi2            = 16.0;  ///< 1 dof, ~4σ  (P_false ≈ 6e-5)
    double gnss_gate_chi2            = 25.0;  ///< 6 dof, ~99.97%
    // Re-sync: if a sensor is rejected continuously for resync_after_s while its
    // sample-to-sample noise stays nominal (RMS of successive innovation
    // differences < resync_noise_max·σ_R; white noise ≈ 1.41), the sensor is
    // judged healthy and the *prediction* diverged -> covariance is inflated and
    // the sensor re-adopted. Erratic data keeps being rejected.
    double resync_after_s            = 3.0;
    double resync_noise_max          = 4.0;

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

    // Dynamic launch detection & statistical discrimination
    double launch_baseline_s           = 1.0;         ///< Pad calibration baseline window
    double launch_hard_specific_force_mps2 = 25.0;    ///< Unmistakable launch override (>2.5g)
    double launch_jerk_launch_mps3     = 35.0;        ///< Ignition shock threshold (elevator max 1.5 m/s³)
    double launch_jerk_alpha           = 0.25;        ///< 1st order LP filter coefficient for jerk
    double launch_delta_v_launch_mps   = 5.0;         ///< Leaky specific-force momentum threshold (m/s)
    double launch_leak_lambda          = 1.20;        ///< 1/s dissipation rate for momentum leak
    double launch_deadband_mps2        = 2.0;         ///< ~0.2g floor for noise/handling
    double elevator_min_hdot_mps       = 0.8;         ///< Min baro climb rate for elevator interlock (m/s)
    double elevator_max_imu_dv_mps     = 1.5;         ///< Max allowable IMU Delta-V during elevator motion (m/s)
    double boost_min_duration_s        = 0.25;        ///< Minimum boost phase duration before burnout check

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

// ESP32-S3-DevKitC-1 (N16R8). Reserved, never use: GPIO 19/20 (native USB),
// 26-32 (SPI flash), 33-37 (octal PSRAM), 0/3/45/46 (strapping), 43/44 (UART0),
// 48 (onboard WS2812 RGB LED).
struct PinConfig {
    // I2C bus (BNO055, BMP585, SHT4x, SGP41) - wired & verified
    int i2c0_sda = 38;
    int i2c0_scl = 39;
    int i2c1_sda = 38;
    int i2c1_scl = 39;

    // UART1 GNSS (N-GS-01 NavIC) - wired & verified (RX live on GPIO 13)
    int gnss_tx  = 21;
    int gnss_rx  = 13;

    // ---- Not yet wired: reserve these pins when integrating -----------------
    // HAKRC 35A 4-in-1 ESC, DShot300 signal pads (M1 FL, M2 FR, M3 RR, M4 RL)
    int motor[4] = {4, 5, 6, 7};

    // Two linear servos that unlatch the drone arms (50 Hz PWM)
    int servo_a  = 15;
    int servo_b  = 16;

    // UART2 XBee 3 Pro
    int xbee_tx  = 17;
    int xbee_rx  = 18;

    // SPI bus (SD card + CC1101) - keep spi_devices_fitted=false until wired
    int spi_sck   = 12;
    int spi_mosi  = 11;
    int spi_miso  = 10;
    int sd_cs     = 9;
    int cc1101_cs = 14;
    bool spi_devices_fitted = false;

    // Legacy SDMMC fields used by SDLogger::init (mapped onto the SPI pins)
    int sd_clk   = 12;
    int sd_cmd   = 11;
    int sd_d0    = 10;

    // Recovery beacon / buzzer (active high)
    int beacon   = 42;

    // Battery voltage divider (ADC1_CH0), when fitted
    int bat_adc  = 1;
};
static constexpr PinConfig PINS{};

// ===========================================================================
// Mission profile: carrier drone/rocket -> release at apogee -> passive chute
// -> arms unlatched at 600 m -> motors vector thrust to steer the canopy back
// to the launch site -> motors off near the ground -> landed.
// ===========================================================================
struct FlightProfileConfig {
    // Launch (PAD -> ASCENT). Either path latches flight:
    //   rocket : |f| >= launch_accel_mps2 for launch_accel_s, then confirmed by an
    //            altitude gain >= launch_confirm_gain_m within launch_confirm_s
    //   carrier: altitude gain >= launch_alt_gain_m while climbing >= launch_climb_mps
    double launch_accel_mps2       = 29.4;  ///< 3 g (BNO055 clips at 4 g in fusion mode)
    double launch_accel_s          = 0.05;
    double launch_confirm_gain_m   = 10.0;
    double launch_confirm_s        = 4.0;
    double launch_alt_gain_m       = 25.0;
    double launch_climb_mps        = 1.0;
    double launch_climb_s          = 2.0;

    // Release from the carrier (ASCENT -> DESCENT)
    double release_drop_m          = 5.0;   ///< At least this far below the peak ...
    double release_descent_mps     = 2.0;   ///< ... sinking at least this fast ...
    double release_descent_s       = 1.0;   ///< ... for this long, plus release evidence:
    double freefall_mps2           = 3.5;   ///< |f| below this == free fall (drop / ejection)
    double freefall_s              = 0.12;
    double shock_mps2              = 29.4;  ///< |f| above this near apogee == ejection / chute snatch
    double release_evidence_s      = 8.0;   ///< Free fall / shock must be this recent
    double release_fallback_mps    = 3.0;   ///< No evidence: need this descent rate ...
    double release_fallback_s      = 3.0;   ///< ... for this long (a carrier sinking slowly never qualifies)

    // Arm unlatch (DESCENT -> ARMS_DEPLOY)
    double arms_deploy_alt_m       = 600.0; ///< AGL, on descent
    double arms_min_after_release_s= 2.0;   ///< Clear of the carrier before unlatching
    double arms_open_time_s        = 1.5;   ///< Servo travel + arms swinging out

    // Steering enable (ARMS_DEPLOY -> STEERING)
    double steer_max_tilt_deg      = 45.0;  ///< Attitude must be sane to start motors
    double steer_max_rate_rps      = 4.0;
    double steer_arm_timeout_s     = 10.0;  ///< Still unsafe after this: stay passive on the chute
    double motor_cutoff_alt_m      = 10.0;  ///< Motors off below this AGL (people / props)

    // Failsafes while steering
    double tumble_tilt_deg         = 65.0;  ///< Motors off if tilted beyond this ...
    double tumble_s                = 0.3;   ///< ... for this long
    double recover_tilt_deg        = 30.0;  ///< Restart after tilt back below this ...
    double recover_s               = 1.0;   ///< ... for this long

    // Landed
    double landed_alt_m            = 5.0;
    double landed_speed_mps        = 0.6;
    double landed_s                = 2.0;
};
static constexpr FlightProfileConfig PROFILE_CFG{};

// ===========================================================================
// Vertical channel filter (altitude, vertical speed, accel bias), float
// ===========================================================================
struct VerticalConfig {
    // Process noise (accel input noise, m/s^2) per mission phase
    float sigma_a_pad      = 0.3f;
    float sigma_a_ascent   = 4.0f;
    float sigma_a_descent  = 2.0f;
    float sigma_a_steering = 3.0f;
    float sigma_a_clipped  = 60.0f;   ///< Accel saturated (true accel unknown, >= 4 g): baro leads
    float sigma_bias       = 0.02f;   ///< Accel bias random walk (m/s^2 / sqrt(s))

    // Baro measurement noise (m, 1 sigma); inflated under prop wash
    float baro_sigma_m          = 0.35f;
    float baro_sigma_steering_m = 1.2f;
    float baro_gate_chi2        = 13.8f;  ///< 1-dof, 99.98 %
    float baro_max_rate_mps     = 120.0f; ///< Faster than this is physically impossible
    float resync_after_s        = 0.5f;   ///< Rejected this long with smooth (non-erratic) data -> re-sync

    // GNSS altitude (weak aiding)
    float gnss_alt_sigma_m      = 6.0f;
    float gnss_gate_chi2        = 10.8f;
    int   gnss_min_sats         = 5;

    float accel_clip_mps2       = 37.0f;  ///< |any axis| above this == BNO055 4 g clipping
};
static constexpr VerticalConfig VERT_CFG{};

// ===========================================================================
// Return-to-launch guidance (thrust vectoring under the canopy)
// ===========================================================================
struct GuidanceConfig {
    float site_avg_s        = 10.0f;  ///< Average GNSS on the pad this long for the launch site
    int   site_min_sats     = 5;
    float k_pos             = 0.08f;  ///< Desired ground speed per metre of distance (1/s)
    float v_max_mps         = 5.0f;   ///< Max commanded ground speed toward the site
    float k_vel             = 0.6f;   ///< Accel command per m/s of velocity error (1/s)
    float k_vel_i           = 0.10f;  ///< Integral of velocity error (learns the wind) (1/s^2)
    float a_max_mps2        = 4.0f;
    float max_tilt_deg      = 25.0f;
    float arrive_radius_m   = 8.0f;   ///< Inside this: no tilt command
    float gnss_timeout_s    = 3.0f;   ///< No fresh fix: level the vehicle
};
static constexpr GuidanceConfig GUIDE_CFG{};

// ===========================================================================
// Actuators
// ===========================================================================
struct ActuatorConfig {
    // Linear servos (arm latches)
    uint32_t servo_lock_us      = 1000;
    uint32_t servo_unlatch_us   = 2000;

    // DShot300 motor output (BLHeli_S). Normalised throttle 0..1 -> DShot 48..2047
    float    idle_throttle      = 0.06f;  ///< Spinning, negligible thrust
    float    base_throttle      = 0.35f;  ///< Collective while steering
    float    max_throttle       = 0.80f;
    float    spinup_s           = 1.5f;   ///< Idle -> base ramp
    float    arm_zero_s         = 0.6f;   ///< DShot 0-throttle frames before spinning (ESC arming)

    // Attitude cascade (angle -> rate -> differential throttle)
    float kp_angle     = 4.0f;    ///< rad/s per rad
    float ki_angle     = 1.5f;    ///< rad/s per rad*s (removes the canopy pendulum's steady tilt error)
    float kp_rate      = 0.12f;   ///< throttle per rad/s
    float ki_rate      = 0.05f;
    float kd_rate      = 0.002f;
    float kp_yaw_rate  = 0.08f;   ///< Yaw-rate damping (stops canopy-induced spin)
    float max_rate_rps = 2.5f;
    float max_torque   = 0.25f;   ///< Differential throttle authority
};
static constexpr ActuatorConfig ACT_CFG{};

// ===========================================================================
// Telemetry configuration  (CAN-7USAT format compliance)
// ===========================================================================

struct TelemetryConfig {
    uint16_t team_id             = 1;        ///< Team 001 (printed zero-padded as "001")
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

/// TEAM_ID field of the telemetry frame (guidelines: 2026-IN-SPACeCAN-7USAT-XXX)
inline constexpr const char TEAM_ID_STR[] = "2026-IN-SPACeCAN-7USAT-001";

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
