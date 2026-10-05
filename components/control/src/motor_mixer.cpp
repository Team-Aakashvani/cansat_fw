#include "control/motor_mixer.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>
static const char* TAG = "MotorMixer";

namespace control {

esp_err_t MotorMixer::init() noexcept {
    const nav::PinConfig& P = nav::PINS;

    // Timer 0: High-frequency DC Motor PWM (5kHz, 10-bit)
    ledc_timer_config_t t_motor{};
    t_motor.speed_mode       = LEDC_LOW_SPEED_MODE;
    t_motor.duty_resolution  = (ledc_timer_bit_t)MOTOR_RES_BITS;
    t_motor.timer_num        = LEDC_TIMER_0;
    t_motor.freq_hz          = MOTOR_PWM_FREQ_HZ;
    t_motor.clk_cfg          = LEDC_AUTO_CLK;
    esp_err_t ret = ledc_timer_config(&t_motor);
    if (ret != ESP_OK) return ret;

    // Timer 1: Standard RC Servo (50Hz, 16-bit)
    ledc_timer_config_t t_servo{};
    t_servo.speed_mode       = LEDC_LOW_SPEED_MODE;
    t_servo.duty_resolution  = (ledc_timer_bit_t)SERVO_RES_BITS;
    t_servo.timer_num        = LEDC_TIMER_1;
    t_servo.freq_hz          = SERVO_FREQ_HZ;
    t_servo.clk_cfg          = LEDC_AUTO_CLK;
    ret = ledc_timer_config(&t_servo);
    if (ret != ESP_OK) return ret;

    // Motor channels (Timer 0, 0 initial duty = completely OFF)
    for (int i = 0; i < N_MOTORS; ++i) {
        ledc_channel_config_t ch{};
        ch.gpio_num   = P.motor[i];
        ch.speed_mode = LEDC_LOW_SPEED_MODE;
        ch.channel    = (ledc_channel_t)i;
        ch.timer_sel  = LEDC_TIMER_0;
        ch.duty       = 0;  // 0V constant (safe and still)
        ch.hpoint     = 0;
        ch.intr_type  = LEDC_INTR_DISABLE;
        ret = ledc_channel_config(&ch);
        if (ret != ESP_OK) return ret;
        motor_us_[i] = nav::CONTROL_CFG.motor_min_pwm_us;
    }

    // Servo channel (Timer 1, 50Hz, neutral 1500us)
    ledc_channel_config_t srv{};
    srv.gpio_num   = P.servo;
    srv.speed_mode = LEDC_LOW_SPEED_MODE;
    srv.channel    = (ledc_channel_t)SERVO_CH;
    srv.timer_sel  = LEDC_TIMER_1;
    srv.duty       = (uint32_t)(((uint64_t)1500 * SERVO_MAX_DUTY) / 20000UL);
    srv.hpoint     = 0;
    ret = ledc_channel_config(&srv);

    ESP_LOGI(TAG, "MotorMixer initialised (5kHz DC Motor PWM + 50Hz Servo)");
    return ret;
}

void MotorMixer::arm() noexcept {
    if (armed_) return;
    ESP_LOGI(TAG, "Arming motors...");
    for (int i = 0; i < N_MOTORS; ++i) set_motor_us(i, nav::CONTROL_CFG.motor_arm_pwm_us);
    vTaskDelay(pdMS_TO_TICKS(1000));
    for (int i = 0; i < N_MOTORS; ++i) set_motor_us(i, nav::CONTROL_CFG.motor_idle_pwm_us);
    armed_ = true;
    ESP_LOGI(TAG, "Motors armed");
}

void MotorMixer::disarm() noexcept {
    for (int i = 0; i < N_MOTORS; ++i) set_motor_us(i, nav::CONTROL_CFG.motor_min_pwm_us);
    armed_ = false;
}

void MotorMixer::set_motor_us(int idx, uint32_t us) noexcept {
    if (idx < 0 || idx >= N_MOTORS) return;
    const uint32_t min_us = nav::CONTROL_CFG.motor_min_pwm_us;
    const uint32_t max_us = nav::CONTROL_CFG.motor_max_pwm_us;
    us = std::clamp(us, min_us, max_us);
    motor_us_[idx] = us;
    apply_motor(idx);
}

void MotorMixer::mix_and_set(double thr, double pitch, double roll, double yaw,
                              double bat_factor) noexcept {
    if (!armed_) {
        for (int i = 0; i < N_MOTORS; ++i) apply_motor(i);
        return;
    }
    const double scale = std::clamp(bat_factor, 0.1, 1.0);
    const double thr_clamped = std::clamp(thr * scale, 0.0, 1.0);
    const double thr_us = nav::CONTROL_CFG.motor_idle_pwm_us
        + thr_clamped * (nav::CONTROL_CFG.motor_max_pwm_us
                         - nav::CONTROL_CFG.motor_idle_pwm_us);

    // Mix (+ config, cross)
    const double range = (nav::CONTROL_CFG.motor_max_pwm_us - nav::CONTROL_CFG.motor_idle_pwm_us) * 0.25;
    double m[4];
    m[0] = thr_us + range*(pitch + roll + yaw);
    m[1] = thr_us + range*(pitch - roll - yaw);
    m[2] = thr_us + range*(-pitch - roll + yaw);
    m[3] = thr_us + range*(-pitch + roll - yaw);

    for (int i = 0; i < N_MOTORS; ++i)
        set_motor_us(i, (uint32_t)std::clamp(m[i],
            (double)nav::CONTROL_CFG.motor_min_pwm_us,
            (double)nav::CONTROL_CFG.motor_max_pwm_us));
}

void MotorMixer::servo_release() noexcept {
    uint32_t duty = (uint32_t)(((uint64_t)2000 * SERVO_MAX_DUTY) / 20000UL);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)SERVO_CH, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)SERVO_CH);
}

void MotorMixer::servo_home() noexcept {
    uint32_t duty = (uint32_t)(((uint64_t)1000 * SERVO_MAX_DUTY) / 20000UL);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)SERVO_CH, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)SERVO_CH);
}

void MotorMixer::set_servo_angle(double degrees) noexcept {
    degrees = std::clamp(degrees, 0.0, 180.0);
    uint32_t us = 1000 + (uint32_t)(degrees * 1000.0 / 180.0);
    uint32_t duty = (uint32_t)(((uint64_t)us * SERVO_MAX_DUTY) / 20000UL);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)SERVO_CH, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)SERVO_CH);
}

uint32_t MotorMixer::us_to_duty(uint32_t us) const noexcept {
    if (us <= nav::CONTROL_CFG.motor_min_pwm_us) return 0;
    uint32_t span = nav::CONTROL_CFG.motor_max_pwm_us - nav::CONTROL_CFG.motor_min_pwm_us;
    if (span == 0) return 0;
    uint32_t rel = us - nav::CONTROL_CFG.motor_min_pwm_us;
    return (uint32_t)(((uint64_t)rel * MOTOR_MAX_DUTY) / span);
}

void MotorMixer::apply_motor(int idx) noexcept {
    uint32_t duty = armed_ ? us_to_duty(motor_us_[idx]) : 0;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)idx, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)idx);
}

} // namespace control
