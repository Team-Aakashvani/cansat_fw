#include "control/motor_mixer.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>
static const char* TAG = "MotorMixer";

namespace control {

esp_err_t MotorMixer::init() noexcept {
    const nav::PinConfig& P = nav::PINS;
    // Timer for motors + servo (50Hz)
    ledc_timer_config_t t{};
    t.speed_mode       = LEDC_LOW_SPEED_MODE;
    t.duty_resolution  = (ledc_timer_bit_t)LEDC_RES_BITS;
    t.timer_num        = LEDC_TIMER_0;
    t.freq_hz          = LEDC_FREQ_HZ;
    t.clk_cfg          = LEDC_AUTO_CLK;
    esp_err_t ret = ledc_timer_config(&t);
    if (ret != ESP_OK) return ret;

    // Motor channels
    for (int i = 0; i < N_MOTORS; ++i) {
        ledc_channel_config_t ch{};
        ch.gpio_num   = P.motor[i];
        ch.speed_mode = LEDC_LOW_SPEED_MODE;
        ch.channel    = (ledc_channel_t)i;
        ch.timer_sel  = LEDC_TIMER_0;
        ch.duty       = us_to_duty(nav::CONTROL_CFG.motor_min_pwm_us);
        ch.hpoint     = 0;
        ch.intr_type  = LEDC_INTR_DISABLE;
        ret = ledc_channel_config(&ch);
        if (ret != ESP_OK) return ret;
        motor_us_[i] = nav::CONTROL_CFG.motor_min_pwm_us;
    }

    // Servo channel
    ledc_channel_config_t srv{};
    srv.gpio_num   = P.servo;
    srv.speed_mode = LEDC_LOW_SPEED_MODE;
    srv.channel    = (ledc_channel_t)SERVO_CH;
    srv.timer_sel  = LEDC_TIMER_0;
    srv.duty       = us_to_duty(1500);  // neutral
    srv.hpoint     = 0;
    ret = ledc_channel_config(&srv);

    ESP_LOGI(TAG, "MotorMixer initialised");
    return ret;
}

void MotorMixer::arm() noexcept {
    ESP_LOGI(TAG, "Arming motors...");
    for (int i = 0; i < N_MOTORS; ++i) set_motor_us(i, nav::CONTROL_CFG.motor_arm_pwm_us);
    vTaskDelay(pdMS_TO_TICKS(2000));
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
    if (!armed_) return;
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
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)SERVO_CH, us_to_duty(2000));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)SERVO_CH);
}

void MotorMixer::servo_home() noexcept {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)SERVO_CH, us_to_duty(1000));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)SERVO_CH);
}

void MotorMixer::set_servo_angle(double degrees) noexcept {
    degrees = std::clamp(degrees, 0.0, 180.0);
    // Linear map: 0° -> 1000µs, 180° -> 2000µs
    uint32_t us = 1000 + (uint32_t)(degrees * 1000.0 / 180.0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)SERVO_CH, us_to_duty(us));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)SERVO_CH);
}

uint32_t MotorMixer::us_to_duty(uint32_t us) const noexcept {
    // Period = 20ms (50Hz). Duty = us/20000 * max_duty
    return (uint32_t)(((uint64_t)us * LEDC_MAX_DUTY) / 20000UL);
}

void MotorMixer::apply_motor(int idx) noexcept {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)idx, us_to_duty(motor_us_[idx]));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)idx);
}

} // namespace control
