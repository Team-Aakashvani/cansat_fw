/**
 * @file servo.cpp
 * @brief LEDC 50 Hz / 14-bit servo pair.
 */
#include "drivers/servo.hpp"
#include "driver/ledc.h"
#include "esp_log.h"
#include <algorithm>

static const char* TAG = "Servo";

namespace drivers {

namespace {
constexpr ledc_timer_t     TIMER = LEDC_TIMER_2;
constexpr ledc_channel_t   CH_A  = LEDC_CHANNEL_6;
constexpr ledc_channel_t   CH_B  = LEDC_CHANNEL_7;
constexpr uint32_t         FREQ  = 50;
constexpr ledc_timer_bit_t RES   = LEDC_TIMER_14_BIT;
constexpr uint32_t         MAXD  = (1u << 14) - 1;
constexpr uint32_t         PERIOD_US = 1000000 / FREQ;

uint32_t us_to_duty(uint32_t us) { return (uint32_t)(((uint64_t)us * MAXD) / PERIOD_US); }
}

esp_err_t ServoPair::init(int gpio_a, int gpio_b, uint32_t initial_us) noexcept {
    ledc_timer_config_t t{};
    t.speed_mode      = LEDC_LOW_SPEED_MODE;
    t.duty_resolution = RES;
    t.timer_num       = TIMER;
    t.freq_hz         = FREQ;
    t.clk_cfg         = LEDC_AUTO_CLK;
    esp_err_t err = ledc_timer_config(&t);
    if (err != ESP_OK) { ESP_LOGE(TAG, "timer: %s", esp_err_to_name(err)); return err; }

    initial_us = std::clamp<uint32_t>(initial_us, 500, 2500);
    const int pins[2] = { gpio_a, gpio_b };
    const ledc_channel_t chs[2] = { CH_A, CH_B };
    for (int i = 0; i < 2; ++i) {
        ledc_channel_config_t c{};
        c.gpio_num   = pins[i];
        c.speed_mode = LEDC_LOW_SPEED_MODE;
        c.channel    = chs[i];
        c.timer_sel  = TIMER;
        c.duty       = us_to_duty(initial_us);
        c.hpoint     = 0;
        c.intr_type  = LEDC_INTR_DISABLE;
        err = ledc_channel_config(&c);
        if (err != ESP_OK) { ESP_LOGE(TAG, "channel GPIO %d: %s", pins[i], esp_err_to_name(err)); return err; }
    }
    us_ = initial_us;
    ready_ = true;
    ESP_LOGI(TAG, "Arm-latch servos on GPIO %d/%d at %lu us", gpio_a, gpio_b, (unsigned long)initial_us);
    return ESP_OK;
}

void ServoPair::set_us(uint32_t us) noexcept {
    if (!ready_) return;
    us = std::clamp<uint32_t>(us, 500, 2500);
    if (us == us_) return;
    const uint32_t d = us_to_duty(us);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_A, d); ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_A);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_B, d); ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_B);
    us_ = us;
}

} // namespace drivers
