/**
 * @file dshot.cpp
 * @brief DShot300 on RMT: pre-computed symbols + copy encoder, one frame per write().
 */
#include "drivers/dshot.hpp"
#include "esp_log.h"
#include <algorithm>
#include <cmath>

static const char* TAG = "DShot";

namespace drivers {

namespace {
constexpr uint32_t RES_HZ    = 40000000;              // 25 ns ticks
constexpr uint32_t BIT_TICKS = RES_HZ / 300000;       // 133 ticks = 3.33 us (DShot300)
constexpr uint32_t T1H       = BIT_TICKS * 3 / 4;     // 75 %
constexpr uint32_t T0H       = BIT_TICKS * 3 / 8;     // 37.5 %
}

esp_err_t DShot::init(const int gpio[N]) noexcept {
    rmt_copy_encoder_config_t ecfg{};
    esp_err_t err = rmt_new_copy_encoder(&ecfg, &enc_);
    if (err != ESP_OK) { ESP_LOGE(TAG, "encoder: %s", esp_err_to_name(err)); return err; }

    for (int i = 0; i < N; ++i) {
        rmt_tx_channel_config_t cfg{};
        cfg.clk_src           = RMT_CLK_SRC_DEFAULT;
        cfg.gpio_num          = (gpio_num_t)gpio[i];
        cfg.mem_block_symbols = 48;      // S3: 4 TX channels x 48 symbols fit the RMT RAM
        cfg.resolution_hz     = RES_HZ;
        cfg.trans_queue_depth = 4;
        err = rmt_new_tx_channel(&cfg, &ch_[i]);
        if (err == ESP_OK) err = rmt_enable(ch_[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "channel %d (GPIO %d): %s", i, gpio[i], esp_err_to_name(err));
            return err;
        }
    }
    ready_ = true;
    ESP_LOGI(TAG, "DShot300 ready on GPIO %d/%d/%d/%d (no signal until armed)",
             gpio[0], gpio[1], gpio[2], gpio[3]);
    return ESP_OK;
}

uint16_t DShot::from_throttle(float t) noexcept {
    if (!(t > 0.0f)) return 0;
    t = std::min(t, 1.0f);
    return (uint16_t)(48 + std::lround(t * 1999.0f));
}

void DShot::write(const uint16_t value[N]) noexcept {
    if (!ready_) return;
    flip_ ^= 1;
    rmt_transmit_config_t tx{};
    tx.loop_count = 0;
    for (int i = 0; i < N; ++i) {
        uint16_t v = value[i];
        if (v > 2047) v = 2047;
        if (v > 0 && v < 48) v = 0;                  // commands are not used in flight
        const uint16_t data = (uint16_t)(v << 1);    // telemetry bit = 0
        const uint16_t crc  = (data ^ (data >> 4) ^ (data >> 8)) & 0x0F;
        const uint16_t pkt  = (uint16_t)((data << 4) | crc);
        rmt_symbol_word_t* s = buf_[i][flip_];
        for (int b = 0; b < 16; ++b) {
            const bool one = (pkt >> (15 - b)) & 1;
            s[b].level0 = 1; s[b].duration0 = one ? T1H : T0H;
            s[b].level1 = 0; s[b].duration1 = BIT_TICKS - (one ? T1H : T0H);
        }
        rmt_transmit(ch_[i], enc_, s, sizeof(buf_[i][0]), &tx);
    }
}

} // namespace drivers
