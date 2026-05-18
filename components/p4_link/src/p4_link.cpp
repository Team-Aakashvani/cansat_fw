#include "p4_link/p4_link.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>

static const char* TAG = "P4Link";

namespace p4_link {

void P4Link::init(hal::UARTBus& uart) noexcept {
    uart_ = &uart;
    status_.connected = false;
    status_.last_heartbeat_ms = 0;
}

void P4Link::send_command(const char* cmd) noexcept {
    if (!uart_) return;

    char data_buf[64];
    int data_len = snprintf(data_buf, sizeof(data_buf), "P4CMD,%s", cmd);
    
    if (data_len > 0) {
        uint8_t crc = calc_checksum(data_buf, data_len);
        char full_buf[128];
        int full_len = snprintf(full_buf, sizeof(full_buf), "$%s*%02X\r\n", data_buf, crc);
        uart_->write((uint8_t*)full_buf, (size_t)full_len);
        ESP_LOGI(TAG, "Sent: %s", full_buf);
    }
}

void P4Link::spin() noexcept {
    if (!uart_) return;

    char line[128];
    while (uart_->read_line(line, sizeof(line), 0) > 0) {
        parse_heartbeat(line);
    }

    // Timeout detection (3 seconds)
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (status_.connected && (now - status_.last_heartbeat_ms > 3000)) {
        ESP_LOGW(TAG, "P4 connection lost (timeout)");
        status_.connected = false;
    }
}

P4Status P4Link::get_status() const noexcept {
    return status_;
}

uint8_t P4Link::calc_checksum(const char* s, size_t len) noexcept {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) crc ^= (uint8_t)s[i];
    return crc;
}

void P4Link::parse_heartbeat(const char* line) noexcept {
    // Expected: "$P4HB,rec,free,fps*XX"
    if (line[0] != '$') return;

    char tmp[128];
    strncpy(tmp, line, sizeof(tmp));
    char* star = strchr(tmp, '*');
    if (!star) return;

    *star = '\0';
    uint8_t received_crc = (uint8_t)strtol(star + 1, nullptr, 16);
    uint8_t actual_crc = calc_checksum(tmp + 1, strlen(tmp + 1));

    if (received_crc != actual_crc) {
        ESP_LOGW(TAG, "CRC error on P4HB: expected %02X, got %02X", actual_crc, received_crc);
        return;
    }

    if (strncmp(tmp + 1, "P4HB,", 5) == 0) {
        unsigned rec = 0, fps = 0;
        float free_gb = 0.0f;
        if (sscanf(tmp + 6, "%u,%f,%u", &rec, &free_gb, &fps) == 3) {
            status_.connected = true;
            status_.recording = (rec != 0);
            status_.sd_free_gb = free_gb;
            status_.fps = (uint8_t)fps;
            status_.last_heartbeat_ms = (uint32_t)(esp_timer_get_time() / 1000);
        }
    }
}

} // namespace p4_link
