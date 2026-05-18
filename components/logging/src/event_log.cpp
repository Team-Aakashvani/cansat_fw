/**
 * @file event_log.cpp
 * @brief NVS-backed flight event log implementation.
 */
#include "logging/event_log.hpp"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <cstring>
#include <cstdio>

static const char* TAG = "EventLog";
static const char* NVS_NS = "evt_log";

namespace logging {

esp_err_t EventLog::init() noexcept {
    mutex_ = xSemaphoreCreateMutex();
    if (!mutex_) return ESP_ERR_NO_MEM;

    // Read current write_idx_ from NVS (persists across reboots)
    nvs_handle_t nvs;
    esp_err_t ret = nvs_open(NVS_NS, NVS_READWRITE, &nvs);
    if (ret != ESP_OK) {
        // First boot or NVS not initialised
        ESP_LOGW(TAG, "NVS open failed (%d) — event log reset", ret);
        write_idx_ = 0;
    } else {
        nvs_get_u32(nvs, "widx", &write_idx_);
        nvs_get_u32(nvs, "total", &total_events_);
        nvs_close(nvs);
    }
    ready_ = true;
    ESP_LOGI(TAG, "EventLog ready, widx=%lu total=%lu",
             (unsigned long)write_idx_, (unsigned long)total_events_);
    return ESP_OK;
}

void EventLog::log_event(EventCode code, uint32_t mission_s,
                          uint8_t d0, uint8_t d1, uint8_t d2,
                          const char* msg) noexcept {
    if (!ready_) return;
    if (!xSemaphoreTake(mutex_, pdMS_TO_TICKS(10))) return;  // non-blocking-ish

    EventRecord r{};
    r.timestamp_ms   = (uint32_t)(esp_timer_get_time() / 1000ULL);
    r.mission_time_s = mission_s;
    r.code           = code;
    r.data[0]        = d0;
    r.data[1]        = d1;
    r.data[2]        = d2;
    if (msg) {
        strncpy(r.msg, msg, sizeof(r.msg) - 1);
        r.msg[sizeof(r.msg) - 1] = '\0';
    }
    store_record(r);
    ++total_events_;
    write_idx_ = (write_idx_ + 1) % MAX_EVENTS;

    // Persist indices periodically (every 16 events to reduce NVS wear)
    if ((total_events_ & 0xF) == 0) {
        nvs_handle_t nvs;
        if (nvs_open(NVS_NS, NVS_READWRITE, &nvs) == ESP_OK) {
            nvs_set_u32(nvs, "widx",  write_idx_);
            nvs_set_u32(nvs, "total", total_events_);
            nvs_commit(nvs);
            nvs_close(nvs);
        }
    }

    ESP_LOGD(TAG, "Event 0x%02X @ %lums mission=%lus '%s'",
             (unsigned)code,
             (unsigned long)r.timestamp_ms,
             (unsigned long)mission_s,
             r.msg);

    xSemaphoreGive(mutex_);
}

size_t EventLog::read_events(EventRecord* out, size_t max) const noexcept {
    if (!ready_ || !out || max == 0) return 0;
    nvs_handle_t nvs;
    if (nvs_open(NVS_NS, NVS_READONLY, &nvs) != ESP_OK) return 0;

    size_t count = 0;
    uint32_t total = total_events_;
    uint32_t start = (total > MAX_EVENTS) ? (write_idx_) : 0;
    size_t   avail = (total > MAX_EVENTS) ? MAX_EVENTS : total;
    size_t   n     = (avail < max) ? avail : max;

    for (size_t i = 0; i < n; ++i) {
        // Read most recent first
        uint32_t idx = (start + (uint32_t)(avail - 1 - i)) % MAX_EVENTS;
        char key[16];
        snprintf(key, sizeof(key), "e%04lu", (unsigned long)idx);
        size_t sz = sizeof(EventRecord);
        if (nvs_get_blob(nvs, key, &out[count], &sz) == ESP_OK
            && sz == sizeof(EventRecord)) {
            ++count;
        }
    }
    nvs_close(nvs);
    return count;
}

void EventLog::close() noexcept {
    if (!ready_) return;
    nvs_handle_t nvs;
    if (nvs_open(NVS_NS, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_u32(nvs, "widx",  write_idx_);
        nvs_set_u32(nvs, "total", total_events_);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    ready_ = false;
}

void EventLog::store_record(const EventRecord& r) noexcept {
    char key[16];
    snprintf(key, sizeof(key), "e%04lu", (unsigned long)(write_idx_ % MAX_EVENTS));
    nvs_handle_t nvs;
    if (nvs_open(NVS_NS, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_blob(nvs, key, &r, sizeof(r));
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

} // namespace logging
