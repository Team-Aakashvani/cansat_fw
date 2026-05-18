/**
 * @file event_log.hpp
 * @brief Discrete flight event logger backed by NVS partition.
 *
 * Records irreversible flight events (state transitions, FDIR faults, BIT
 * results, deploy triggers) to a dedicated "event_log" NVS partition.
 * Events persist across warm resets and are readable post-flight via USB.
 *
 * Storage: each event is a fixed 64-byte record. The partition holds up to
 * MAX_EVENTS = (partition_size / 64). When full, oldest events are overwritten.
 *
 * Thread safety: log_event() uses an internal mutex. Safe to call from any task.
 */
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include <cstdint>

namespace logging {

enum class EventCode : uint8_t {
    BOOT            = 0x01,
    BIT_PASS        = 0x02,
    BIT_FAIL        = 0x03,
    LAUNCH_DETECT   = 0x10,
    BOOST_START     = 0x11,
    APOGEE          = 0x12,
    PARACHUTE_DEPLOY= 0x13,
    DRONE_DEPLOY    = 0x14,
    LANDED          = 0x15,
    FDIR_QUARANTINE = 0x20,
    FDIR_CLEAR      = 0x21,
    CMD_CX_ON       = 0x30,
    CMD_CX_OFF      = 0x31,
    CMD_ST          = 0x32,
    CMD_CAL         = 0x33,
    POWER_LOW       = 0x40,
    WATCHDOG_RESET  = 0x50,
    CUSTOM          = 0xFF,
};

struct EventRecord {
    uint32_t  timestamp_ms;   ///< esp_timer_get_time() / 1000
    uint32_t  mission_time_s;
    EventCode code;
    uint8_t   data[3];        ///< Event-specific payload (e.g. state code)
    char      msg[54];        ///< Null-terminated human-readable note
} __attribute__((packed));

static_assert(sizeof(EventRecord) == 64, "EventRecord must be 64 bytes");

class EventLog {
public:
    static constexpr size_t MAX_EVENTS = 256;  ///< Ring capacity

    EventLog() noexcept = default;
    ~EventLog() noexcept { close(); }

    /**
     * @brief Initialise NVS-backed event log.
     * Creates NVS namespace "evt_log" in the default NVS partition.
     * @return ESP_OK on success.
     */
    esp_err_t init() noexcept;

    /**
     * @brief Log a flight event.
     * Thread-safe. Non-blocking (uses a try-lock; drops if contested).
     * @param code         Event category.
     * @param mission_s    Current mission time in seconds.
     * @param data0..2     Optional byte payload.
     * @param msg          Optional human-readable message (truncated to 53 chars).
     */
    void log_event(EventCode code, uint32_t mission_s,
                   uint8_t data0 = 0, uint8_t data1 = 0, uint8_t data2 = 0,
                   const char* msg = "") noexcept;

    /**
     * @brief Read back all stored events (most recent first).
     * @param out   Caller-supplied buffer.
     * @param max   Maximum entries to read.
     * @return Number of events written to @p out.
     */
    size_t read_events(EventRecord* out, size_t max) const noexcept;

    /// Total events logged since init (may exceed MAX_EVENTS due to ring wrap).
    uint32_t total_events() const noexcept { return total_events_; }

    void close() noexcept;

private:
    bool              ready_        = false;
    uint32_t          write_idx_    = 0;   ///< NVS key counter
    uint32_t          total_events_ = 0;
    SemaphoreHandle_t mutex_        = nullptr;

    void store_record(const EventRecord& r) noexcept;
};

} // namespace logging
