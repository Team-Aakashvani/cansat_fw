/**
 * @file flight_recorder.hpp
 * @brief Flight data recorder in internal flash ("flightlog" partition, 12 MB ring).
 *
 * Fixed 64-byte records (CRC16), written sequentially as a ring:
 *   0xF0 session header  - one per boot (boot count, baro reference, build)
 *   0x01 nav record      - 50 Hz: phase, AGL, vertical speed, attitude quaternion,
 *                          accel, gyro, GNSS, actuator state, guidance command
 *   0x02 event           - mission events as text
 * A global sequence number orders records across the wrap.
 *
 * Flash timing: a page program stalls both cores for well under a millisecond, but
 * a 4 KB sector erase stalls them for ~45 ms. So sectors are erased AHEAD of the write
 * pointer in the background while on the pad (keeping >= 3 MB = ~15 min pre-erased),
 * never while flying unless the margin runs out. Power on a few minutes before launch.
 *
 * Producers (nav task, events) only enqueue; a low-priority task on core 1 writes.
 * Worst-case loss on sudden power-off: the last ~1 s of buffered records.
 */
#pragma once

#include "esp_err.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <atomic>
#include <cstdint>
#include <cstddef>

namespace logging {

struct __attribute__((packed)) NavRecord {
    uint16_t magic;        ///< 0xA55A
    uint8_t  type;         ///< 0x01
    uint8_t  phase;
    uint32_t seq;
    uint32_t t_ms;         ///< MCU time since boot
    float    agl_m;
    float    vz_mps;
    float    baro_alt_m;   ///< raw baro altitude (filter input)
    int16_t  q[4];         ///< vehicle quaternion x 30000 (w, x, y, z)
    int16_t  acc[3];       ///< vehicle-frame specific force x 100 (m/s^2)
    int16_t  gyr[3];       ///< vehicle-frame rate x 1000 (rad/s)
    int32_t  lat_e7;
    int32_t  lon_e7;
    int16_t  gnss_alt_dm;  ///< GNSS MSL altitude, decimetres
    uint8_t  sats;
    uint8_t  flags;        ///< b0 arms, b1 baro held, b2 accel clipped, b3 motors suspended, b4-6 ESC state
    uint8_t  thr[4];       ///< motor throttle x 255
    int8_t   tilt_sp[2];   ///< guidance tilt command (deg)
    uint16_t crc;
};
static_assert(sizeof(NavRecord) == 64, "NavRecord must be 64 bytes");

class FlightRecorder {
public:
    static constexpr uint16_t MAGIC      = 0xA55A;
    static constexpr uint8_t  T_NAV      = 0x01;
    static constexpr uint8_t  T_EVENT    = 0x02;
    static constexpr uint8_t  T_SESSION  = 0xF0;
    static constexpr size_t   REC        = 64;
    static constexpr size_t   SECTOR     = 4096;

    struct Status {
        bool     ready = false;
        uint32_t capacity = 0;        ///< bytes
        uint32_t write_off = 0;       ///< bytes into the partition
        uint32_t erased_ahead = 0;    ///< bytes pre-erased ahead of the write pointer
        uint32_t session_bytes = 0;   ///< written this boot
        uint32_t dropped = 0;         ///< records lost (queue full)
        bool     erasing_all = false;
    };

    /// Locate the write pointer (scan), write the session header, start the writer task.
    esp_err_t init(uint32_t boot_count, float baro_p0_pa, bool resumed) noexcept;

    /// Non-blocking. seq / magic / crc are filled in here.
    void log_nav(NavRecord r) noexcept;
    void log_event(uint8_t phase, const char* text) noexcept;

    /// In flight: no background erase (sector erases stall both cores ~45 ms)
    void set_flight_active(bool active) noexcept { flight_active_.store(active); }

    Status status() const noexcept;

    /// Readout over the console (run from the CLI task; prints to stdout)
    void list_sessions() noexcept;
    void dump_session(int index) noexcept;   ///< index 1 = latest, 2 = previous, ...
    /// Erase the whole log in the background (pad only)
    void request_erase_all() noexcept { erase_all_req_.store(true); }

private:
    const esp_partition_t* part_ = nullptr;
    QueueHandle_t          q_ = nullptr;
    SemaphoreHandle_t      flash_mtx_ = nullptr;   ///< writer vs. readout
    uint32_t               nsec_ = 0;
    uint32_t               write_off_ = 0;
    uint32_t               margin_ = 0;            ///< pre-erased bytes from write_off_ onwards
    uint32_t               erase_next_ = 0;        ///< next sector to erase (= write_off_ + margin_)
    uint32_t               session_bytes_ = 0;
    std::atomic<uint32_t>  seq_{0};
    std::atomic<uint32_t>  dropped_{0};
    std::atomic<bool>      flight_active_{false};
    std::atomic<bool>      erase_all_req_{false};
    std::atomic<bool>      erasing_all_{false};
    bool                   ready_ = false;

    static void task_entry(void* arg);
    void task_loop() noexcept;
    void push(const uint8_t rec[REC]) noexcept;
    bool erase_one() noexcept;                      ///< erase the next sector ahead (caller holds flash_mtx_)
    uint32_t margin() const noexcept;
    static uint16_t crc16(const uint8_t* d, size_t n) noexcept;
    static bool valid(const uint8_t* rec) noexcept;

    template <typename F> void for_each_record(F&& fn) noexcept;   ///< oldest -> newest
};

} // namespace logging
