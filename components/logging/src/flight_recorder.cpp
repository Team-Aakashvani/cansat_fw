/**
 * @file flight_recorder.cpp
 * @brief Flash ring-buffer flight data recorder.
 */
#include "logging/flight_recorder.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"
#include "esp_heap_caps.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <algorithm>

static const char* TAG = "FlightRec";

namespace logging {

namespace {
constexpr uint32_t MARGIN_TARGET = 3u * 1024u * 1024u;   // ~15 min at 50 Hz pre-erased
constexpr int      BATCH         = 4;                    // records per flash program
uint8_t            s_sector_buf[FlightRecorder::SECTOR]; // readout buffer (CLI context)

inline void put_u16(uint8_t* p, uint16_t v) { memcpy(p, &v, 2); }
inline void put_u32(uint8_t* p, uint32_t v) { memcpy(p, &v, 4); }
inline uint16_t get_u16(const uint8_t* p) { uint16_t v; memcpy(&v, p, 2); return v; }
inline uint32_t get_u32(const uint8_t* p) { uint32_t v; memcpy(&v, p, 4); return v; }
}

namespace {
struct Crc16Table {
    uint16_t t[256];
    constexpr Crc16Table() : t() {
        for (int i = 0; i < 256; ++i) {
            uint16_t c = (uint16_t)(i << 8);
            for (int b = 0; b < 8; ++b) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
            t[i] = c;
        }
    }
};
constexpr Crc16Table CRC16_TABLE{};
}

uint16_t FlightRecorder::crc16(const uint8_t* d, size_t n) noexcept {
    uint16_t c = 0xFFFF;                                   // CRC-16/CCITT-FALSE, table driven
    for (size_t i = 0; i < n; ++i) c = (uint16_t)((c << 8) ^ CRC16_TABLE.t[((c >> 8) ^ d[i]) & 0xFF]);
    return c;
}

bool FlightRecorder::valid(const uint8_t* rec) noexcept {
    return get_u16(rec) == MAGIC && get_u16(rec + 62) == crc16(rec, 62);
}

uint32_t FlightRecorder::margin() const noexcept { return margin_; }

esp_err_t FlightRecorder::init(uint32_t boot_count, float baro_p0_pa, bool resumed) noexcept {
    part_ = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, (esp_partition_subtype_t)0x40, "flightlog");
    if (!part_) { ESP_LOGE(TAG, "no 'flightlog' partition (flash the 16 MB partition table)"); return ESP_ERR_NOT_FOUND; }
    nsec_ = part_->size / SECTOR;
    const uint32_t cap = nsec_ * SECTOR;

    // 1. First record of every sector: erased / valid (seq) / garbage
    uint8_t* state = (uint8_t*)calloc(nsec_, 1);           // 0 erased, 1 valid, 2 garbage
    if (!state) return ESP_ERR_NO_MEM;
    int best = -1; uint32_t best_seq = 0;
    uint8_t rec[REC];
    for (uint32_t s = 0; s < nsec_; ++s) {
        esp_partition_read(part_, s * SECTOR, rec, REC);
        if (get_u16(rec) == 0xFFFF && rec[2] == 0xFF) { state[s] = 0; continue; }
        if (valid(rec)) {
            state[s] = 1;
            const uint32_t sq = get_u32(rec + 4);
            if (best < 0 || (int32_t)(sq - best_seq) > 0) { best = (int)s; best_seq = sq; }
        } else state[s] = 2;
    }

    // 2. Write pointer: first free slot after the newest record
    if (best < 0) {
        write_off_ = 0;
    } else {
        uint32_t k = 0;
        for (; k < SECTOR / REC; ++k) {
            esp_partition_read(part_, best * SECTOR + k * REC, rec, REC);
            if (get_u16(rec) == 0xFFFF && rec[2] == 0xFF) break;
            if (valid(rec)) best_seq = std::max(best_seq, get_u32(rec + 4));
        }
        write_off_ = (best * SECTOR + k * REC) % cap;
        seq_.store(best_seq + 1);
    }

    // 3. Pre-erased margin ahead of the write pointer
    uint32_t sec = write_off_ / SECTOR;
    margin_ = 0;
    if (write_off_ % SECTOR) { margin_ = SECTOR - write_off_ % SECTOR; sec = (sec + 1) % nsec_; }
    for (uint32_t n = 0; n < nsec_ - 1 && state[sec] == 0; ++n) { margin_ += SECTOR; sec = (sec + 1) % nsec_; }
    erase_next_ = sec * SECTOR;
    free(state);

    // Queue storage (16 KB) in PSRAM when available: frees internal RAM for the flight tasks
    q_ = xQueueCreateWithCaps(256, REC, MALLOC_CAP_SPIRAM);
    if (!q_) q_ = xQueueCreate(256, REC);
    flash_mtx_ = xSemaphoreCreateMutex();
    if (!q_ || !flash_mtx_) return ESP_ERR_NO_MEM;
    ready_ = true;

    // Session header
    uint8_t h[REC]; memset(h, 0, REC);
    put_u16(h, MAGIC); h[2] = T_SESSION; h[3] = resumed ? 1 : 0;
    put_u32(h + 8, (uint32_t)(esp_timer_get_time() / 1000));
    put_u32(h + 12, boot_count);
    memcpy(h + 16, &baro_p0_pa, 4);
    snprintf((char*)h + 20, 42, "%s %s", __DATE__, __TIME__);
    push(h);

    xTaskCreatePinnedToCore(task_entry, "flightrec", 4096, this, 2, nullptr, 1);
    ESP_LOGI(TAG, "Flight log: %lu KB, write @%lu KB, %lu KB pre-erased, next seq %lu%s",
             (unsigned long)(cap / 1024), (unsigned long)(write_off_ / 1024),
             (unsigned long)(margin_ / 1024), (unsigned long)seq_.load(), resumed ? " (resumed)" : "");
    return ESP_OK;
}

void FlightRecorder::push(const uint8_t rec_in[REC]) noexcept {
    if (!ready_) return;
    uint8_t rec[REC];
    memcpy(rec, rec_in, REC);
    put_u16(rec, MAGIC);
    put_u32(rec + 4, seq_.fetch_add(1));
    put_u16(rec + 62, crc16(rec, 62));
    if (xQueueSend(q_, rec, 0) != pdTRUE) dropped_.fetch_add(1);
}

void FlightRecorder::log_nav(NavRecord r) noexcept {
    r.type = T_NAV;
    push(reinterpret_cast<const uint8_t*>(&r));
}

void FlightRecorder::log_event(uint8_t phase, const char* text) noexcept {
    uint8_t e[REC]; memset(e, 0, REC);
    e[2] = T_EVENT; e[3] = phase;
    put_u32(e + 8, (uint32_t)(esp_timer_get_time() / 1000));
    strncpy((char*)e + 12, text ? text : "", 49);
    push(e);
}

FlightRecorder::Status FlightRecorder::status() const noexcept {
    Status s;
    s.ready = ready_;
    s.capacity = nsec_ * SECTOR;
    s.write_off = write_off_;
    s.erased_ahead = margin_;
    s.session_bytes = session_bytes_;
    s.dropped = dropped_.load();
    s.erasing_all = erasing_all_.load();
    return s;
}

bool FlightRecorder::erase_one() noexcept {
    const uint32_t cap = nsec_ * SECTOR;
    if (margin_ >= cap - SECTOR) return false;              // never erase the sector being filled
    if (esp_partition_erase_range(part_, erase_next_, SECTOR) != ESP_OK) return false;
    erase_next_ = (erase_next_ + SECTOR) % cap;
    margin_ += SECTOR;
    return true;
}

void FlightRecorder::task_entry(void* arg) { static_cast<FlightRecorder*>(arg)->task_loop(); }

void FlightRecorder::task_loop() noexcept {
    const uint32_t cap = nsec_ * SECTOR;
    uint8_t buf[BATCH * REC];
    int n = 0;
    int64_t last_flush = esp_timer_get_time(), last_erase = 0;

    auto flush = [&]() {
        xSemaphoreTake(flash_mtx_, portMAX_DELAY);
        int done = 0;
        while (done < n) {
            const int contig = std::min<int>(n - done, (int)((cap - write_off_) / REC));
            const uint32_t bytes = (uint32_t)contig * REC;
            while (margin_ < bytes && erase_one()) {}       // margin ran out: erase now
            if (margin_ < bytes) break;
            esp_partition_write(part_, write_off_, buf + done * REC, bytes);
            write_off_ = (write_off_ + bytes) % cap;
            margin_ -= bytes;
            session_bytes_ += bytes;
            done += contig;
        }
        xSemaphoreGive(flash_mtx_);
        n = 0;
        last_flush = esp_timer_get_time();
    };

    while (true) {
        if (erase_all_req_.exchange(false) && !flight_active_.load()) {
            erasing_all_.store(true);
            ESP_LOGW(TAG, "Erasing the whole flight log (%lu KB)...", (unsigned long)(cap / 1024));
            for (uint32_t off = 0; off < cap; off += 65536) {
                xSemaphoreTake(flash_mtx_, portMAX_DELAY);
                esp_partition_erase_range(part_, off, std::min<uint32_t>(65536, cap - off));
                xSemaphoreGive(flash_mtx_);
                vTaskDelay(pdMS_TO_TICKS(30));               // let the flight tasks run between blocks
            }
            xQueueReset(q_); n = 0;
            write_off_ = 0; margin_ = cap; erase_next_ = 0; session_bytes_ = 0;
            erasing_all_.store(false);
            log_event(0, "LOG ERASED");
            ESP_LOGW(TAG, "Flight log erased");
        }

        uint8_t rec[REC];
        if (xQueueReceive(q_, rec, pdMS_TO_TICKS(50)) == pdTRUE) {
            memcpy(buf + n * REC, rec, REC);
            if (++n == BATCH) flush();
        }
        const int64_t now = esp_timer_get_time();
        if (n > 0 && now - last_flush > 1000000) flush();

        // Background erase-ahead: pad only, one sector every 250 ms
        if (!flight_active_.load() && margin_ < MARGIN_TARGET && now - last_erase > 250000) {
            xSemaphoreTake(flash_mtx_, portMAX_DELAY);
            erase_one();
            xSemaphoreGive(flash_mtx_);
            last_erase = now;
        }
    }
}

template <typename F>
void FlightRecorder::for_each_record(F&& fn) noexcept {
    // Oldest data sits just after the erased gap, i.e. at erase_next_. Read through a
    // memory-mapped window (cached flash reads), 256 KB at a time, in two linear runs:
    // [start, end) then [0, start).
    constexpr uint32_t WIN = 64 * SECTOR;
    const uint32_t cap = nsec_ * SECTOR;
    const uint32_t start = (erase_next_ / SECTOR) * SECTOR;
    const uint32_t runs[2][2] = { { start, cap }, { 0, start } };
    for (const auto& run : runs) {
        for (uint32_t off = run[0]; off < run[1]; off += WIN) {
            const uint32_t len = std::min(WIN, run[1] - off);
            const void* ptr = nullptr;
            esp_partition_mmap_handle_t h;
            if (esp_partition_mmap(part_, off, len, ESP_PARTITION_MMAP_DATA, &ptr, &h) != ESP_OK) {
                for (uint32_t s = off; s < off + len; s += SECTOR) {          // fallback: plain reads
                    xSemaphoreTake(flash_mtx_, portMAX_DELAY);
                    esp_partition_read(part_, s, s_sector_buf, SECTOR);
                    xSemaphoreGive(flash_mtx_);
                    for (uint32_t k = 0; k < SECTOR / REC; ++k)
                        if (valid(s_sector_buf + k * REC)) fn(s_sector_buf + k * REC);
                }
                continue;
            }
            const uint8_t* base = static_cast<const uint8_t*>(ptr);
            for (uint32_t k = 0; k < len / REC; ++k) {
                const uint8_t* r = base + k * REC;
                if (r[0] == 0xFF && r[1] == 0xFF) continue;                      // erased slot
                if (!valid(r)) continue;
                uint8_t rec[REC];
                memcpy(rec, r, REC);                                              // stable copy for the callback
                fn(rec);
            }
            esp_partition_munmap(h);
        }
    }
}

namespace {
struct SessionInfo {
    uint32_t seq0, boot, n_nav, n_evt, t_first, t_last;
    float    peak;
    bool     resumed;
};
}

void FlightRecorder::list_sessions() noexcept {
    if (!ready_) { printf("LOGLIST,ERROR,no flight log\n"); return; }
    static SessionInfo ss[48];
    int n = 0;
    for_each_record([&](const uint8_t* r) {
        if (r[2] == T_SESSION || n == 0) {
            // n == 0 and no header: the ring overwrote this session's start; list it anyway
            if (n == 48) { memmove(ss, ss + 1, sizeof(SessionInfo) * 47); n = 47; }
            const bool head = (r[2] == T_SESSION);
            ss[n++] = SessionInfo{ get_u32(r + 4), head ? get_u32(r + 12) : 0u, 0, 0, 0, 0, 0.0f, head && r[3] != 0 };
            if (head) return;
        }
        {
            SessionInfo& c = ss[n - 1];
            const uint32_t t = get_u32(r + 8);
            if (c.n_nav + c.n_evt == 0) c.t_first = t;
            c.t_last = t;
            if (r[2] == T_NAV) { c.n_nav++; float a; memcpy(&a, r + 12, 4); c.peak = std::max(c.peak, a); }
            else c.n_evt++;
        }
    });
    const Status st = status();
    printf("LOGLIST,BEGIN,%d,%lu,%lu\n", n, (unsigned long)(st.capacity / 1024), (unsigned long)(st.erased_ahead / 1024));
    for (int i = n - 1, idx = 1; i >= 0; --i, ++idx) {
        const SessionInfo& c = ss[i];
        char boot[24];
        if (c.boot) snprintf(boot, sizeof(boot), "boot %lu", (unsigned long)c.boot);
        else        snprintf(boot, sizeof(boot), "start overwritten");
        printf("LOGLIST,%d,%s,%lu records,%lu events,%.0f s,peak %.1f m%s\n", idx, boot,
               (unsigned long)c.n_nav, (unsigned long)c.n_evt,
               (double)(c.t_last - c.t_first) / 1000.0, (double)c.peak, c.resumed ? ",resumed" : "");
    }
    printf("LOGLIST,END\n");
}

void FlightRecorder::dump_session(int index) noexcept {
    if (!ready_) { printf("LOGBEGIN,ERROR\nLOGEND\n"); return; }
    // Pass 1: sequence number of the requested session header
    static uint32_t heads[48];
    int n = 0;
    for_each_record([&](const uint8_t* r) {
        if (r[2] == T_SESSION || n == 0) {
            if (n == 48) { memmove(heads, heads + 1, sizeof(uint32_t) * 47); n = 47; }
            heads[n++] = get_u32(r + 4);
        }
    });
    if (index < 1 || index > n) { printf("LOGBEGIN,ERROR,no session %d\nLOGEND\n", index); return; }
    const uint32_t s0 = heads[n - index];
    const bool last = (index == 1);
    const uint32_t s1 = last ? 0 : heads[n - index + 1];

    printf("LOGBEGIN,%d\n", index);
    printf("LOGHDR,t_ms,phase,agl_m,vz_mps,baro_m,qw,qx,qy,qz,ax,ay,az,gx,gy,gz,lat,lon,gnss_alt_m,sats,flags,m1,m2,m3,m4,sp_x_deg,sp_y_deg\n");
    uint32_t count = 0;
    for_each_record([&](const uint8_t* r) {
        const uint32_t sq = get_u32(r + 4);
        if ((int32_t)(sq - s0) < 0) return;
        if (!last && (int32_t)(sq - s1) >= 0) return;
        if (r[2] == T_NAV) {
            NavRecord v; memcpy(&v, r, REC);
            printf("LOG,%lu,%u,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.7f,%.7f,%.1f,%u,%u,%u,%u,%u,%u,%d,%d\n",
                   (unsigned long)v.t_ms, v.phase, (double)v.agl_m, (double)v.vz_mps, (double)v.baro_alt_m,
                   v.q[0] / 30000.0, v.q[1] / 30000.0, v.q[2] / 30000.0, v.q[3] / 30000.0,
                   v.acc[0] / 100.0, v.acc[1] / 100.0, v.acc[2] / 100.0,
                   v.gyr[0] / 1000.0, v.gyr[1] / 1000.0, v.gyr[2] / 1000.0,
                   v.lat_e7 / 1e7, v.lon_e7 / 1e7, v.gnss_alt_dm / 10.0, v.sats, v.flags,
                   v.thr[0], v.thr[1], v.thr[2], v.thr[3], v.tilt_sp[0], v.tilt_sp[1]);
        } else if (r[2] == T_EVENT) {
            char txt[51]; memcpy(txt, r + 12, 50); txt[50] = 0;
            printf("LOGEV,%lu,%u,%s\n", (unsigned long)get_u32(r + 8), r[3], txt);
        } else if (r[2] == T_SESSION) {
            float p0; memcpy(&p0, r + 16, 4);
            char b[43]; memcpy(b, r + 20, 42); b[42] = 0;
            printf("LOGSESSION,boot %lu,p0 %.1f Pa,build %s%s\n", (unsigned long)get_u32(r + 12),
                   (double)p0, b, r[3] ? ",resumed after reset" : "");
        }
        if ((++count & 63) == 0) vTaskDelay(1);            // keep the watchdog / USB happy
    });
    printf("LOGEND,%lu\n", (unsigned long)count);
}

} // namespace logging
