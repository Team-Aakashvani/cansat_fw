/**
 * @file ble_link.cpp
 * @brief NimBLE peripheral exposing a Nordic-UART-style line service.
 */
#include "comms/ble_link.hpp"
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <atomic>
#include <cstring>
#include <cstdio>
#include <algorithm>

static const char* TAG = "BLE";

namespace comms {

namespace {

// 6E400001-B5A3-F393-E0A9-E50E24DCCA9E (little-endian byte order)
const ble_uuid128_t UUID_SVC = BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                                                0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);
const ble_uuid128_t UUID_RX  = BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                                                0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);
const ble_uuid128_t UUID_TX  = BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                                                0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

BleLink::LineHandler s_on_line;
uint16_t              s_tx_handle = 0;
std::atomic<uint16_t> s_conn{BLE_HS_CONN_HANDLE_NONE};
std::atomic<bool>     s_subscribed{false};
std::atomic<bool>     s_enabled{false};
std::atomic<bool>     s_synced{false};
std::atomic<uint16_t> s_mtu{23};
uint8_t               s_addr_type = 0;
char                  s_rx_line[160];
size_t                s_rx_len = 0;

int gap_event(struct ble_gap_event* ev, void* arg);

void advertise() {
    if (!s_enabled.load() || !s_synced.load() || s_conn.load() != BLE_HS_CONN_HANDLE_NONE) return;
    if (ble_gap_adv_active()) return;

    ble_hs_adv_fields f{};
    f.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    const char* name = ble_svc_gap_device_name();
    f.name = (uint8_t*)name; f.name_len = (uint8_t)strlen(name); f.name_is_complete = 1;
    f.tx_pwr_lvl_is_present = 1; f.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    if (ble_gap_adv_set_fields(&f) != 0) { ESP_LOGE(TAG, "adv fields"); return; }

    ble_hs_adv_fields rsp{};                         // 128-bit UUID does not fit beside the name
    rsp.uuids128 = (ble_uuid128_t*)&UUID_SVC; rsp.num_uuids128 = 1; rsp.uuids128_is_complete = 1;
    ble_gap_adv_rsp_set_fields(&rsp);

    ble_gap_adv_params p{};
    p.conn_mode = BLE_GAP_CONN_MODE_UND;
    p.disc_mode = BLE_GAP_DISC_MODE_GEN;
    p.itvl_min = 160; p.itvl_max = 320;              // 100-200 ms: modest power / RF
    const int rc = ble_gap_adv_start(s_addr_type, nullptr, BLE_HS_FOREVER, &p, gap_event, nullptr);
    if (rc != 0) ESP_LOGW(TAG, "adv start rc=%d", rc);
}

int gap_event(struct ble_gap_event* ev, void* /*arg*/) {
    switch (ev->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (ev->connect.status == 0) {
            s_conn.store(ev->connect.conn_handle);
            ESP_LOGI(TAG, "client connected");
        } else {
            advertise();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        s_conn.store(BLE_HS_CONN_HANDLE_NONE);
        s_subscribed.store(false);
        s_mtu.store(23);
        ESP_LOGI(TAG, "client disconnected (reason %d)", ev->disconnect.reason);
        advertise();
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        advertise();
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (ev->subscribe.attr_handle == s_tx_handle) s_subscribed.store(ev->subscribe.cur_notify != 0);
        break;
    case BLE_GAP_EVENT_MTU:
        s_mtu.store(ev->mtu.value);
        break;
    default:
        break;
    }
    return 0;
}

int chr_access(uint16_t /*conn*/, uint16_t /*attr*/, struct ble_gatt_access_ctxt* ctxt, void* /*arg*/) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return 0;
    uint8_t buf[256];
    uint16_t n = 0;
    if (ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &n) != 0) return BLE_ATT_ERR_UNLIKELY;
    for (uint16_t i = 0; i < n; ++i) {
        const char c = (char)buf[i];
        if (c == '\n' || c == '\r') {
            if (s_rx_len > 0) {
                s_rx_line[s_rx_len] = '\0';
                if (s_on_line) s_on_line(s_rx_line);
                s_rx_len = 0;
            }
        } else if (s_rx_len < sizeof(s_rx_line) - 1) {
            s_rx_line[s_rx_len++] = c;
        }
    }
    return 0;
}

ble_gatt_chr_def s_chrs[3];
ble_gatt_svc_def s_svcs[2];

void on_sync() {
    ble_hs_util_ensure_addr(0);
    ble_hs_id_infer_auto(0, &s_addr_type);
    s_synced.store(true);
    advertise();
}

void on_reset(int reason) { s_synced.store(false); ESP_LOGW(TAG, "host reset (%d)", reason); }

void host_task(void*) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

} // namespace

bool BleLink::init(const char* name, LineHandler on_line) noexcept {
    s_on_line = std::move(on_line);
    if (nimble_port_init() != ESP_OK) { ESP_LOGE(TAG, "nimble_port_init failed"); return false; }

    memset(s_chrs, 0, sizeof(s_chrs));
    s_chrs[0].uuid = &UUID_RX.u;
    s_chrs[0].access_cb = chr_access;
    s_chrs[0].flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP;
    s_chrs[1].uuid = &UUID_TX.u;
    s_chrs[1].access_cb = chr_access;
    s_chrs[1].flags = BLE_GATT_CHR_F_NOTIFY;
    s_chrs[1].val_handle = &s_tx_handle;
    memset(s_svcs, 0, sizeof(s_svcs));
    s_svcs[0].type = BLE_GATT_SVC_TYPE_PRIMARY;
    s_svcs[0].uuid = &UUID_SVC.u;
    s_svcs[0].characteristics = s_chrs;

    ble_hs_cfg.sync_cb  = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
    ble_svc_gap_init();
    ble_svc_gatt_init();
    if (ble_gatts_count_cfg(s_svcs) != 0 || ble_gatts_add_svcs(s_svcs) != 0) {
        ESP_LOGE(TAG, "GATT registration failed");
        return false;
    }
    ble_svc_gap_device_name_set(name);
    s_enabled.store(true);
    nimble_port_freertos_init(host_task);
    ESP_LOGI(TAG, "BLE link advertising as '%s'", name);
    return true;
}

void BleLink::set_enabled(bool en) noexcept {
    if (s_enabled.exchange(en) == en) return;
    if (en) {
        advertise();
        ESP_LOGI(TAG, "BLE link enabled");
    } else {
        if (ble_gap_adv_active()) ble_gap_adv_stop();
        const uint16_t c = s_conn.load();
        if (c != BLE_HS_CONN_HANDLE_NONE) ble_gap_terminate(c, BLE_ERR_REM_USER_CONN_TERM);
        ESP_LOGW(TAG, "BLE link disabled (radio silent)");
    }
}

bool BleLink::enabled() const noexcept { return s_enabled.load(); }

bool BleLink::connected() const noexcept {
    return s_conn.load() != BLE_HS_CONN_HANDLE_NONE && s_subscribed.load();
}

bool BleLink::send_line(const char* line) noexcept {
    if (!connected() || !s_enabled.load() || !line) return false;
    const uint16_t c = s_conn.load();
    char msg[320];
    const int len = snprintf(msg, sizeof(msg), "%s\n", line);
    if (len <= 0) return false;
    const size_t total = std::min<size_t>((size_t)len, sizeof(msg) - 1);
    const size_t chunk = (size_t)std::max<int>(20, std::min<int>(240, (int)s_mtu.load() - 3));
    for (size_t off = 0; off < total; off += chunk) {
        const size_t n = std::min(chunk, total - off);
        os_mbuf* om = ble_hs_mbuf_from_flat(msg + off, (uint16_t)n);
        if (!om || ble_gatts_notify_custom(c, s_tx_handle, om) != 0) return false;   // out of buffers: drop
    }
    return true;
}

} // namespace comms
