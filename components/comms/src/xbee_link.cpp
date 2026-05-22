/**
 * @file xbee_link.cpp
 * @brief XBee RF link implementation using UART.
 */
#include "comms/xbee_link.hpp"
#include "esp_log.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>

static const char* TAG = "XBeeLink";

namespace comms {

static uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc;
}

esp_err_t XBeeLink::init(hal::UARTBus& uart) noexcept {
    uart_ = &uart;
    tx_queue_ = xQueueCreate(TX_QUEUE_LEN, sizeof(TxEntry));
    if (!tx_queue_) return ESP_ERR_NO_MEM;

    cb_mutex_ = xSemaphoreCreateMutex();
    if (!cb_mutex_) return ESP_ERR_NO_MEM;

    ready_ = true;
    ESP_LOGI(TAG, "XBee link ready (UART %d baud)", (int)nav::TELEM_CFG.xbee_baud);
    return ESP_OK;
}

bool XBeeLink::enqueue_packet(const char* csv, size_t len) noexcept {
    if (!ready_ || !tx_queue_ || !csv || len == 0) return false;
    TxEntry entry{};
    
    // 1. Build the base string: <TEAM_ID>,<PAYLOAD>
    char base[TX_BUF_LEN];
    int n = snprintf(base, sizeof(base), "%u,%s", 
                     (unsigned)nav::TELEM_CFG.team_id, csv);
    if (n < 0 || (size_t)n >= sizeof(base)) return false;

    // 2. Calculate CRC of the base string
    uint16_t crc = crc16_ccitt(reinterpret_cast<const uint8_t*>(base), (size_t)n);

    // 3. Append CRC: <BASE>,<CRC>\n
    int full = snprintf(entry.buf, TX_BUF_LEN, "%s,%04X\n", base, crc);
    if (full < 0 || (size_t)full >= TX_BUF_LEN) return false;
    entry.len = (size_t)full;

    if (uxQueueSpacesAvailable(tx_queue_) == 0) {
        TxEntry discard{};
        xQueueReceive(tx_queue_, &discard, 0);
    }
    return xQueueSend(tx_queue_, &entry, 0) == pdTRUE;
}

void XBeeLink::set_rx_callback(RxCallback cb) noexcept {
    if (cb_mutex_) xSemaphoreTake(cb_mutex_, portMAX_DELAY);
    rx_cb_ = cb;
    if (cb_mutex_) xSemaphoreGive(cb_mutex_);
}

bool XBeeLink::spin() noexcept {
    if (!ready_) return false;

    // --- TX: send all pending packets ---
    TxEntry entry{};
    while (xQueueReceive(tx_queue_, &entry, 0) == pdTRUE) {
        int n = uart_->write((const uint8_t*)entry.buf, entry.len);
        if (n == (int)entry.len) {
            ++tx_count_;
        } else {
            ++tx_errors_;
        }
    }

    // --- RX: Accumulate bytes until newline ---
    uint8_t byte;
    bool received = false;
    while (uart_->read(&byte, 1, 0) > 0) {
        if (byte == '\n' || byte == '\r') {
            if (rx_ptr_ > 0) {
                rx_buf_[rx_ptr_] = '\0';
                last_rx_len_ = rx_ptr_;
                
                if (promiscuous_) {
                    received = true;
                } else {
                    UplinkCommand cmd{};
                    if (parse_uplink(rx_buf_, rx_ptr_, cmd)) {
                        cmd.rssi_dbm = -50; // XBee AT mode doesn't provide RSSI inline
                        cmd.snr_db   = 10.0;
                        ++rx_count_;
                        if (cb_mutex_) xSemaphoreTake(cb_mutex_, portMAX_DELAY);
                        RxCallback local_cb = rx_cb_;
                        if (cb_mutex_) xSemaphoreGive(cb_mutex_);
                        if (local_cb) local_cb(cmd);
                        received = true;
                    }
                }
                rx_ptr_ = 0;
            }
        } else {
            if (rx_ptr_ < RX_BUF_LEN - 1) {
                rx_buf_[rx_ptr_++] = byte;
            } else {
                rx_ptr_ = 0; // Overflow, drop
            }
        }
    }
    return received;
}

bool XBeeLink::parse_uplink(const uint8_t* buf, size_t len,
                             UplinkCommand& out) const noexcept {
    char tmp[RX_BUF_LEN];
    memcpy(tmp, buf, len);
    tmp[len] = '\0';

    char* last_comma = strrchr(tmp, ',');
    if (!last_comma) return false;

    uint16_t received_crc = (uint16_t)strtol(last_comma + 1, nullptr, 16);
    *last_comma = '\0';
    uint16_t actual_crc = crc16_ccitt(reinterpret_cast<const uint8_t*>(tmp), strlen(tmp));

    if (received_crc != actual_crc) return false;

    char* saveptr = nullptr;
    char* tok = strtok_r(tmp, ",", &saveptr);
    if (!tok) return false;
    unsigned team_id = (unsigned)atoi(tok);
    if (team_id != (unsigned)nav::TELEM_CFG.team_id) return false;

    tok = strtok_r(nullptr, ",", &saveptr);
    if (!tok) return false;

    if      (strcmp(tok, "CX")    == 0) out.type = CommandType::CX;
    else if (strcmp(tok, "ST")    == 0) out.type = CommandType::ST;
    else if (strcmp(tok, "SIM")   == 0) out.type = CommandType::SIM;
    else if (strcmp(tok, "SIMP")  == 0) out.type = CommandType::SIMP;
    else if (strcmp(tok, "SIMG")  == 0) out.type = CommandType::SIMG;
    else if (strcmp(tok, "SIMI")  == 0) out.type = CommandType::SIMI;
    else if (strcmp(tok, "CAL")   == 0) out.type = CommandType::CAL;
    else if (strcmp(tok, "ABORT") == 0) out.type = CommandType::ABORT;
    else if (strcmp(tok, "CHUTE") == 0) out.type = CommandType::CHUTE;
    else if (strcmp(tok, "RTL")   == 0) out.type = CommandType::RTL;
    else if (strcmp(tok, "MAP")   == 0) out.type = CommandType::MAP;
    else if (strcmp(tok, "OTA")   == 0) out.type = CommandType::OTA;
    else return false;

    tok = strtok_r(nullptr, ",", &saveptr);
    if (tok) {
        strncpy(out.arg, tok, sizeof(out.arg) - 1);
        out.arg[sizeof(out.arg) - 1] = '\0';
    } else {
        out.arg[0] = '\0';
    }
    return true;
}

} // namespace comms
