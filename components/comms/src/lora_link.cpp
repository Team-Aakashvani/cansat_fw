/**
 * @file lora_link.cpp
 * @brief LoRa RF link implementation.
 */
#include "comms/lora_link.hpp"
#include "esp_log.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>

static const char* TAG = "LoRaLink";

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

esp_err_t LoRaLink::init(hal::SPIBus& spi, int cs_pin, int rst_pin, int irq_pin) noexcept {
    tx_queue_ = xQueueCreate(TX_QUEUE_LEN, sizeof(TxEntry));
    if (!tx_queue_) {
        ESP_LOGE(TAG, "Failed to create TX queue");
        return ESP_ERR_NO_MEM;
    }
    cb_mutex_ = xSemaphoreCreateMutex();
    if (!cb_mutex_) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = radio_.init(spi, cs_pin, rst_pin, irq_pin);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SX1278 init failed: %d", ret);
        return ret;
    }
    ready_ = true;
    ESP_LOGI(TAG, "LoRa link ready (433MHz SF10 BW125 CR4/5)");
    return ESP_OK;
}

bool LoRaLink::enqueue_packet(const char* csv, size_t len) noexcept {
    if (!tx_queue_ || !csv || len == 0) return false;
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

    // If full, pop oldest to make room (non-blocking overwrite)
    if (uxQueueSpacesAvailable(tx_queue_) == 0) {
        TxEntry discard{};
        xQueueReceive(tx_queue_, &discard, 0);
        ESP_LOGD(TAG, "TX queue full — dropped oldest");
    }
    return xQueueSend(tx_queue_, &entry, 0) == pdTRUE;
}

void LoRaLink::set_rx_callback(RxCallback cb) noexcept {
    if (cb_mutex_) xSemaphoreTake(cb_mutex_, portMAX_DELAY);
    rx_cb_ = cb;
    if (cb_mutex_) xSemaphoreGive(cb_mutex_);
}

bool LoRaLink::spin() noexcept {
    if (!ready_) return false;

    // --- TX: send one pending packet -----------------------------------------
    TxEntry entry{};
    if (xQueueReceive(tx_queue_, &entry, 0) == pdTRUE) {
        esp_err_t ret = radio_.transmit(
            reinterpret_cast<const uint8_t*>(entry.buf), (uint8_t)entry.len);
        if (ret == ESP_OK) {
            ++tx_count_;
            ESP_LOGD(TAG, "TX[%lu] %zu bytes", (unsigned long)tx_count_, entry.len);
        } else {
            ++tx_errors_;
            ESP_LOGW(TAG, "TX error: %d", ret);
        }
    }

    // --- RX: 200ms window for uplink commands ---------------------------------
    esp_err_t ret = radio_.read_packet(last_rx_pkt_, 200);
    if (ret == ESP_OK && last_rx_pkt_.len > 0) {
        if (promiscuous_) return true;

        UplinkCommand cmd{};
        if (parse_uplink(last_rx_pkt_.data, last_rx_pkt_.len, cmd)) {
            cmd.rssi_dbm = last_rx_pkt_.rssi_dbm;
            cmd.snr_db   = last_rx_pkt_.snr_db;
            ++rx_count_;
            ESP_LOGI(TAG, "RX cmd type=%d arg='%s' RSSI=%d",
                     (int)cmd.type, cmd.arg, (int)cmd.rssi_dbm);
            if (cb_mutex_) xSemaphoreTake(cb_mutex_, portMAX_DELAY);
            RxCallback local_cb = rx_cb_;
            if (cb_mutex_) xSemaphoreGive(cb_mutex_);
            if (local_cb) local_cb(cmd);
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// Internal: parse uplink packet
// Expected: "<TEAM_ID>,<CMD>[,<ARG>]\n"
// ---------------------------------------------------------------------------
bool LoRaLink::parse_uplink(const uint8_t* buf, size_t len,
                             UplinkCommand& out) const noexcept {
    // Copy to null-terminated temp buffer
    char tmp[RX_BUF_LEN + 1];
    size_t copy = (len < RX_BUF_LEN) ? len : RX_BUF_LEN;
    memcpy(tmp, buf, copy);
    tmp[copy] = '\0';
    // Strip trailing whitespace / newline
    for (int i = (int)copy - 1; i >= 0 && (tmp[i] == '\n' || tmp[i] == '\r' || tmp[i] == ' '); --i)
        tmp[i] = '\0';

    // Find the last comma which should precede the CRC
    char* last_comma = strrchr(tmp, ',');
    if (!last_comma) {
        ESP_LOGW(TAG, "RX packet missing CRC comma");
        return false;
    }

    // Verify CRC
    uint16_t received_crc = (uint16_t)strtol(last_comma + 1, nullptr, 16);
    *last_comma = '\0'; // Truncate at the comma to calculate CRC of the rest
    uint16_t actual_crc = crc16_ccitt(reinterpret_cast<const uint8_t*>(tmp), strlen(tmp));

    if (received_crc != actual_crc) {
        ESP_LOGW(TAG, "CRC mismatch: got %04X, expected %04X", received_crc, actual_crc);
        return false;
    }

    // Parse team_id
    char* saveptr = nullptr;
    char* tok = strtok_r(tmp, ",", &saveptr);
    if (!tok) return false;
    unsigned team_id = (unsigned)atoi(tok);
    if (team_id != (unsigned)nav::TELEM_CFG.team_id) {
        ESP_LOGD(TAG, "RX team_id mismatch: got %u expected %u",
                 team_id, (unsigned)nav::TELEM_CFG.team_id);
        return false;
    }

    // Parse command
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
    else {
        out.type = CommandType::UNKNOWN;
        return false;
    }

    // Optional argument
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
