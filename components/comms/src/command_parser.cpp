/**
 * @file command_parser.cpp
 * @brief Uplink command dispatcher implementation.
 */
#include "comms/command_parser.hpp"
#include "esp_log.h"
#include <cstring>
#include <cstdlib>
#include <cstdio>

static const char* TAG = "CmdParser";

namespace comms {

RxCallback CommandParser::make_rx_callback() noexcept {
    // Capture 'this' — safe as long as CommandParser outlives the LoRaLink.
    return [this](const UplinkCommand& cmd) { dispatch(cmd); };
}

void CommandParser::dispatch(const UplinkCommand& cmd) noexcept {
    switch (cmd.type) {
    case CommandType::CX: {
        bool en = (strcmp(cmd.arg, "ON") == 0);
        ESP_LOGI(TAG, "CX → telemetry %s", en ? "ON" : "OFF");
        if (cx_handler_) cx_handler_(en);
        break;
    }
    case CommandType::ST: {
        uint32_t t = parse_time_str(cmd.arg);
        ESP_LOGI(TAG, "ST → mission time %lu s (arg='%s')", (unsigned long)t, cmd.arg);
        if (st_handler_) st_handler_(t);
        break;
    }
    case CommandType::CAL: {
        ESP_LOGI(TAG, "CAL → calibrate ground altitude");
        if (cal_handler_) cal_handler_();
        break;
    }
    case CommandType::SIM: {
        ESP_LOGI(TAG, "SIM → mode '%s'", cmd.arg);
        if (sim_handler_) sim_handler_(cmd.arg);
        break;
    }
    case CommandType::SIMP: {
        float pa = (float)atof(cmd.arg);
        ESP_LOGI(TAG, "SIMP → %.1f Pa", (double)pa);
        if (simp_handler_) simp_handler_(pa);
        break;
    }
    case CommandType::SIMG: {
        double e=0, n=0, u=0, ve=0, vn=0, vu=0;
        if (sscanf(cmd.arg, "%lf,%lf,%lf,%lf,%lf,%lf", &e, &n, &u, &ve, &vn, &vu) == 6) {
            ESP_LOGI(TAG, "SIMG → pos(%.1f,%.1f,%.1f) vel(%.1f,%.1f,%.1f)", e, n, u, ve, vn, vu);
            if (simg_handler_) simg_handler_(e, n, u, ve, vn, vu);
        }
        break;
    }
    case CommandType::SIMI: {
        double ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
        if (sscanf(cmd.arg, "%lf,%lf,%lf,%lf,%lf,%lf", &ax, &ay, &az, &gx, &gy, &gz) == 6) {
            ESP_LOGI(TAG, "SIMI → acc(%.1f,%.1f,%.1f) gyr(%.1f,%.1f,%.1f)", ax, ay, az, gx, gy, gz);
            if (simi_handler_) simi_handler_(ax, ay, az, gx, gy, gz);
        }
        break;
    }
    case CommandType::ABORT: {
        ESP_LOGW(TAG, "ABORT → emergency abort triggered");
        if (abort_handler_) abort_handler_();
        break;
    }
    case CommandType::CHUTE: {
        ESP_LOGW(TAG, "CHUTE → manual parachute deployment");
        if (chute_handler_) chute_handler_();
        break;
    }
    case CommandType::RTL: {
        ESP_LOGI(TAG, "RTL → return to launch triggered");
        if (rtl_handler_) rtl_handler_();
        break;
    }
    case CommandType::MAP: {
        ESP_LOGI(TAG, "MAP → RF mapping mission toggle");
        if (mapping_handler_) mapping_handler_();
        break;
    }
    case CommandType::OTA: {
        ESP_LOGI(TAG, "OTA → command '%s'", cmd.arg);
        if (ota_handler_) ota_handler_(cmd.arg);
        break;
    }
    default:
        ESP_LOGW(TAG, "Unknown command type %d", (int)cmd.type);
        break;
    }
}

uint32_t CommandParser::parse_time_str(const char* s) noexcept {
    if (!s || s[0] == '\0') return 0;
    unsigned h = 0, m = 0, sec = 0;
    if (sscanf(s, "%u:%u:%u", &h, &m, &sec) != 3) {
        ESP_LOGW(TAG, "parse_time_str failed: '%s'", s);
        return 0;
    }
    return (uint32_t)(h * 3600u + m * 60u + sec);
}

} // namespace comms
