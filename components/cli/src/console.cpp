/**
 * @file console.cpp
 * @brief USB/Serial CLI implementation.
 */
#include "cli/console.hpp"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>

#include "cli/test_suite.hpp"
static const char* TAG = "CLI";

namespace cli {

void Console::run() noexcept {
    char line[128];
    while (true) {
        if (fgets(line, sizeof(line), stdin) == nullptr) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // Strip newline
        line[strcspn(line, "\r\n")] = '\0';

        if (strlen(line) == 0) continue;

        process_line(line);
    }
}

void Console::inject_line(const char* line) noexcept {
    char buf[128];
    snprintf(buf, sizeof(buf), "%s", line ? line : "");
    buf[strcspn(buf, "\r\n")] = '\0';
    if (buf[0] != '\0') process_line(buf);
}

void Console::process_line(char* line) noexcept {
    // Copy for dispatch before tokenizing if needed
    char original[128];
    snprintf(original, sizeof(original), "%s", line);

    char* argv[10];
    int argc = 0;
    char* tok = strtok(line, " ");
    while (tok != nullptr && argc < 10) {
        argv[argc++] = tok;
        tok = strtok(nullptr, " ");
    }

    if (argc == 0) return;

    if (strncmp(original, "CMD,", 4) == 0 || strncmp(original, "cmd,", 4) == 0 ||
        strcasecmp(argv[0], "SIM") == 0 || strcasecmp(argv[0], "SIMP") == 0 ||
        strcasecmp(argv[0], "SIMG") == 0 || strcasecmp(argv[0], "SIMI") == 0 ||
        strcasecmp(argv[0], "CAL") == 0 || strcasecmp(argv[0], "CX") == 0 ||
        strcasecmp(argv[0], "MTR") == 0 || strcasecmp(argv[0], "MOTOR") == 0 ||
        strcasecmp(argv[0], "PID") == 0 ||
        strcasecmp(argv[0], "ABORT") == 0 || strcasecmp(argv[0], "CHUTE") == 0 ||
        strcasecmp(argv[0], "RTL") == 0 || strcasecmp(argv[0], "ST") == 0) {
        handle_dispatch(original);
        return;
    }

    if (strcmp(argv[0], "help") == 0) {
        print_help();
    } else if (strcmp(argv[0], "test") == 0) {
        handle_test(argc, argv);
    } else if (strcmp(argv[0], "get") == 0) {
        handle_get(argc, argv);
    } else if (strcmp(argv[0], "set") == 0) {
        handle_set(argc, argv);
    } else if (strcmp(argv[0], "dispatch") == 0) {
        handle_dispatch(original + 9); // Skip "dispatch "
    } else if (strcmp(argv[0], "reboot") == 0) {
        printf("Rebooting...\n");
        esp_restart();
    } else if (strcmp(argv[0], "status") == 0) {
        printf("Team ID: %u\n", nvs_.get_team_id());
        printf("Boot Count: %lu\n", (unsigned long)nvs_.get_boot_count());
        printf("Ground Alt: %.2f m\n", (double)nvs_.get_ground_alt_m());
        printf("BIT Override: %s\n", nvs_.get_bit_override() ? "ENABLED" : "disabled");
        printf("Heap free: %u B internal (min ever %u B, largest block %u B), PSRAM %u B\n",
               (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
               (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
               (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
               (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        TaskStatus_t ts[24];
        UBaseType_t n = uxTaskGetSystemState(ts, 24, nullptr);
        for (UBaseType_t i = 0; i < n; ++i)
            printf("  task %-12s stack headroom %5u B\n", ts[i].pcTaskName, (unsigned)ts[i].usStackHighWaterMark);
    } else {
        printf("Unknown command: %s. Type 'help' or 'test --help'.\n", argv[0]);
    }
}

void Console::print_help() noexcept {
    printf("Available commands:\n");
    printf("  help                - Show this help\n");
    printf("  test --help         - Show hardware & algorithm diagnostic test modes\n");
    printf("  status              - Show system status\n");
    printf("  get <key>           - Get NVS config value\n");
    printf("  set <key> <val...>  - Set NVS config value (use 3 vals for mag_cal)\n");
    printf("  dispatch <raw_cmd>  - Inject a LoRa-style command\n");
    printf("  reboot              - Restart the ESP32\n");
    printf("\nNVS Keys: team_id, ground_alt, baro_offset, bit_override, mag_cal\n");
}

void Console::print_test_help() noexcept {
    printf("\n========================================================================\n");
    printf("      AAKASHWANI CAN-7USAT HARDWARE & ALGORITHM TEST MODES              \n");
    printf("========================================================================\n");
    printf("  test --help             : Show this testing command options menu\n");
    printf("  test --alt-led          : Live Altitude Height Check via Onboard WS2812\n");
    printf("                            (Lift CanSat >= 0.25m -> Orange, Apogee -> Purple, Land -> Red)\n");
    printf("  test --led-scan         : Scan all ESP32 candidate GPIOs to find onboard discrete LED\n");
    printf("  test --bit              : Run Built-In Self-Test (IMU, Baro, GPS, XBee, SD)\n");
    printf("  test --sensors          : Live streaming readout of all 4 I2C sensors\n");
    printf("  test --motor <0-25>     : Test HakRC ESC1 motor spin (GPIO 14) at %% throttle\n");
    printf("  test --stop             : Stop any running continuous test mode\n");
    printf("========================================================================\n\n");
}

void Console::handle_test(int argc, char** argv) noexcept {
    if (argc < 2 || strcmp(argv[1], "--help") == 0 || strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "help") == 0) {
        print_test_help();
        return;
    }

    if (strcmp(argv[1], "--bit") == 0 || strcmp(argv[1], "bit") == 0) {
        printf("\n[TEST] Triggering System Built-In Self-Test (BIT)...\n");
        test_suite::run_test_bit();
    } else if (strcmp(argv[1], "--alt-led") == 0 || strcmp(argv[1], "alt-led") == 0 || strcmp(argv[1], "--height") == 0) {
        printf("\n[TEST] Starting Altitude Height Check Test with Flight Mission State Machine...\n");
        printf("       Lift CanSat >= 0.25m above desk. Type 'test --stop' to exit.\n");
        test_suite::run_test_alt_led(true);
    } else if (strcmp(argv[1], "--led-scan") == 0 || strcmp(argv[1], "led-scan") == 0 || strcmp(argv[1], "--leds") == 0) {
        test_suite::run_test_led_scan();
    } else if (strcmp(argv[1], "--sensors") == 0 || strcmp(argv[1], "sensors") == 0) {
        printf("\n[TEST] Streaming 4x I2C sensor constellation readings. Type 'test --stop' to exit.\n");
        test_suite::run_test_sensors(true);
    } else if (strcmp(argv[1], "--motor") == 0 || strcmp(argv[1], "motor") == 0) {
        float pct = (argc >= 3) ? (float)atof(argv[2]) : 5.0f;
        if (pct < 0.0f) pct = 0.0f;
        if (pct > 25.0f) {
            printf("[SAFETY] Bench throttle capped at 25%% without propellers!\n");
            pct = 25.0f;
        }
        printf("\n[TEST] Testing HakRC ESC1 (GPIO 14) at %.1f%% throttle for 2.5 seconds...\n", (double)pct);
        test_suite::run_test_motor(pct);
    } else if (strcmp(argv[1], "--stop") == 0 || strcmp(argv[1], "stop") == 0) {
        printf("\n[TEST] Stopping all active test modes. Restoring flight standby.\n");
        test_suite::stop_all_tests();
    } else {
        printf("Unknown test option: '%s'. Type 'test --help' for options.\n", argv[1]);
    }
}

void Console::handle_get(int argc, char** argv) noexcept {
    if (argc < 2) {
        printf("Usage: get <key>\n");
        return;
    }

    if (strcmp(argv[1], "team_id") == 0) {
        printf("team_id = %u\n", nvs_.get_team_id());
    } else if (strcmp(argv[1], "ground_alt") == 0) {
        printf("ground_alt = %.2f\n", (double)nvs_.get_ground_alt_m());
    } else if (strcmp(argv[1], "baro_offset") == 0) {
        printf("baro_offset = %.2f\n", (double)nvs_.get_baro_offset_pa());
    } else if (strcmp(argv[1], "bit_override") == 0) {
        printf("bit_override = %d\n", nvs_.get_bit_override());
    } else if (strcmp(argv[1], "mag_cal") == 0) {
        float cal[3];
        nvs_.get_mag_cal(cal);
        printf("mag_cal = [%.3f, %.3f, %.3f]\n", (double)cal[0], (double)cal[1], (double)cal[2]);
    } else {
        printf("Unknown key: %s\n", argv[1]);
    }
}

void Console::handle_set(int argc, char** argv) noexcept {
    if (argc < 3) {
        printf("Usage: set <key> <value>\n");
        return;
    }

    if (strcmp(argv[1], "team_id") == 0) {
        uint16_t id = (uint16_t)atoi(argv[2]);
        nvs_.set_team_id(id);
        printf("Set team_id = %u\n", id);
    } else if (strcmp(argv[1], "ground_alt") == 0) {
        float val = (float)atof(argv[2]);
        nvs_.set_ground_alt_m(val);
        printf("Set ground_alt = %.2f\n", (double)val);
    } else if (strcmp(argv[1], "baro_offset") == 0) {
        float val = (float)atof(argv[2]);
        nvs_.set_baro_offset_pa(val);
        printf("Set baro_offset = %.2f\n", (double)val);
    } else if (strcmp(argv[1], "bit_override") == 0) {
        bool val = (atoi(argv[2]) != 0);
        nvs_.set_bit_override(val);
        printf("Set bit_override = %d\n", val);
    } else if (strcmp(argv[1], "mag_cal") == 0) {
        if (argc < 5) {
            printf("Usage: set mag_cal <x> <y> <z>\n");
            return;
        }
        float cal[3];
        cal[0] = (float)atof(argv[2]);
        cal[1] = (float)atof(argv[3]);
        cal[2] = (float)atof(argv[4]);
        nvs_.set_mag_cal(cal);
        printf("Set mag_cal = [%.3f, %.3f, %.3f]\n", (double)cal[0], (double)cal[1], (double)cal[2]);
    } else {
        printf("Unknown key: %s\n", argv[1]);
    }
}

void Console::handle_dispatch(const char* line) noexcept {
    if (!line || strlen(line) == 0) return;
    
    comms::UplinkCommand cmd{};
    char tmp[128];
    snprintf(tmp, sizeof(tmp), "%s", line);
    char* p = tmp;

    // Skip "CMD," prefix if present
    if (strncmp(p, "CMD,", 4) == 0 || strncmp(p, "cmd,", 4) == 0) {
        p += 4;
    }

    char* saveptr = nullptr;
    char* tok = strtok_r(p, ", ", &saveptr);
    if (!tok) return;

    // Skip optional team_id (e.g. "001")
    unsigned team_id = 0;
    if (sscanf(tok, "%u", &team_id) == 1 && (team_id == (unsigned)nav::TELEM_CFG.team_id || team_id == 0)) {
        tok = strtok_r(nullptr, ", ", &saveptr);
        if (!tok) return;
    }

    if      (strcasecmp(tok, "CX") == 0)    cmd.type = comms::CommandType::CX;
    else if (strcasecmp(tok, "ST") == 0)    cmd.type = comms::CommandType::ST;
    else if (strcasecmp(tok, "CAL") == 0)   cmd.type = comms::CommandType::CAL;
    else if (strcasecmp(tok, "SIM") == 0)   cmd.type = comms::CommandType::SIM;
    else if (strcasecmp(tok, "SIMP") == 0)  cmd.type = comms::CommandType::SIMP;
    else if (strcasecmp(tok, "SIMG") == 0)  cmd.type = comms::CommandType::SIMG;
    else if (strcasecmp(tok, "SIMI") == 0)  cmd.type = comms::CommandType::SIMI;
    else if (strcasecmp(tok, "ABORT") == 0) cmd.type = comms::CommandType::ABORT;
    else if (strcasecmp(tok, "CHUTE") == 0) cmd.type = comms::CommandType::CHUTE;
    else if (strcasecmp(tok, "RTL") == 0)   cmd.type = comms::CommandType::RTL;
    else if (strcasecmp(tok, "MTR") == 0 || strcasecmp(tok, "MOTOR") == 0) cmd.type = comms::CommandType::MTR;
    else if (strcasecmp(tok, "PID") == 0)                                   cmd.type = comms::CommandType::PID;
    else if (strcasecmp(tok, "MAP") == 0)   cmd.type = comms::CommandType::MAP;
    else if (strcasecmp(tok, "OTA") == 0)   cmd.type = comms::CommandType::OTA;
    else if (strcasecmp(tok, "TARE") == 0)  cmd.type = comms::CommandType::TARE;
    else if (strcasecmp(tok, "NORTH") == 0) cmd.type = comms::CommandType::NORTH;
    else if (strcasecmp(tok, "LIFT") == 0)  cmd.type = comms::CommandType::LIFT;
    else if (strcasecmp(tok, "LOG") == 0)   cmd.type = comms::CommandType::LOG;
    else {
        printf("Unknown command type: %s\n", tok);
        return;
    }

    tok = strtok_r(nullptr, "\r\n", &saveptr);
    if (tok) {
        while (*tok == ',' || *tok == ' ') tok++;
        // Remove trailing CRC if present
        char* last_comma = strrchr(tok, ',');
        if (last_comma && strlen(last_comma + 1) == 4) {
            *last_comma = '\0';
        }
        strncpy(cmd.arg, tok, sizeof(cmd.arg) - 1);
        cmd.arg[sizeof(cmd.arg) - 1] = '\0';
    } else {
        cmd.arg[0] = '\0';
    }

    printf("[CMD] Executing: %s (%s)\n", line, cmd.arg);
    parser_.dispatch(cmd);
}

} // namespace cli
