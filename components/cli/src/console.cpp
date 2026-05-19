/**
 * @file console.cpp
 * @brief USB/Serial CLI implementation.
 */
#include "cli/console.hpp"
#include "esp_log.h"
#include "esp_system.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>

static const char* TAG = "CLI";

namespace cli {

void Console::run() noexcept {
    char line[128];
    printf("\nAAKASHVANI CLI v1.0\n");
    printf("Type 'help' for commands.\n");

    while (true) {
        printf("\n> ");
        fflush(stdout);

        if (fgets(line, sizeof(line), stdin) == nullptr) {
            continue;
        }

        // Strip newline
        line[strcspn(line, "\r\n")] = '\0';

        if (strlen(line) == 0) continue;

        process_line(line);
    }
}

void Console::process_line(char* line) noexcept {
    // Copy for dispatch before tokenizing if needed
    char original[128];
    strncpy(original, line, sizeof(original));

    char* argv[10];
    int argc = 0;
    char* tok = strtok(line, " ");
    while (tok != nullptr && argc < 10) {
        argv[argc++] = tok;
        tok = strtok(nullptr, " ");
    }

    if (argc == 0) return;

    if (strcmp(argv[0], "help") == 0) {
        print_help();
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
    } else {
        printf("Unknown command: %s. Type 'help'.\n", argv[0]);
    }
}

void Console::print_help() noexcept {
    printf("Available commands:\n");
    printf("  help                - Show this help\n");
    printf("  status              - Show system status\n");
    printf("  get <key>           - Get NVS config value\n");
    printf("  set <key> <val...>  - Set NVS config value (use 3 vals for mag_cal)\n");
    printf("  dispatch <raw_cmd>  - Inject a LoRa-style command\n");
    printf("  reboot              - Restart the ESP32\n");
    printf("\nNVS Keys: team_id, ground_alt, baro_offset, bit_override, mag_cal\n");
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
    if (strlen(line) == 0) return;
    
    comms::UplinkCommand cmd{};
    char tmp[64];
    strncpy(tmp, line, sizeof(tmp));
    char* comma = strchr(tmp, ',');
    
    if (comma) {
        *comma = '\0';
        strncpy(cmd.arg, comma + 1, sizeof(cmd.arg) - 1);
    }

    if      (strcmp(tmp, "CX") == 0)    cmd.type = comms::CommandType::CX;
    else if (strcmp(tmp, "ST") == 0)    cmd.type = comms::CommandType::ST;
    else if (strcmp(tmp, "CAL") == 0)   cmd.type = comms::CommandType::CAL;
    else if (strcmp(tmp, "SIM") == 0)   cmd.type = comms::CommandType::SIM;
    else if (strcmp(tmp, "SIMP") == 0)  cmd.type = comms::CommandType::SIMP;
    else if (strcmp(tmp, "SIMG") == 0)  cmd.type = comms::CommandType::SIMG;
    else if (strcmp(tmp, "SIMI") == 0)  cmd.type = comms::CommandType::SIMI;
    else if (strcmp(tmp, "ABORT") == 0) cmd.type = comms::CommandType::ABORT;
    else if (strcmp(tmp, "CHUTE") == 0) cmd.type = comms::CommandType::CHUTE;
    else if (strcmp(tmp, "RTL") == 0)   cmd.type = comms::CommandType::RTL;
    else if (strcmp(tmp, "MAP") == 0)   cmd.type = comms::CommandType::MAP;
    else if (strcmp(tmp, "OTA") == 0)   cmd.type = comms::CommandType::OTA;
    else {
        printf("Unknown command type: %s\n", tmp);
        return;
    }

    printf("Dispatching command %s...\n", tmp);
    parser_.dispatch(cmd);
}

} // namespace cli
