/**
 * @file console.hpp
 * @brief USB/Serial CLI for in-field configuration and debugging.
 */
#pragma once

#include "config_mgr/nvs_config.hpp"
#include "comms/command_parser.hpp"

namespace cli {

class Console {
public:
    Console(config_mgr::NVSConfig& nvs, comms::CommandParser& parser) noexcept
        : nvs_(nvs), parser_(parser) {}

    /**
     * @brief Main loop for the CLI. Should be called from a dedicated FreeRTOS task.
     * Blocks on stdin.
     */
    void run() noexcept;

private:
    config_mgr::NVSConfig& nvs_;
    comms::CommandParser& parser_;

    void process_line(char* line) noexcept;
    void print_help() noexcept;
    void handle_get(int argc, char** argv) noexcept;
    void handle_set(int argc, char** argv) noexcept;
    void handle_dispatch(const char* line) noexcept;
};

} // namespace cli
