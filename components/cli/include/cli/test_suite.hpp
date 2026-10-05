/**
 * @file test_suite.hpp
 * @brief Hardware and algorithm diagnostic test suite hooks.
 */
#pragma once

#include <cstdint>

namespace test_suite {

void run_test_bit() noexcept;
void run_test_alt_led(bool enable) noexcept;
void run_test_sensors(bool enable) noexcept;
void run_test_motor(float throttle_pct) noexcept;
void run_test_led_scan() noexcept;
void stop_all_tests() noexcept;

} // namespace test_suite
