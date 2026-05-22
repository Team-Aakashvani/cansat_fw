/**
 * @file command_parser.hpp
 * @brief Ground-station uplink command dispatcher.
 *
 * Receives UplinkCommand structs from XBeeLink and dispatches them to
 * registered handler callbacks. Each command type has one handler slot.
 *
 * Handlers are called from the radio task context (Core 1).
 * If a handler needs to communicate with other tasks it should use a
 * FreeRTOS queue or event group.
 *
 * Supported commands (CAN-7USAT §6.3):
 *   CX,ON     — Start telemetry
 *   CX,OFF    — Stop telemetry
 *   ST,HH:MM:SS  — Override mission timer
 *   CAL       — Calibrate ground altitude (baro + GNSS)
 *   SIM,ENABLE   — Prepare simulation mode
 *   SIM,DISABLE  — Exit simulation mode
 *   SIM,ACTIVATE — Activate simulation
 *   SIMP,<pa>   — Feed simulated pressure value
 */
#pragma once

#include "comms/comms_types.hpp"
#include <functional>
#include <cstdint>

namespace comms {

/// Command callback signatures
using CxHandler     = std::function<void(bool enable_telemetry)>;
using StHandler     = std::function<void(uint32_t mission_time_s)>;
using CalHandler    = std::function<void()>;
using SimHandler    = std::function<void(const char* mode)>;  // "ENABLE","DISABLE","ACTIVATE"
using SimpHandler   = std::function<void(float pressure_pa)>;
using SimgHandler   = std::function<void(double east, double north, double up, double ve, double vn, double vu)>;
using SimiHandler   = std::function<void(double ax, double ay, double az, double gx, double gy, double gz)>;
using GenericHandler= std::function<void()>;
using OtaHandler    = std::function<void(const char* cmd)>;

class CommandParser {
public:
    CommandParser() noexcept = default;

    // ----- Handler registration -----------------------------------------------

    void on_cx     (CxHandler   h) noexcept { cx_handler_   = h; }
    void on_st     (StHandler   h) noexcept { st_handler_   = h; }
    void on_cal    (CalHandler  h) noexcept { cal_handler_  = h; }
    void on_sim    (SimHandler  h) noexcept { sim_handler_  = h; }
    void on_simp   (SimpHandler h) noexcept { simp_handler_ = h; }
    void on_simg   (SimgHandler h) noexcept { simg_handler_ = h; }
    void on_simi   (SimiHandler h) noexcept { simi_handler_ = h; }
    void on_abort  (GenericHandler h) noexcept { abort_handler_  = h; }
    void on_chute  (GenericHandler h) noexcept { chute_handler_  = h; }
    void on_rtl    (GenericHandler h) noexcept { rtl_handler_    = h; }
    void on_mapping(GenericHandler h) noexcept { mapping_handler_ = h; }
    void on_ota    (OtaHandler     h) noexcept { ota_handler_     = h; }

    /**
     * @brief Build the RxCallback to pass to XBeeLink::set_rx_callback().
     *
     * Usage:
     *   xbee.set_rx_callback(parser.make_rx_callback());
     */
    RxCallback make_rx_callback() noexcept;

    /// Dispatch a command directly (useful for testing / USB injection).
    void dispatch(const UplinkCommand& cmd) noexcept;

private:
    CxHandler   cx_handler_{};
    StHandler   st_handler_{};
    CalHandler  cal_handler_{};
    SimHandler  sim_handler_{};
    SimpHandler simp_handler_{};
    SimgHandler simg_handler_{};
    SimiHandler simi_handler_{};
    GenericHandler abort_handler_{};
    GenericHandler chute_handler_{};
    GenericHandler rtl_handler_{};
    GenericHandler mapping_handler_{};
    OtaHandler     ota_handler_{};

    /// Parse "HH:MM:SS" → seconds. Returns 0 on parse error.
    static uint32_t parse_time_str(const char* s) noexcept;
};

} // namespace comms
