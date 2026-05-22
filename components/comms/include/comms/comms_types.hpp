/**
 * @file comms_types.hpp
 * @brief Common types for communication links (LoRa, XBee, etc.)
 */
#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>

namespace comms {

/// Uplink command types decoded from received packets.
enum class CommandType : uint8_t {
    UNKNOWN = 0,
    CX,       ///< Enable/disable telemetry: CX,ON | CX,OFF
    ST,       ///< Set mission time: ST,HH:MM:SS
    SIM,      ///< Simulation mode: SIM,ENABLE | SIM,DISABLE | SIM,ACTIVATE
    SIMP,     ///< Simulated pressure value: SIMP,<pa>
    SIMG,     ///< Simulated GNSS value: SIMG,<lat>,<lon>,<alt>,<ve>,<vn>,<vu>
    SIMI,     ///< Simulated IMU value: SIMI,<ax>,<ay>,<az>,<gx>,<gy>,<gz>
    CAL,      ///< Calibrate ground altitude: CAL
    ABORT,    ///< Emergency mission abort
    CHUTE,    ///< Manual parachute deployment
    RTL,      ///< Return to Launch (controlled descent)
    MAP,      ///< RF mapping toggle
    OTA,      ///< OTA update: OTA,START | OTA,CHUNK,<hex> | OTA,FINISH | OTA,ABORT
};

struct UplinkCommand {
    CommandType type;
    char        arg[32];  ///< Null-terminated argument string (may be empty)
    int8_t      rssi_dbm;
    float       snr_db;
};

/// Callback invoked from the radio task when a valid uplink command is received.
using RxCallback = std::function<void(const UplinkCommand&)>;

} // namespace comms
