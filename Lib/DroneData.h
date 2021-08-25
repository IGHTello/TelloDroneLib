#pragma once

#include <chrono>
#include <string>
#include <optional>
#include "Utils/Types.h"

namespace Tello {

struct ActivationData {
    std::chrono::system_clock::time_point activation_time;
    u8 raw_serial_data[14];
    std::string serial_number;
    bool unknown_flag;
    std::chrono::system_clock::time_point unknown_time;
    u8 unknown_data[14];
};

struct DroneInfo {
    std::optional<std::string> ssid;
    std::optional<std::string> firmware_version;
    std::optional<std::string> loader_version;
    std::optional<u8> bitrate;
    std::optional<u16> flight_height_limit;
    std::optional<u16> low_battery_warning;
    std::optional<float> attitude_angle;
    std::optional<std::string> country_code;
    std::optional<ActivationData> activation_data;
    std::optional<std::string> unique_identifier;
    std::optional<bool> activation_status;
    u8 light_strength { 0 };
    u8 wifi_strength { 0 };
    u8 wifi_disturb { 0 };
};

}
