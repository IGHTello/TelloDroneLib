#pragma once

#include <chrono>
#include <string>
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
    std::string ssid;
    std::string firmware_version;
    std::string loader_version;
    u8 bitrate { 0 };
    u16 flight_height_limit { 0 };
    u16 low_battery_warning { 0 };
    float attitude_angle { 0 };
    std::string country_code;
    ActivationData activation_data;
    std::string unique_identifier;
    bool activation_status { false };
};

}
