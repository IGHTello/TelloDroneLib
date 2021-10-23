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

struct FlightData {
    i16 height;
    i16 north_speed;
    i16 east_speed;
    i16 ground_speed;
    i16 flight_time;
    bool imu_state;
    bool pressure_state;
    bool down_visual_state;
    bool power_state;
    bool battery_state;
    bool gravity_state;
    bool wind_state;
    i8 imu_calibration_state;
    i8 battery_percentage;
    i16 flight_time_left;
    i16 battery_left;
    bool eMSky; // flying?
    bool eMGround; // on ground?
    bool eMOpen;
    bool drone_hover;
    bool outage_recording;
    bool battery_low;
    bool batery_lower;
    bool factory_mode;
    u8 flight_mode;
    u8 throw_fly_timer;
    u8 camera_state;
    u8 electrical_machinery_state;
    bool front_in;
    bool front_out;
    bool front_LSC;
    u8 center_gravity_calibration_status;
    bool soaring_up_into_the_sky;
    bool temperature_height;
};

struct MVOData {
    i16 velocity_x;
    i16 velocity_y;
    i16 velocity_z;
    float position_x;
    float position_y;
    float position_z;
};

struct IMUData {
    float quaternion_w;
    float quaternion_x;
    float quaternion_y;
    float quaternion_z;
    i16 temperature;
};

}
