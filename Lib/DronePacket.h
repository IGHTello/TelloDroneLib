#pragma once

#include "Utils/Types.h"
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace Tello {

enum class PacketDirection {
    TO_DRONE,
    FROM_DRONE
};

enum struct CommandID : u16 {
    GET_SSID = 17,
    SET_SSID = 18,
    GET_WIFI_PASSWORD = 19,
    SET_WIFI_PASSWORD = 20,
    GET_COUNTRY_CODE = 21,
    SET_COUNTRY_CODE = 22,
    WIFI_STATE = 26,
    SET_BITRATE = 32,
    SET_AUTOMATIC_BITRATE = 33,
    SET_EIS = 36,                       // Electronic Image Stabilization
    REQUEST_VIDEO_SPS_PPS_HEADERS = 37, // Sequence number always 0
    GET_BITRATE = 40,
    TAKE_A_PICTURE = 48,
    SET_CAMERA_MODE = 49,
    SET_RECORDING = 50,
    SET_CAMERA_EV = 52, // Exposure Value [-9, 9]
    LIGHT_STRENGTH = 53,
    SET_PHOTO_QUALITY = 55,
    ERROR_TIP_UNK1 = 67,
    ERROR_TIP_UNK2 = 68,
    GET_FIRMWARE_VERSION = 69,
    GET_CURRENT_TIME = 70,
    GET_ACTIVATION_DATA = 71,
    GET_UNIQUE_IDENTIFIER = 72, // Seems to be different to serial number somehow
    GET_LOADER_VERSION = 73,
    SHUTDOWN_DRONE = 74,
    GET_ACTIVATION_STATUS = 75,
    ACTIVATE_DRONE = 76,
    SET_CURRENT_FLIGHT_CONTROLS = 80, // Sequence number always 0
    TAKE_OFF = 84,
    LAND_DRONE = 85,
    FLIGHT_DATA = 86,
    SET_FLIGHT_HEIGHT_LIMIT = 88,
    FLIP_DRONE = 92, // TODO: TestActivity expects payload in response?!
    THROW_AND_FLY = 93,
    PALM_LAND = 94,
    DRONE_LOG_HEADER = 4176,
    DRONE_LOG_DATA = 4177,
    DRONE_LOG_CONFIGURATION = 4178,
    SET_LOW_BATTERY_WARNING = 4181,
    GET_FLIGHT_HEIGHT_LIMIT = 4182,
    GET_LOW_BATTERY_WARNING = 4183,
    SET_ATTITUDE_ANGLE = 4184,
    GET_ATTITUDE_ANGLE = 4185,
    CONN_REQ = 0xFFFE, // These are not real command IDs, they are used to represent the non-standard
    CONN_ACK = 0xFFFF, // packets at the start of the drone-app communication
};

struct DronePacket {
    PacketDirection direction;
    u8 packet_type;
    CommandID cmd_id;
    u16 seq_num;
    std::vector<u8> data;

    DronePacket(u16 seq_num, u8 packet_type, CommandID cmd_id, std::vector<u8> data)
        : seq_num(seq_num)
        , packet_type(packet_type)
        , cmd_id(cmd_id)
        , data(std::move(data))
        , direction(PacketDirection::FROM_DRONE)
    {
    }

    DronePacket(u8 packet_type, CommandID cmd_id, std::vector<u8> data = {})
        : seq_num(-1)
        , packet_type(packet_type)
        , cmd_id(cmd_id)
        , data(std::move(data))
        , direction(PacketDirection::TO_DRONE)
    {
    }

    std::vector<u8> serialize();

    static std::optional<DronePacket> deserialize(std::span<u8> packet_bytes);
};

}
