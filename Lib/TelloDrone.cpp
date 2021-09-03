#include "TelloDrone.h"
#include "Utils/StringHelpers.h"
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <span>
#include <sstream>
#include <thread>
#include <unistd.h>

namespace Tello {

static constexpr u16 TELLO_CMD_PORT = 8889;
static constexpr u16 TELLO_VIDEO_PORT = 7777;
static constexpr char const* TELLO_CMD_IP = "192.168.10.1";
static constexpr u16 FFMPEG_PORT = 9999;
static constexpr char const* FFMPEG_IP = "127.0.0.1";
static constexpr std::chrono::seconds PACKET_ACK_TIMEOUT = std::chrono::seconds(10);

Drone::Drone()
{
    m_video_socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (m_video_socket_fd == -1) {
        perror("socket() -> m_video_socket_fd");
        exit(1);
    }
    sockaddr_in video_receive_addr {};
    video_receive_addr.sin_family = AF_INET;
    video_receive_addr.sin_addr.s_addr = INADDR_ANY;
    video_receive_addr.sin_port = htons(TELLO_VIDEO_PORT);
    if (bind(m_video_socket_fd, reinterpret_cast<sockaddr*>(&video_receive_addr), sizeof(sockaddr_in)) < 0) {
        perror("bind(m_video_socket_fd)");
        exit(1);
    }
    timeval sock_timeout {};
    sock_timeout.tv_sec = 1;
    sock_timeout.tv_usec = 0;
    if (setsockopt(m_video_socket_fd, SOL_SOCKET, SO_RCVTIMEO, (char*)&sock_timeout, sizeof(sock_timeout)) < 0) {
        perror("setsockopt()");
        exit(1);
    }

    m_ffmpeg_socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (m_ffmpeg_socket_fd == -1) {
        perror("socket() -> m_ffmpeg_socket_fd");
        exit(1);
    }
    m_ffmpeg_addr.sin_family = AF_INET;
    m_ffmpeg_addr.sin_port = htons(FFMPEG_PORT);
    m_ffmpeg_addr.sin_addr.s_addr = inet_addr(FFMPEG_IP);

    m_cmd_socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (m_cmd_socket_fd == -1) {
        perror("socket() -> m_cmd_socket_fd");
        exit(1);
    }
    m_cmd_addr.sin_family = AF_INET;
    m_cmd_addr.sin_port = htons(TELLO_CMD_PORT);
    m_cmd_addr.sin_addr.s_addr = inet_addr(TELLO_CMD_IP);

    if (setsockopt(m_cmd_socket_fd, SOL_SOCKET, SO_RCVTIMEO, (char*)&sock_timeout, sizeof(sock_timeout)) < 0) {
        perror("setsockopt()");
        exit(1);
    }

    m_video_receive_thread = std::thread(&Drone::video_receive_thread_routine, this);
    m_cmd_receive_thread = std::thread(&Drone::cmd_receive_thread_routine, this);
    m_drone_controls_thread = std::thread(&Drone::drone_controls_thread_routine, this);

    send_setup_packet();
}

void Drone::video_receive_thread_routine()
{
    std::vector<u8> current_frame;
    usize current_frame_num = 0;
    isize last_segment_num_received = -1;
    bool discard_current_frame = false;
    bool received_sequence_parameter_set = false;
    u8 frames_since_last_SPS_request = 0;

    u8 packet_buffer[4096];
    while (!m_shutting_down) {
        isize bytes_received = recvfrom(m_video_socket_fd, packet_buffer, sizeof(packet_buffer), 0, nullptr,
            nullptr);

        if (bytes_received < 0) {
            if (errno != EAGAIN)
                std::cerr << "Failed to receive bytes from video socket, errno: " << strerror(errno) << std::endl;
            continue;
        }

        if (bytes_received < 2) {
            if constexpr (VIDEO_DEBUG_LOGGING)
                std::cerr << "Received invalid video packet, less than 2 bytes of data!" << std::endl;
            continue;
        }

        int frame_num = packet_buffer[0];
        auto segment_num = packet_buffer[1] & 127;
        auto last_segment_in_frame = (packet_buffer[1] & 128) == 128;

        if constexpr (VERBOSE_VIDEO_DEBUG_LOGGING)
            std::cout << "Got segment " << segment_num << " of frame " << frame_num << " (end="
                      << last_segment_in_frame
                      << "), last was " << last_segment_num_received << " of frame " << current_frame_num
                      << std::endl;

        if (frame_num != current_frame_num) {
            if constexpr (VERBOSE_VIDEO_DEBUG_LOGGING)
                std::cout << "Lost segments on frame boundary " << current_frame_num << ':' << frame_num
                          << std::endl;
            // Seems like we lost part of the last frame, so we'll have to discard it

            current_frame_num = frame_num;
            // Fixup the `last_segment_num_received` counter so we won't also detect an intra-frame
            last_segment_num_received = segment_num - 1;

            if (segment_num != 0) {
                // We also lost some frames in this next frame, so we'll have to discard it too
                discard_current_frame = true;
            } else {
                // If by chance we did not skip any segments in the next frame, we don't have to discard it too
                current_frame.clear();
            }
        }

        if (((last_segment_num_received + 1) & 127) != segment_num) {
            if constexpr (VERBOSE_VIDEO_DEBUG_LOGGING)
                std::cout << "Lost segments of frame " << current_frame_num << std::endl;
            // Seems like we lost part of this frame, discard it
            discard_current_frame = true;
        }

        last_segment_num_received = segment_num;
        if (!discard_current_frame) [[likely]] {
            current_frame.reserve(current_frame.size() + bytes_received - 2);
            current_frame.insert(current_frame.end(), packet_buffer + 2, packet_buffer + bytes_received);
        }

        if (last_segment_in_frame) {
            if (!discard_current_frame) {
                if constexpr (VIDEO_DEBUG_LOGGING)
                    std::cout << "Finished receiving full frame" << std::endl;

                if (current_frame[0] == 0x00 && current_frame[1] == 0x00 && current_frame[2] == 0x00 && current_frame[3] == 0x01) { // NAL Unit Start Code Prefix
                    u8 nal_type = current_frame[4] & 0x1F;
                    if (nal_type == 7) {
                        if constexpr (VERBOSE_VIDEO_DEBUG_LOGGING)
                            std::cout << "Received sequence parameter set" << std::endl;
                        received_sequence_parameter_set = true;
                    }
                }
                if (received_sequence_parameter_set) {
                    sendto(m_ffmpeg_socket_fd, current_frame.data(), current_frame.size(), 0,
                        reinterpret_cast<const sockaddr*>(&m_ffmpeg_addr), sizeof(m_ffmpeg_addr));
                } else {
                    if (frames_since_last_SPS_request == 8) {
                        if constexpr (VERBOSE_VIDEO_DEBUG_LOGGING)
                            std::cout << "Requesting sequence parameter set" << std::endl;
                        queue_packet(DronePacket(DronePacket(96, CommandID::REQUEST_VIDEO_SPS_PPS_HEADERS)));
                        frames_since_last_SPS_request = 0;
                    }
                    frames_since_last_SPS_request++;
                }
            }

            current_frame.clear();
            current_frame_num = (current_frame_num + 1) & 255;
            last_segment_num_received = -1;
            discard_current_frame = false;
        }
    }
}

void Drone::cmd_receive_thread_routine()
{
    socklen_t cmd_addr_size = sizeof(m_cmd_addr);
    u8 packet_buffer[4096];
    while (!m_shutting_down) {
        isize bytes_received = recvfrom(m_cmd_socket_fd, packet_buffer, sizeof(packet_buffer), 0,
            reinterpret_cast<sockaddr*>(&m_cmd_addr), &cmd_addr_size);
        if (bytes_received < 0) {
            if (errno != EAGAIN)
                std::cerr << "Failed to receive bytes from cmd socket, errno: " << strerror(errno) << std::endl;
            continue;
        }

        auto packet = DronePacket::deserialize(std::span<u8>(packet_buffer, bytes_received));
        if (packet.has_value())
            handle_packet(packet.value());
        else if constexpr (DRONE_DEBUG_LOGGING)
            std::cerr << "Failed to parse packet of length `" << bytes_received << "`" << std::endl;
    }
}

void Drone::send_setup_packet()
{
    std::vector<u8> packet_bytes(2);
    packet_bytes[0] = TELLO_VIDEO_PORT & 0xFF;
    packet_bytes[1] = (TELLO_VIDEO_PORT >> 8) & 0xFF;
    queue_packet(DronePacket(0, CommandID::CONN_REQ, std::move(packet_bytes)));
}

void Drone::send_initialization_sequence()
{
    queue_packet(DronePacket(96, CommandID::REQUEST_VIDEO_SPS_PPS_HEADERS));
    queue_packet(DronePacket(72, CommandID::GET_FIRMWARE_VERSION));
    queue_packet(DronePacket(72, CommandID::GET_LOADER_VERSION));
    queue_packet(DronePacket(72, CommandID::GET_BITRATE));
    queue_packet(DronePacket(72, CommandID::GET_FLIGHT_HEIGHT_LIMIT));
    queue_packet(DronePacket(72, CommandID::GET_LOW_BATTERY_WARNING));
    queue_packet(DronePacket(72, CommandID::GET_ATTITUDE_ANGLE));
    queue_packet(DronePacket(72, CommandID::GET_COUNTRY_CODE));
    queue_packet(DronePacket(72, CommandID::SET_CAMERA_EV, { 0x00 }));
    queue_packet(DronePacket(72, CommandID::SET_PHOTO_QUALITY, { 0x00 }));
    queue_packet(DronePacket(72, CommandID::SET_BITRATE, { 0x00 }));
    queue_packet(DronePacket(104, CommandID::SET_RECORDING, { 0x00 }));
    queue_packet(DronePacket(72, CommandID::GET_SSID));
    queue_packet(DronePacket(72, CommandID::SET_CAMERA_MODE, { 0x00 }));
    queue_packet(DronePacket(72, CommandID::GET_ACTIVATION_DATA));
    queue_packet(DronePacket(72, CommandID::GET_UNIQUE_IDENTIFIER));
    queue_packet(DronePacket(72, CommandID::GET_ACTIVATION_STATUS));
}

void Drone::send_timed_requests_if_needed()
{
    if (m_timed_request_ticks >= 50) {
        m_timed_request_ticks = 0;
        if (m_connected)
            queue_packet(DronePacket(96, CommandID::REQUEST_VIDEO_SPS_PPS_HEADERS));
        else
            send_setup_packet();
    }
    m_timed_request_ticks++;
}

void Drone::drone_controls_thread_routine()
{
    while (!m_shutting_down) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        send_timed_requests_if_needed();

        std::vector<u8> packet_data(11);

        std::unique_lock<std::mutex> lock(m_received_acks_mutex);
        u64 packed_drone_controls = ((u64)m_right_stick_x & 0x7FF) | (((u64)m_right_stick_y & 0x7FF) << 11) | (((u64)m_left_stick_y & 0x7FF) << 22) | (((u64)m_left_stick_x & 0x7FF) << 33) | ((u64)m_quick_mode << 44);
        lock.unlock();
        packet_data[0] = packed_drone_controls & 0xFF;
        packet_data[1] = (packed_drone_controls >> 8) & 0xFF;
        packet_data[2] = (packed_drone_controls >> 16) & 0xFF;
        packet_data[3] = (packed_drone_controls >> 24) & 0xFF;
        packet_data[4] = (packed_drone_controls >> 32) & 0xFF;
        packet_data[5] = (packed_drone_controls >> 40) & 0xFF;

        auto current_time_point = std::chrono::system_clock::now();
        auto days = std::chrono::floor<std::chrono::days>(current_time_point);
        auto time = std::chrono::hh_mm_ss(std::chrono::floor<std::chrono::milliseconds>(current_time_point - days));
        packet_data[6] = time.hours().count();
        packet_data[7] = time.minutes().count();
        packet_data[8] = time.seconds().count();
        auto milliseconds = time.subseconds().count();
        packet_data[9] = milliseconds & 0xFF;
        packet_data[10] = (milliseconds >> 8) & 0xFF;

        queue_packet(DronePacket(96, CommandID::SET_CURRENT_FLIGHT_CONTROLS, std::move(packet_data)));
    }
}

Drone::~Drone()
{
    close();
}

bool Drone::is_connected()
{
    std::unique_lock<std::mutex> lock(m_connected_mutex);
    return m_connected;
}

void Drone::wait_until_connected()
{
    std::unique_lock<std::mutex> lock(m_connected_mutex);
    m_connected_cv.wait(lock, [this]() { return m_connected; });
}

void Drone::close()
{
    if (m_shutting_down)
        return;

    queue_packet(DronePacket(104, CommandID::LAND_DRONE, { 0x00 }));

    m_shutting_down = true;
    m_video_receive_thread.join();
    ::close(m_video_socket_fd);
    m_cmd_receive_thread.join();
    ::close(m_cmd_socket_fd);
    m_drone_controls_thread.join();
}

void Drone::queue_packet_internal(DronePacket& packet)
{
    assert(packet.direction == PacketDirection::TO_DRONE);
    if (packet.cmd_id == CommandID::CONN_REQ || packet.cmd_id == CommandID::REQUEST_VIDEO_SPS_PPS_HEADERS || packet.cmd_id == CommandID::SET_CURRENT_FLIGHT_CONTROLS) {
        packet.seq_num = 0;
    } else {
        packet.seq_num = m_cmd_seq_num++;
        std::unique_lock<std::mutex> lock(m_received_acks_mutex);
        m_received_acks[packet.seq_num] = false;
    }
    auto packet_bytes = packet.serialize();
    sendto(m_cmd_socket_fd, packet_bytes.data(), packet_bytes.size(), 0,
        reinterpret_cast<const sockaddr*>(&m_cmd_addr), sizeof(m_cmd_addr));
}

bool Drone::send_packet_and_wait_until_ack(DronePacket packet)
{
    queue_packet_internal(packet);
    if constexpr (VERBOSE_DRONE_DEBUG_LOGGING)
        std::cout << "Waiting for ack for packet " << packet.seq_num << " of type " << static_cast<u16>(packet.cmd_id) << std::endl;
    std::unique_lock<std::mutex> lock(m_received_acks_mutex);
    return m_received_acks_cv.wait_for(lock, PACKET_ACK_TIMEOUT, [this, seq_num = packet.seq_num]() { return m_received_acks[seq_num]; });
}

void Drone::send_packet_and_assert_ack(DronePacket packet)
{
    auto ack_received = send_packet_and_wait_until_ack(std::move(packet));
    assert(ack_received);
}

void Drone::handle_packet(const DronePacket& packet)
{
    assert(packet.direction == PacketDirection::FROM_DRONE);

    if constexpr (VERBOSE_DRONE_DEBUG_LOGGING)
        std::cout << "Received packet of type " << static_cast<u16>(packet.cmd_id) << std::endl;

    bool success = !packet.data.empty() && packet.data[0] == 0;
    switch (packet.cmd_id) {
    case CommandID::FLIGHT_DATA: {
        auto current_time = std::chrono::system_clock::now();
        auto time_since_last_update = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - m_last_update_time);
        if (time_since_last_update.count() > 3000) {
            std::unique_lock<std::mutex> lock(m_connected_mutex);
            m_connected = false;
        } else {
            std::unique_lock<std::mutex> lock(m_connected_mutex);
            bool was_connected = m_connected;
            m_connected = true;
            lock.unlock();

            if (!was_connected)
                send_initialization_sequence();

            m_connected_cv.notify_all();
        }
        m_last_update_time = current_time;
        // TODO: READ FLIGHT DATA
        break;
    }
    case CommandID::CONN_ACK: {
        if constexpr (DRONE_DEBUG_LOGGING)
            std::cout << "Received connection acknowledgement!" << std::endl;
        break;
    }
    case CommandID::SET_SSID:
    case CommandID::SET_COUNTRY_CODE:
    case CommandID::SET_WIFI_PASSWORD:
    case CommandID::SET_ATTITUDE_ANGLE:
    case CommandID::ACTIVATE_DRONE:
    case CommandID::SET_BITRATE:
    case CommandID::SET_EIS:
    case CommandID::SET_AUTOMATIC_BITRATE:
    case CommandID::SET_RECORDING:
    case CommandID::SET_CAMERA_EV:
    case CommandID::SET_PHOTO_QUALITY:
    case CommandID::SET_CAMERA_MODE:
    case CommandID::LAND_DRONE:
    case CommandID::TAKE_OFF:
    case CommandID::TAKE_A_PICTURE:
    case CommandID::FLIP_DRONE:
    case CommandID::THROW_AND_FLY:
    case CommandID::PALM_LAND:
    case CommandID::SET_LOW_BATTERY_WARNING:
    case CommandID::DRONE_LOG_DATA:
    case CommandID::SET_FLIGHT_HEIGHT_LIMIT:
        break;
    case CommandID::DRONE_LOG_HEADER: {
        assert(packet.data.size() >= 3);
        std::vector<u8> packet_bytes(3);
        packet_bytes[0] = 0x00;
        packet_bytes[1] = packet.data[0];
        packet_bytes[2] = packet.data[1];
        queue_packet(DronePacket(80, CommandID::DRONE_LOG_HEADER, std::move(packet_bytes)));
        break;
    }
    case CommandID::DRONE_LOG_CONFIGURATION: {
        assert(packet.data.size() >= 7);
        std::vector<u8> packet_bytes(7);
        packet_bytes[0] = 0x00;
        packet_bytes[1] = packet.data[1];
        packet_bytes[2] = packet.data[2];
        packet_bytes[3] = packet.data[3];
        packet_bytes[4] = packet.data[4];
        packet_bytes[5] = packet.data[5];
        packet_bytes[6] = packet.data[6];
        queue_packet(DronePacket(80, CommandID::DRONE_LOG_CONFIGURATION, std::move(packet_bytes)));
        break;
    }
    case CommandID::GET_CURRENT_TIME: {
        std::vector<u8> packet_bytes(14);
        auto current_time_point = std::chrono::system_clock::now();
        auto days = std::chrono::floor<std::chrono::days>(current_time_point);
        auto date = std::chrono::year_month_day(days);
        auto time = std::chrono::hh_mm_ss(
            std::chrono::floor<std::chrono::milliseconds>(current_time_point - days));
        auto year = static_cast<i32>(date.year());
        auto month = static_cast<u32>(date.month());
        auto day = static_cast<u32>(date.day());
        auto hours = time.hours().count();
        auto minutes = time.minutes().count();
        auto seconds = time.seconds().count();
        auto milliseconds = time.subseconds().count();
        packet_bytes[0] = year & 0xFF;
        packet_bytes[1] = (year >> 8) & 0xFF;
        packet_bytes[2] = month & 0xFF;
        packet_bytes[3] = (month >> 8) & 0xFF;
        packet_bytes[4] = day & 0xFF;
        packet_bytes[5] = (day >> 8) & 0xFF;
        packet_bytes[6] = hours & 0xFF;
        packet_bytes[7] = (hours >> 8) & 0xFF;
        packet_bytes[8] = minutes & 0xFF;
        packet_bytes[9] = (minutes >> 8) & 0xFF;
        packet_bytes[10] = seconds & 0xFF;
        packet_bytes[11] = (seconds >> 8) & 0xFF;
        packet_bytes[12] = milliseconds & 0xFF;
        packet_bytes[13] = (milliseconds >> 8) & 0xFF;
        queue_packet(DronePacket(80, CommandID::GET_CURRENT_TIME, std::move(packet_bytes)));
        break;
    }
    case CommandID::GET_SSID: {
        if (success) {
            assert(packet.data.size() >= 2);
            auto raw_ssid = std::string(packet.data.begin() + 1, packet.data.end());
            trim(raw_ssid);
            m_drone_info.ssid = std::move(raw_ssid);
        } else {
            std::cerr << "GET_SSID failed" << std::endl;
        }
        break;
    }
    case CommandID::GET_FIRMWARE_VERSION: {
        if (success) {
            assert(packet.data.size() >= 11);
            m_drone_info.firmware_version = std::string(packet.data.begin() + 1, packet.data.begin() + 11);
        } else {
            std::cerr << "GET_FIRMWARE_VERSION failed" << std::endl;
        }
        break;
    }
    case CommandID::GET_LOADER_VERSION: {
        if (success) {
            assert(packet.data.size() >= 11);
            m_drone_info.loader_version = std::string(packet.data.begin() + 1, packet.data.begin() + 11);
        } else {
            std::cerr << "GET_LOADER_VERSION failed" << std::endl;
        }
        break;
    }
    case CommandID::GET_BITRATE: {
        if (success) {
            assert(packet.data.size() >= 2);
            m_drone_info.bitrate = packet.data[1];
        } else {
            std::cerr << "GET_BITRATE failed" << std::endl;
        }
        break;
    }
    case CommandID::GET_FLIGHT_HEIGHT_LIMIT: {
        if (success) {
            assert(packet.data.size() >= 3);
            m_drone_info.flight_height_limit = packet.data[1] | ((u16)packet.data[2] << 8);
        } else {
            std::cerr << "GET_FLIGHT_HEIGHT_LIMIT failed" << std::endl;
        }
    }
    case CommandID::GET_LOW_BATTERY_WARNING: {
        if (success) {
            assert(packet.data.size() >= 3);
            m_drone_info.low_battery_warning = packet.data[1] | ((u16)packet.data[2] << 8);
        } else {
            std::cerr << "GET_LOW_BATTERY_WARNING failed" << std::endl;
        }
        break;
    }
    case CommandID::GET_ATTITUDE_ANGLE: {
        if (success) {
            assert(packet.data.size() >= 5);
            u32 float_bytes = packet.data[1] | ((u32)packet.data[2] << 8) | ((u32)packet.data[3] << 16) | ((u32)packet.data[4] << 24);
            m_drone_info.attitude_angle = *reinterpret_cast<float*>(&float_bytes);
        } else {
            std::cerr << "GET_ATTITUDE_ANGLE failed" << std::endl;
        }
        break;
    }
    case CommandID::GET_COUNTRY_CODE: {
        if (success) {
            assert(packet.data.size() >= 3);
            m_drone_info.country_code = std::string(packet.data.begin() + 1, packet.data.begin() + 3);
        } else {
            std::cerr << "GET_COUNTRY_CODE failed" << std::endl;
        }
        break;
    }
    case CommandID::GET_ACTIVATION_DATA: {
        if (success) {
            assert(packet.data.size() >= 58);
            // FIXME: Parse DATA
        } else {
            std::cerr << "GET_ACTIVATION_DATA failed" << std::endl;
        }
        break;
    }
    case CommandID::GET_UNIQUE_IDENTIFIER: {
        if (success) {
            assert(packet.data.size() >= 17);
            std::stringstream stream;
            for (size_t i = 0; i < 16; ++i)
                stream << std::hex << packet.data[i + 1];
            m_drone_info.unique_identifier = stream.str();
        } else {
            std::cerr << "GET_UNIQUE_IDENTIFIER failed" << std::endl;
        }
        break;
    }
    case CommandID::GET_ACTIVATION_STATUS: {
        m_drone_info.activation_status = success;
        break;
    }
    case CommandID::WIFI_STATE: {
        m_drone_info.wifi_strength = packet.data[0];
        m_drone_info.wifi_disturb = packet.data[1];
        break;
    }
    case CommandID::LIGHT_STRENGTH: {
        m_drone_info.light_strength = packet.data[0];
        break;
    }
    default:
        if constexpr (DRONE_DEBUG_LOGGING)
            std::cerr << "Unhandled packet with cmd_id=" << static_cast<u16>(packet.cmd_id) << std::endl;
        break;
    }

    std::unique_lock<std::mutex> lock(m_received_acks_mutex);
    m_received_acks[packet.seq_num] = true;
    lock.unlock();
    if constexpr (VERBOSE_DRONE_DEBUG_LOGGING)
        std::cout << "Received ack for packet " << packet.seq_num << std::endl;
    m_received_acks_cv.notify_all();
}

std::string Drone::get_ssid()
{
    if (!m_drone_info.ssid.has_value()) [[unlikely]]
        send_packet_and_assert_ack(DronePacket(72, CommandID::GET_SSID));
    return *m_drone_info.ssid;
}

std::string Drone::get_firmware_version()
{
    if (!m_drone_info.firmware_version.has_value()) [[unlikely]]
        send_packet_and_assert_ack(DronePacket(72, CommandID::GET_FIRMWARE_VERSION));
    return *m_drone_info.firmware_version;
}

std::string Drone::get_loader_version()
{
    if (!m_drone_info.loader_version.has_value()) [[unlikely]]
        send_packet_and_assert_ack(DronePacket(72, CommandID::GET_LOADER_VERSION));
    return *m_drone_info.loader_version;
}

u8 Drone::get_bitrate()
{
    if (!m_drone_info.bitrate.has_value()) [[unlikely]]
        send_packet_and_assert_ack(DronePacket(72, CommandID::GET_BITRATE));
    return *m_drone_info.bitrate;
}

u16 Drone::get_flight_height_limit()
{
    if (!m_drone_info.flight_height_limit.has_value()) [[unlikely]]
        send_packet_and_assert_ack(DronePacket(72, CommandID::GET_FLIGHT_HEIGHT_LIMIT));
    return *m_drone_info.flight_height_limit;
}

u16 Drone::get_low_battery_warning()
{
    if (!m_drone_info.low_battery_warning.has_value()) [[unlikely]]
        send_packet_and_assert_ack(DronePacket(72, CommandID::GET_LOW_BATTERY_WARNING));
    return *m_drone_info.low_battery_warning;
}

float Drone::get_attitude_angle()
{
    if (!m_drone_info.attitude_angle.has_value()) [[unlikely]]
        send_packet_and_assert_ack(DronePacket(72, CommandID::GET_ATTITUDE_ANGLE));
    return *m_drone_info.attitude_angle;
}

std::string Drone::get_country_code()
{
    if (!m_drone_info.country_code.has_value()) [[unlikely]]
        send_packet_and_assert_ack(DronePacket(72, CommandID::GET_COUNTRY_CODE));
    return *m_drone_info.country_code;
}

std::string Drone::get_unique_identifier()
{
    if (!m_drone_info.unique_identifier.has_value()) [[unlikely]]
        send_packet_and_assert_ack(DronePacket(72, CommandID::GET_UNIQUE_IDENTIFIER));
    return *m_drone_info.unique_identifier;
}

bool Drone::get_activation_status()
{
    if (!m_drone_info.activation_status.has_value()) [[unlikely]]
        send_packet_and_assert_ack(DronePacket(72, CommandID::GET_ACTIVATION_STATUS));
    return *m_drone_info.activation_status;
}

void Drone::set_flight_height_limit(u16 flight_height_limit)
{
    send_packet_and_assert_ack(DronePacket(72, CommandID::SET_FLIGHT_HEIGHT_LIMIT, { static_cast<u8>(flight_height_limit & 0xFF), static_cast<u8>(flight_height_limit >> 8) }));
}

bool Drone::take_off()
{
    return send_packet_and_wait_until_ack(DronePacket(104, CommandID::TAKE_OFF));
}

bool Drone::land()
{
    return send_packet_and_wait_until_ack(DronePacket(104, CommandID::LAND_DRONE, { 0x00 }));
}

void Drone::shutdown()
{
    queue_packet(DronePacket(80, CommandID::SHUTDOWN_DRONE, { 0, 0 }));
}

static inline u16 float_to_tello(float value) {
    assert(value <= 1 && value >= -1);
    return 1024 + (u16)(value * 660);
}

void Drone::set_joysticks_state(float right_stick_x, float right_stick_y, float left_stick_x, float left_stick_y)
{
    m_right_stick_x = float_to_tello(right_stick_x);
    m_right_stick_y = float_to_tello(right_stick_y);
    m_left_stick_x = float_to_tello(left_stick_x);
    m_left_stick_y = float_to_tello(left_stick_y);
}

}
