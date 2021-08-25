#pragma once

#include "DroneData.h"
#include "DronePacket.h"
#include "Utils/Types.h"
#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <netinet/in.h>
#include <bitset>
#include <sys/socket.h>
#include <thread>

namespace Tello {

#define DRONE_DEBUG_LOGGING 1
#define VIDEO_DEBUG_LOGGING 0
#define VERBOSE_VIDEO_DEBUG_LOGGING 0

class Drone {
public:
    Drone();
    ~Drone();

    void shutdown();

    [[nodiscard]] bool is_connected();
    void wait_until_connected();

    // Drone Info getters
    [[nodiscard]] std::string get_ssid();
    [[nodiscard]] std::string get_firmware_version();
    [[nodiscard]] std::string get_loader_version();
    [[nodiscard]] u8 get_bitrate();
    [[nodiscard]] u16 get_flight_height_limit();
    [[nodiscard]] u16 get_low_battery_warning();
    [[nodiscard]] float get_attitude_angle();
    [[nodiscard]] std::string get_country_code();
    [[nodiscard]] std::string get_unique_identifier();
    [[nodiscard]] bool get_activation_status();

    // Actions
    void take_off();
    void land();

private:
    void send_setup_packet();
    void send_initialization_sequence();
    void send_timed_requests_if_needed();

    void queue_packet(DronePacket packet);
    void send_packet_and_wait_until_ack(const DronePacket& packet);

    void handle_packet(const DronePacket& packet);

    void drone_controls_thread_routine();
    void cmd_receive_thread_routine();
    void video_receive_thread_routine();

    std::thread m_cmd_receive_thread;
    int m_cmd_socket_fd;
    sockaddr_in m_cmd_addr {};

    std::atomic<u16> m_cmd_seq_num;
    std::bitset<65536> m_received_acks;
    std::mutex m_received_acks_mutex;
    std::condition_variable m_received_acks_cv;

    std::thread m_video_receive_thread;
    int m_video_socket_fd;
    int m_ffmpeg_socket_fd;
    sockaddr_in m_ffmpeg_addr {};

    std::thread m_drone_controls_thread;

    DroneInfo m_drone_info;

    std::chrono::system_clock::time_point m_last_update_time;
    bool m_connected { false };
    std::mutex m_connected_mutex;
    std::condition_variable m_connected_cv;
    u8 m_timed_request_ticks { 0 };

    bool m_shutting_down { false };
};

}
