#pragma once

#include "DroneData.h"
#include "DronePacket.h"
#include "Utils/Types.h"
#include <arpa/inet.h>
#include <atomic>
#include <bitset>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <netinet/in.h>
#include <sys/socket.h>
#include <thread>

namespace Tello {

#define DRONE_DEBUG_LOGGING 1
#define VERBOSE_DRONE_DEBUG_LOGGING 0
#define VIDEO_DEBUG_LOGGING 0
#define VERBOSE_VIDEO_DEBUG_LOGGING 0

class Drone {
public:
    Drone();
    ~Drone();

    [[nodiscard]] bool is_connected();
    void wait_until_connected();

    // Drone Info getters - BLOCKING
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

    // Drone info getters - NON-BLOCKING
    [[nodiscard]] const FlightData& get_flight_data();
    [[nodiscard]] const MVOData& get_mvo_data();
    [[nodiscard]] const IMUData& get_imu_data();

    // Drone info setters - BLOCKING
    void set_flight_height_limit(u16);
    void set_low_battery_warning(u16);

    // Actions - BLOCKING
    bool take_off();
    bool throw_take_off();
    bool land();
    bool palm_land();
    bool cancel_landing();
    bool start_bouncing();
    bool stop_bouncing();
    bool flip(FlipDirection);
    bool start_smart_video(SmartVideoAction);
    bool stop_smart_video(SmartVideoAction);

    // Actions - NON-BLOCKING
    void shutdown();
    void take_off_non_blocking();
    void land_non_blocking();
    void set_joysticks_state(float right_stick_x, float right_stick_y, float left_stick_x, float left_stick_y);
    void hover();
    void set_normal_speed();
    void set_fast_speed();
    // speed is between 0 and 1
    void forward(float speed);
    void backward(float speed);
    void left(float speed);
    void right(float speed);
    void up(float speed);
    void down(float speed);
    void clockwise(float speed);
    void counterclockwise(float speed);

private:
    void close();

    void send_setup_packet();
    void send_initialization_sequence();
    void send_timed_requests_if_needed();

    void queue_packet_internal(DronePacket& packet);
    void queue_packet(DronePacket packet) { queue_packet_internal(packet); }
    bool send_packet_and_wait_until_ack(DronePacket packet);
    void send_packet_and_assert_ack(DronePacket packet);

    void handle_packet(const DronePacket& packet);

    void decode_flight_data(const std::vector<u8>& data);
    void decode_log_data(const std::vector<u8>& data);

    void drone_controls_thread_routine();
    void cmd_receive_thread_routine();
    void video_receive_thread_routine();

    std::thread m_cmd_receive_thread;
    int m_cmd_socket_fd;
    sockaddr_in m_cmd_addr {};

    std::atomic<u16> m_cmd_seq_num { 1 };
    std::bitset<65536> m_received_acks;
    std::mutex m_received_acks_mutex;
    std::condition_variable m_received_acks_cv;

    std::thread m_video_receive_thread;
    int m_video_socket_fd;
    int m_ffmpeg_socket_fd;
    sockaddr_in m_ffmpeg_addr {};

    std::thread m_drone_controls_thread;

    // These may need locking...
    DroneInfo m_drone_info;
    FlightData m_flight_data;
    MVOData m_mvo_data;
    IMUData m_imu_data;

    std::chrono::system_clock::time_point m_last_update_time;
    bool m_connected { false };
    std::mutex m_connected_mutex;
    std::condition_variable m_connected_cv;
    u8 m_timed_request_ticks { 0 };

    std::mutex m_controls_mutex;
    u16 m_right_stick_x { 1024 };
    u16 m_right_stick_y { 1024 };
    u16 m_left_stick_x { 1024 };
    u16 m_left_stick_y { 1024 };
    bool m_quick_mode { false };

    bool m_shutting_down { false };
};

}
