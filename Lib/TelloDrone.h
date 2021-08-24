#pragma once

#include <atomic>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <mutex>
#include <set>
#include <thread>
#include <condition_variable>
#include <chrono>
#include "Utils/Types.h"
#include "DronePacket.h"

static constexpr bool DEBUG_LOGGING = false;

class TelloDrone {
public:
    TelloDrone();
    ~TelloDrone();

	void shutdown();

    [[nodiscard]] bool is_connected();
    void wait_until_connected();


private:
    void queue_packet(DronePacket packet);
    void send_packet_and_wait_until_ack(const DronePacket& packet);

    void handle_packet(const DronePacket& packet);

    void cmd_receive_thread_routine();
    void video_receive_thread_routine();

	std::thread m_cmd_send_thread;
    int m_cmd_socket_fd;
    sockaddr_in m_cmd_addr {};

	std::atomic<u16> m_cmd_seq_num;
	std::set<u16> m_received_acks;
	std::mutex m_received_acks_mutex;
	std::condition_variable m_received_acks_cv;

	std::thread m_video_receive_thread;
	int m_video_socket_fd;
    sockaddr_in m_video_addr {};

	struct {
		std::string ssid;
		std::string firmware_version;
		std::string loader_version;
		u8 bitrate { 0 };
		u16 flight_height_limit { 0 };
		u16 low_battery_warning { 0 };
		float attitude_angle { 0 };
		std::string country_code;
		struct {
			std::chrono::system_clock::time_point activation_time;
			u8 raw_serial_data[14];
			std::string serial_number;
			bool unknown_flag;
			std::chrono::system_clock::time_point unknown_time;
			u8 unknown_data[14];
		} activation_data;
		std::string unique_identifier;
		bool activation_status { false };
	} m_drone_info;

	std::chrono::system_clock::time_point m_last_update_time;
	bool m_connected { false };
	std::mutex m_connected_mutex;
	std::condition_variable m_connected_cv;

	bool m_shutting_down { false };
};
