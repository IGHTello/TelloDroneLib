#include "TelloDrone.h"
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <span>

static constexpr u16 TELLO_CMD_PORT = 8889;
static constexpr u16 TELLO_VIDEO_PORT = 7777;
static constexpr char const *TELLO_CMD_IP = "192.168.10.1";

TelloDrone::TelloDrone() : m_cmd_seq_num(1), m_shutting_down(false) {
	m_video_socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (m_video_socket_fd == -1) {
		perror("socket() -> m_video_socket_fd");
		exit(1);
	}

	m_video_addr.sin_family = AF_INET;
	m_video_addr.sin_port = htons(TELLO_VIDEO_PORT);
	m_video_addr.sin_addr.s_addr = INADDR_ANY;

	m_cmd_socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (m_cmd_socket_fd == -1) {
		perror("socket() -> m_cmd_socket_fd");
		exit(1);
	}

	m_cmd_addr.sin_family = AF_INET;
	m_cmd_addr.sin_port = htons(TELLO_CMD_PORT);
	m_cmd_addr.sin_addr.s_addr = inet_addr(TELLO_CMD_IP);

	struct timeval sock_timeout{};
	sock_timeout.tv_sec = 1;
	sock_timeout.tv_usec = 0;
	if (setsockopt(m_cmd_socket_fd, SOL_SOCKET, SO_RCVTIMEO, (char *) &sock_timeout, sizeof(sock_timeout)) < 0) {
		perror("setsockopt()");
		exit(1);
	}

	m_video_receive_thread = std::thread(&TelloDrone::video_receive_thread_routine, this);
	m_cmd_send_thread = std::thread(&TelloDrone::cmd_receive_thread_routine, this);
}

void TelloDrone::video_receive_thread_routine() {
	std::vector<u8> current_frame;
	usize current_frame_num = 0;
	isize last_segment_num_received = -1;
	bool discard_current_frame = false;

	socklen_t video_addr_size = sizeof(m_video_addr);
	u8 packet_buffer[4096];
	while (!m_shutting_down) {
		isize bytes_received = recvfrom(m_video_socket_fd, packet_buffer, sizeof(packet_buffer), 0,
										reinterpret_cast<sockaddr *>(&m_video_addr), &video_addr_size);
		if (bytes_received < 0) {
			if (errno != EAGAIN)
				std::cerr << "Failed to receive bytes from video socket, errno: " << strerror(errno) << std::endl;
			continue;
		}

		if (bytes_received < 2) {
			if constexpr (DEBUG_LOGGING)
				std::cerr << "Received invalid video packet, less than 2 bytes of data!" << std::endl;
			continue;
		}

		auto frame_num = packet_buffer[0];
		auto segment_num = packet_buffer[1] & 127;
		auto last_segment_in_frame = (packet_buffer[1] & 128) == 128;

		if constexpr (DEBUG_LOGGING)
			std::cout << "Got segment " << segment_num << " of frame " << frame_num << " (end=" << last_segment_in_frame
					  << "), last was " << last_segment_num_received << " of frame " << current_frame_num << std::endl;

		if (frame_num != current_frame_num) {
			if constexpr (DEBUG_LOGGING)
				std::cout << "Lost segments on frame boundary " << current_frame_num << ':' << frame_num << std::endl;
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
			if constexpr (DEBUG_LOGGING)
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
				// FIXME: do something with frame
			}

			current_frame.clear();
			current_frame_num = (current_frame_num + 1) & 255;
			last_segment_num_received = -1;
			discard_current_frame = false;
		}
	}
}

void TelloDrone::cmd_receive_thread_routine() {
	socklen_t cmd_addr_size = sizeof(m_cmd_addr);
	u8 packet_buffer[4096];
	while (!m_shutting_down) {
		isize bytes_received = recvfrom(m_cmd_socket_fd, packet_buffer, sizeof(packet_buffer), 0,
										reinterpret_cast<sockaddr *>(&m_cmd_addr), &cmd_addr_size);
		if (bytes_received < 0) {
			if (errno != EAGAIN)
				std::cerr << "Failed to receive bytes from cmd socket, errno: " << strerror(errno) << std::endl;
			continue;
		}

		auto packet = DronePacket::deserialize(std::span<u8>(packet_buffer, bytes_received), false);
		if (packet.has_value())
			handle_packet(packet.value());
		else if constexpr (DEBUG_LOGGING)
			std::cerr << "Failed to parse packet of length `" << bytes_received << "`" << std::endl;
	}
}

TelloDrone::~TelloDrone() {
	shutdown();
}

bool TelloDrone::is_connected() {
	std::unique_lock<std::mutex> lock(m_connected_mutex);
	return m_connected;
}

void TelloDrone::wait_until_connected() {
	std::unique_lock<std::mutex> lock(m_connected_mutex);
	while (!m_connected)
		m_connected_cv.wait(lock);
}

void TelloDrone::shutdown() {
	if (m_shutting_down)
		return;
	m_shutting_down = true;
	// FIXME: LAND
	m_video_receive_thread.join();
	close(m_video_socket_fd);
	m_cmd_send_thread.join();
	close(m_cmd_socket_fd);
}

void TelloDrone::queue_packet(DronePacket packet) {
	auto packet_bytes = packet.serialize();
	sendto(m_cmd_socket_fd, packet_bytes.data(), packet_bytes.size(), 0,
		   reinterpret_cast<const sockaddr *>(&m_cmd_addr), sizeof(m_cmd_addr));
}

void TelloDrone::send_packet_and_wait_until_ack(const DronePacket &packet) {
	queue_packet(packet);
	std::unique_lock<std::mutex> lock(m_received_acks_mutex);
	while (!m_received_acks.contains(packet.seq_num))
		m_received_acks_cv.wait(lock);
	m_received_acks.erase(packet.seq_num);
}

void TelloDrone::handle_packet(const DronePacket &packet) {
	bool success;
	if (!packet.data.empty()) {
		success = packet.data[0] == 0;
	} else {
		success = false;
	}

	auto add_packet_ack = [&]() {
		std::unique_lock<std::mutex> lock(m_received_acks_mutex);
		m_received_acks.insert(packet.seq_num);
		m_received_acks_cv.notify_all();
	};

	switch (packet.cmd_id) {
		case DronePacket::CommandID::FLIGHT_DATA: {
			auto time_since_last_update = std::chrono::duration_cast<std::chrono::milliseconds>(
					std::chrono::system_clock::now() - m_last_update_time);
			if (time_since_last_update.count() > 3000) {
				std::unique_lock<std::mutex> lock(m_connected_mutex);
				m_connected = false;
			} else {
				std::unique_lock<std::mutex> lock(m_connected_mutex);
				m_connected = true;
				lock.unlock();
				m_connected_cv.notify_all();
				// FIXME: do connection stuff
			}
			break;
		}
		case DronePacket::CommandID::SET_SSID:
		case DronePacket::CommandID::SET_COUNTRY_CODE:
		case DronePacket::CommandID::SET_WIFI_PASSWORD:
		case DronePacket::CommandID::SET_ATTITUDE_ANGLE:
		case DronePacket::CommandID::ACTIVATE_DRONE:
		case DronePacket::CommandID::SET_BITRATE:
		case DronePacket::CommandID::SET_EIS:
		case DronePacket::CommandID::SET_AUTOMATIC_BITRATE:
		case DronePacket::CommandID::SET_RECORDING_MAYBE:
		case DronePacket::CommandID::SET_CAMERA_EV:
		case DronePacket::CommandID::SET_PHOTO_QUALITY:
		case DronePacket::CommandID::SET_CAMERA_MODE:
		case DronePacket::CommandID::LAND_DRONE:
		case DronePacket::CommandID::TAKE_OFF:
		case DronePacket::CommandID::TAKE_A_PICTURE:
		case DronePacket::CommandID::FLIP_DRONE:
		case DronePacket::CommandID::THROW_AND_FLY:
		case DronePacket::CommandID::PALM_LAND:
		case DronePacket::CommandID::SET_LOW_BATTERY_WARNING:
		case DronePacket::CommandID::CONN_ACK:
			add_packet_ack();
			break;
		default:
			if constexpr (DEBUG_LOGGING)
				std::cerr << "Unhandled packet with cmd_id=" << static_cast<u16>(packet.cmd_id) << std::endl;
			break;
	}
}
