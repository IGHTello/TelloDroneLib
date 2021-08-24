#include "TelloDrone.h"
#include <iostream>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <span>
#include <cassert>
#include <sstream>

static constexpr u16 TELLO_CMD_PORT = 8889;
static constexpr u16 TELLO_VIDEO_PORT = 7777;
static constexpr char const *TELLO_CMD_IP = "192.168.10.1";

TelloDrone::TelloDrone() : m_cmd_seq_num(1), m_shutting_down(false) {
	m_video_socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (m_video_socket_fd == -1) {
		perror("socket() -> m_video_socket_fd");
		exit(1);
	}
	sockaddr_in video_receive_addr {};
	video_receive_addr.sin_family = AF_INET;
	video_receive_addr.sin_addr.s_addr = INADDR_ANY;
	video_receive_addr.sin_port = htons(TELLO_VIDEO_PORT);
	if (bind(m_video_socket_fd, reinterpret_cast<sockaddr *>(&video_receive_addr), sizeof(sockaddr_in)) < 0) {
		perror("bind(m_video_socket_fd)");
		exit(1);
	}

	struct timeval sock_timeout{};
	sock_timeout.tv_sec = 1;
	sock_timeout.tv_usec = 0;
	if (setsockopt(m_video_socket_fd, SOL_SOCKET, SO_RCVTIMEO, (char *) &sock_timeout, sizeof(sock_timeout)) < 0) {
		perror("setsockopt()");
		exit(1);
	}

	m_cmd_socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (m_cmd_socket_fd == -1) {
		perror("socket() -> m_cmd_socket_fd");
		exit(1);
	}
	m_cmd_addr.sin_family = AF_INET;
	m_cmd_addr.sin_port = htons(TELLO_CMD_PORT);
	m_cmd_addr.sin_addr.s_addr = inet_addr(TELLO_CMD_IP);

	if (setsockopt(m_cmd_socket_fd, SOL_SOCKET, SO_RCVTIMEO, (char *) &sock_timeout, sizeof(sock_timeout)) < 0) {
		perror("setsockopt()");
		exit(1);
	}

	m_video_receive_thread = std::thread(&TelloDrone::video_receive_thread_routine, this);
	m_cmd_receive_thread = std::thread(&TelloDrone::cmd_receive_thread_routine, this);
	m_drone_controls_thread = std::thread(&TelloDrone::drone_controls_thread_routine, this);
}

void TelloDrone::video_receive_thread_routine() {
	std::vector<u8> current_frame;
	usize current_frame_num = 0;
	isize last_segment_num_received = -1;
	bool discard_current_frame = false;
	u8 full_frames_received = 0;

	u8 packet_buffer[4096];
	while (!m_shutting_down) {
		isize bytes_received = recvfrom(m_video_socket_fd, packet_buffer, sizeof(packet_buffer), 0, nullptr, nullptr);

		if (bytes_received < 0) {
			if (errno != EAGAIN)
				std::cerr << "Failed to receive bytes from video socket, errno: " << strerror(errno) << std::endl;
			queue_packet(DronePacket(DronePacket(96, CommandID::PRODUCE_VIDEO_I_FRAME_MAYBE))); // FIXME: ???
			continue;
		}

		if (bytes_received < 2) {
			if constexpr (DEBUG_LOGGING)
				std::cerr << "Received invalid video packet, less than 2 bytes of data!" << std::endl;
			continue;
		}

		int frame_num = packet_buffer[0];
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
				if constexpr (DEBUG_LOGGING)
					std::cout << "Finished receiving full frame" << std::endl;
				if (full_frames_received == 8) {
					queue_packet(DronePacket(DronePacket(96, CommandID::PRODUCE_VIDEO_I_FRAME_MAYBE)));
					full_frames_received = 0;
				}
				full_frames_received++;
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

		auto packet = DronePacket::deserialize(std::span<u8>(packet_buffer, bytes_received));
		if (packet.has_value())
			handle_packet(packet.value());
		else if constexpr (DEBUG_LOGGING)
			std::cerr << "Failed to parse packet of length `" << bytes_received << "`" << std::endl;
	}
}

void TelloDrone::send_initialization_sequence() {
	queue_packet(DronePacket(96, CommandID::PRODUCE_VIDEO_I_FRAME_MAYBE));
	queue_packet(DronePacket(72, CommandID::GET_FIRMWARE_VERSION));
	queue_packet(DronePacket(72, CommandID::GET_BITRATE));
	queue_packet(DronePacket(72, CommandID::GET_FLIGHT_HEIGHT_LIMIT));
	queue_packet(DronePacket(72, CommandID::GET_LOW_BATTERY_WARNING));
	queue_packet(DronePacket(72, CommandID::GET_ATTITUDE_ANGLE));
	queue_packet(DronePacket(72, CommandID::GET_COUNTRY_CODE));
	queue_packet(DronePacket(72, CommandID::SET_CAMERA_EV, {0x00}));
	queue_packet(DronePacket(72, CommandID::SET_PHOTO_QUALITY, {0x00}));
	queue_packet(DronePacket(72, CommandID::SET_BITRATE, {0x00}));
	queue_packet(DronePacket(104, CommandID::SET_RECORDING_MAYBE, {0x01}));
	queue_packet(DronePacket(72, CommandID::GET_SSID));
	queue_packet(DronePacket(72, CommandID::GET_LOADER_VERSION));
	queue_packet(DronePacket(72, CommandID::SET_CAMERA_MODE, {0x00}));
	queue_packet(DronePacket(72, CommandID::GET_ACTIVATION_DATA));
	queue_packet(DronePacket(72, CommandID::GET_ACTIVATION_STATUS));
}

void TelloDrone::send_setup_packet_if_needed() {
	if (m_connection_request_ticks >= 50 || m_connection_request_ticks == 0) {
		m_connection_request_ticks = 0;

		std::vector<u8> packet_bytes(2);
		packet_bytes[0] = TELLO_VIDEO_PORT & 0xFF;
		packet_bytes[1] = (TELLO_VIDEO_PORT >> 8) & 0xFF;
		queue_packet(DronePacket(0, CommandID::CONN_REQ, std::move(packet_bytes)));
	}
	m_connection_request_ticks++;
}

void TelloDrone::drone_controls_thread_routine() {
	while (!m_shutting_down) {
		std::this_thread::sleep_for(std::chrono::milliseconds(20));

		if (!m_connected)
			send_setup_packet_if_needed();

		std::vector<u8> packet_data(11);
		u64 joystick_unk1 = 1024;
		u64 joystick_unk2 = 1024;
		u64 joystick_unk3 = 1024;
		u64 joystick_unk4 = 1024;
		u64 flight_speed_quick = 0;
		u64 packed_drone_controls =
				(joystick_unk1 & 0x7FF) | ((joystick_unk2 & 0x7FF) << 11) | ((joystick_unk3 & 0x7FF) << 22) |
				((joystick_unk4 & 0x7FF) << 33) | (flight_speed_quick << 44);
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

	// FIXME: Send some kind of land packet?
	queue_packet(DronePacket(80, CommandID::SHUTDOWN_DRONE, {0, 0}));

	m_shutting_down = true;
	m_video_receive_thread.join();
	close(m_video_socket_fd);
	m_cmd_receive_thread.join();
	close(m_cmd_socket_fd);
	m_drone_controls_thread.join();
}

void TelloDrone::queue_packet(DronePacket packet) {
	assert(packet.direction == PacketDirection::TO_DRONE);
	packet.seq_num = m_cmd_seq_num++;
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
	assert(packet.direction == PacketDirection::FROM_DRONE);

	auto add_packet_ack = [&]() {
		std::unique_lock<std::mutex> lock(m_received_acks_mutex);
		m_received_acks.insert(packet.seq_num);
		m_received_acks_cv.notify_all();
	};

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
				m_connected = true;
				lock.unlock();

				send_initialization_sequence();

				m_connected_cv.notify_all();
			}
			m_last_update_time = current_time;
			// TODO: READ FLIGHT DATA
			break;
		}
		case CommandID::CONN_ACK: {
			if constexpr (DEBUG_LOGGING)
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
		case CommandID::SET_RECORDING_MAYBE:
		case CommandID::SET_CAMERA_EV:
		case CommandID::SET_PHOTO_QUALITY:
		case CommandID::SET_CAMERA_MODE:
		case CommandID::LAND_DRONE:
		case CommandID::TAKE_OFF:
		case CommandID::TAKE_A_PICTURE:
		case CommandID::FLIP_DRONE:
		case CommandID::THROW_AND_FLY:
		case CommandID::PALM_LAND:
		case CommandID::SET_LOW_BATTERY_WARNING: {
			add_packet_ack();
			break;
		}
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
			auto time = std::chrono::hh_mm_ss(std::chrono::floor<std::chrono::milliseconds>(current_time_point - days));
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
				m_drone_info.ssid = std::string(packet.data.begin() + 1, packet.data.end());
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
				m_drone_info.flight_height_limit = packet.data[1] | ((u16) packet.data[2] << 8);
			} else {
				std::cerr << "GET_FLIGHT_HEIGHT_LIMIT failed" << std::endl;
			}
		}
		case CommandID::GET_LOW_BATTERY_WARNING: {
			if (success) {
				assert(packet.data.size() >= 3);
				m_drone_info.low_battery_warning = packet.data[1] | ((u16) packet.data[2] << 8);
			} else {
				std::cerr << "GET_LOW_BATTERY_WARNING failed" << std::endl;
			}
			break;
		}
		case CommandID::GET_ATTITUDE_ANGLE: {
			if (success) {
				assert(packet.data.size() >= 5);
				u32 float_bytes = packet.data[1] | ((u32) packet.data[2] << 8) | ((u32) packet.data[3] << 16) |
								  ((u32) packet.data[4] << 24);
				m_drone_info.attitude_angle = *reinterpret_cast<float *>(&float_bytes);
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
				std::cout << "FIXME ACTIVATION DATA" << std::endl;
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
		default:
			if constexpr (DEBUG_LOGGING)
				std::cerr << "Unhandled packet with cmd_id=" << static_cast<u16>(packet.cmd_id) << std::endl;
			break;
	}
}
