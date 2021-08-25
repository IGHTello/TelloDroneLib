#include "DronePacket.h"
#include "Utils/CRCHelpers.h"
#include <cstring>

static constexpr usize MINIMUM_PACKET_LENGTH = 11;

std::vector<u8> DronePacket::serialize() {
    std::vector<u8> packet_bytes(MINIMUM_PACKET_LENGTH + data.size());
    if(cmd_id == CommandID::CONN_REQ) {
        const char* header = "conn_req:";
        packet_bytes.assign(header, header + 9);
        packet_bytes.push_back(data[0]);
        packet_bytes.push_back(data[1]);
        return packet_bytes;
    }

    packet_bytes.resize(MINIMUM_PACKET_LENGTH + data.size());
    packet_bytes[0] = 0xCC;
    u16 packet_length = (MINIMUM_PACKET_LENGTH + data.size()) << 3;
    packet_bytes[1] = packet_length & 0xFF;
    packet_bytes[2] = packet_length >> 8;
    packet_bytes[3] = fast_crc8({ packet_bytes.begin(), 3 });
    packet_bytes[4] = packet_type;
    packet_bytes[5] = static_cast<u16>(cmd_id) & 0xFF;
    packet_bytes[6] = static_cast<u16>(cmd_id) >> 8;
    packet_bytes[7] = seq_num & 0xFF;
    packet_bytes[8] = seq_num >> 8;

    if (!data.empty()) {
        packet_bytes.insert(packet_bytes.begin() + 8, data.cbegin(), data.cend());
    }

    usize crc_off = 9 + data.size();
    u16 packet_crc = fast_crc16(packet_bytes);
    packet_bytes[crc_off] = packet_crc & 0xFF;
    packet_bytes[crc_off + 1] = packet_crc >> 8;

    return packet_bytes;
}

std::optional<DronePacket> DronePacket::deserialize(std::span<u8> packet_bytes) {
    if(packet_bytes.size() < MINIMUM_PACKET_LENGTH)
        return {};

    if(memcmp(packet_bytes.data(), "conn_ack:", 9) == 0) {
        auto packet_data = std::vector<u8>(packet_bytes.begin() + 9, packet_bytes.end());
        return DronePacket(0, 0, CommandID::CONN_ACK, std::move(packet_data));
    }

    if(packet_bytes[0] != 0xCC)
        return {};

    u16 packet_length = ((packet_bytes[2] << 8) | packet_bytes[1]) >> 3;
    u16 data_length = packet_length - MINIMUM_PACKET_LENGTH;
    if(packet_bytes.size() < packet_length || packet_length < MINIMUM_PACKET_LENGTH)
        return {};

    if(packet_bytes[3] != fast_crc8(std::span<u8>(packet_bytes).subspan(0, 3)))
        return {};

    u16 packet_checksum = (static_cast<u16>(packet_bytes[packet_length - 1]) << 8) | packet_bytes[packet_length - 2];
    if(packet_checksum != fast_crc16(std::span<u8>(packet_bytes).subspan(0, packet_length - 2)))
        return {};

    u8 packet_type = packet_bytes[4];
    u16 cmd_id = (static_cast<u16>(packet_bytes[6]) << 8) | packet_bytes[5];
    u16 seq_num = (static_cast<u16>(packet_bytes[8]) << 8) | packet_bytes[7];
    std::vector<u8> data(packet_bytes.begin() + 9, packet_bytes.begin() + 9 + data_length);

    return DronePacket(seq_num, packet_type, static_cast<CommandID>(cmd_id), std::move(data));
}
