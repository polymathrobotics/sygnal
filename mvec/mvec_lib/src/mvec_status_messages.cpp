#include "mvec_lib/mvec_status_messages.hpp"

namespace polymath::sygnal
{

MvecFuseStatusMessage::MvecFuseStatusMessage() : is_valid_(false), timestamp_(std::chrono::steady_clock::now())
{
    fuse_statuses_.fill(MvecFuseStatus::NOT_USED);
}

bool MvecFuseStatusMessage::parse(const socketcan::CanFrame & frame)
{
    if (frame.get_id_type() != socketcan::IdType::EXTENDED ||
        frame.get_frame_type() != socketcan::FrameType::DATA) {
        is_valid_ = false;
        return false;
    }

    std::array<unsigned char, CAN_MAX_DLC> raw_data = frame.get_data();
    
    constexpr uint8_t len_data_bits = MvecHardware::MAX_FUSES * 2;
    
    auto data = unpackData<uint64_t>(raw_data, 1 * sizeof(unsigned char), len_data_bits);

    for (size_t i = 0; i < MvecHardware::MAX_FUSES; i++) {
        uint8_t raw_status = (data >> (i * 2)) & 0x03;
        fuse_statuses_[i] = static_cast<MvecFuseStatus>(raw_status);
    }

    is_valid_ = true;
    timestamp_ = std::chrono::steady_clock::now();
    return true;
}

MvecFuseStatus MvecFuseStatusMessage::get_fuse_status(uint8_t fuse_id) const
{
    if (fuse_id >= MvecHardware::MAX_FUSES) {
        return MvecFuseStatus::NOT_USED;
    }
    return fuse_statuses_[fuse_id];
}

MvecRelayStatusMessage::MvecRelayStatusMessage() : is_valid_(false), timestamp_(std::chrono::steady_clock::now())
{
    relay_statuses_.fill(MvecRelayStatus::RELAY_LOCATION_NOT_USED);
}

bool MvecRelayStatusMessage::parse(const socketcan::CanFrame & frame)
{
    if (frame.get_id_type() != socketcan::IdType::EXTENDED ||
        frame.get_frame_type() != socketcan::FrameType::DATA) {
        is_valid_ = false;
        return false;
    }

    std::array<unsigned char, CAN_MAX_DLC> raw_data = frame.get_data();
    
    constexpr uint8_t len_data_bits = MvecHardware::MAX_RELAYS * 4;
    
    auto data = unpackData<uint64_t>(raw_data, 1 * sizeof(unsigned char), len_data_bits);

    for (size_t i = 0; i < MvecHardware::MAX_RELAYS; i++) {
        uint8_t raw_status = (data >> (i * 4)) & 0x0F;
        relay_statuses_[i] = static_cast<MvecRelayStatus>(raw_status);
    }

    is_valid_ = true;
    timestamp_ = std::chrono::steady_clock::now();
    return true;
}

MvecRelayStatus MvecRelayStatusMessage::get_relay_status(uint8_t relay_id) const
{
    if (relay_id >= MvecHardware::MAX_RELAYS) {
        return MvecRelayStatus::RELAY_LOCATION_NOT_USED;
    }
    return relay_statuses_[relay_id];
}

MvecErrorStatusMessage::MvecErrorStatusMessage() : grid_address_(0), error_bits_(0), is_valid_(false), timestamp_(std::chrono::steady_clock::now())
{
}

bool MvecErrorStatusMessage::parse(const socketcan::CanFrame & frame)
{
    if (frame.get_id_type() != socketcan::IdType::EXTENDED ||
        frame.get_frame_type() != socketcan::FrameType::DATA) {
        is_valid_ = false;
        return false;
    }

    std::array<unsigned char, CAN_MAX_DLC> raw_data = frame.get_data();
    
    // Byte 0 is grid address
    grid_address_ = raw_data[0];
    
    // Extract 13 error bits from bytes 1-2
    error_bits_ = unpackData<uint16_t>(raw_data, 1 * sizeof(unsigned char), MvecHardware::NUM_ERROR_BITS);

    is_valid_ = true;
    timestamp_ = std::chrono::steady_clock::now();
    return true;
}

bool MvecErrorStatusMessage::has_error(MvecErrorType error_type) const
{
    return (error_bits_ & static_cast<uint16_t>(error_type)) != 0;
}

} // polymath::sygnal