#include <catch2/catch.hpp>
#include "mvec_lib/mvec_constants.hpp"
#include "mvec_lib/mvec_status_messages.hpp"

using namespace polymath::sygnal;

static socketcan::CanFrame createTestFrame(uint32_t can_id, const std::vector<uint8_t>& data)
{
    socketcan::CanFrame frame;
    frame.set_can_id(can_id);
    frame.set_id_as_extended();
    std::array<unsigned char, CAN_MAX_DLC> frame_data;
    frame_data.fill(0);
    for (size_t i = 0; i < data.size() && i < CAN_MAX_DLC; ++i) {
        frame_data[i] = data[i];
    }
    frame.set_data(frame_data);
    return frame;
}

TEST_CASE("FuseStatus enum values", "[status_enums]")
{
    REQUIRE(static_cast<uint8_t>(MvecFuseStatus::NO_FAULT) == 0x0);
    REQUIRE(static_cast<uint8_t>(MvecFuseStatus::BLOWN) == 0x1);
    REQUIRE(static_cast<uint8_t>(MvecFuseStatus::NOT_POWERED) == 0x2);
    REQUIRE(static_cast<uint8_t>(MvecFuseStatus::NOT_USED) == 0x3);
}

TEST_CASE("RelayStatus enum values", "[status_enums]")
{
    REQUIRE(static_cast<uint8_t>(MvecRelayStatus::OKAY) == 0x00);
    REQUIRE(static_cast<uint8_t>(MvecRelayStatus::COIL_OPEN) == 0x01);
    REQUIRE(static_cast<uint8_t>(MvecRelayStatus::COIL_SHORTED_OR_DRIVER_FAILED) == 0x02);
    REQUIRE(static_cast<uint8_t>(MvecRelayStatus::NO_CONTACT_OPEN) == 0x03);
    REQUIRE(static_cast<uint8_t>(MvecRelayStatus::NC_CONTACT_OPEN) == 0x04);
    REQUIRE(static_cast<uint8_t>(MvecRelayStatus::COIL_NOT_RECEIVING_POWER) == 0x05);
    REQUIRE(static_cast<uint8_t>(MvecRelayStatus::NO_CONTACT_SHORTED) == 0x06);
    REQUIRE(static_cast<uint8_t>(MvecRelayStatus::NC_CONTACT_SHORTED) == 0x07);
    REQUIRE(static_cast<uint8_t>(MvecRelayStatus::HIGH_SIDE_DRIVER_FAULT) == 0x0B);
    REQUIRE(static_cast<uint8_t>(MvecRelayStatus::HIGH_SIDE_OPEN_LOAD) == 0x0C);
    REQUIRE(static_cast<uint8_t>(MvecRelayStatus::HIGH_SIDE_OVER_VOLTAGE) == 0x0D);
    REQUIRE(static_cast<uint8_t>(MvecRelayStatus::RELAY_LOCATION_NOT_USED) == 0x0F);
}

TEST_CASE("ErrorStatus bit masks", "[status_enums]")
{
    REQUIRE(MvecErrorBits::INVALID_CONFIG == 0x0001);
    REQUIRE(MvecErrorBits::GRID_ID_CHANGED == 0x0002);
    REQUIRE(MvecErrorBits::CAN_ADDRESS_CHANGED == 0x0004);
    REQUIRE(MvecErrorBits::CAN_RX_COMM_ERROR == 0x0008);
    REQUIRE(MvecErrorBits::CAN_TX_COMM_ERROR == 0x0010);
    REQUIRE(MvecErrorBits::UNEXPECTED_RESET == 0x0020);
    REQUIRE(MvecErrorBits::OVER_VOLTAGE == 0x0040);
    REQUIRE(MvecErrorBits::SPI_ERROR == 0x0080);
    REQUIRE(MvecErrorBits::SHORT_MESSAGE_RECEIVED == 0x0100);
    REQUIRE(MvecErrorBits::BAD_FLASH_ADDRESS == 0x0200);
    REQUIRE(MvecErrorBits::INVALID_LENGTH == 0x0400);
    REQUIRE(MvecErrorBits::CHECKSUM_FAILURE == 0x0800);
    REQUIRE(MvecErrorBits::FLASH_MISCOMPARE == 0x1000);
}

TEST_CASE("FuseStatusMessage with enums", "[status_enums]")
{
    MvecFuseStatusMessage msg;
    
    // Create test data where first fuse is blown (0x1), second is not powered (0x2)
    std::vector<uint8_t> data = {0x00, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 0x09 = 0b00001001 = fuse0=1, fuse1=2
    
    auto frame = createTestFrame(0x18A01B0, data);
    REQUIRE(msg.parse(frame));
    REQUIRE(msg.is_valid());
    
    REQUIRE(msg.get_fuse_status(0) == MvecFuseStatus::BLOWN);
    REQUIRE(msg.get_fuse_status(1) == MvecFuseStatus::NOT_POWERED);
    REQUIRE(msg.get_fuse_status(2) == MvecFuseStatus::NO_FAULT);
}

TEST_CASE("RelayStatusMessage with enums", "[status_enums]")
{
    MvecRelayStatusMessage msg;
    
    // Create test data where first relay is okay (0x0), second has coil open (0x1)
    std::vector<uint8_t> data = {0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 0x10 = relay0=0, relay1=1
    
    auto frame = createTestFrame(0x18A02B0, data);
    REQUIRE(msg.parse(frame));
    REQUIRE(msg.is_valid());
    
    REQUIRE(msg.get_relay_status(0) == MvecRelayStatus::OKAY);
    REQUIRE(msg.get_relay_status(1) == MvecRelayStatus::COIL_OPEN);
}

TEST_CASE("ErrorStatusMessage with enums", "[status_enums]")
{
    MvecErrorStatusMessage msg;
    
    // Grid address 0x42, error bits with INVALID_CONFIG (0x01) and GRID_ID_CHANGED (0x02)
    std::vector<uint8_t> data = {0x42, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 0x03 = 0b00000011
    
    auto frame = createTestFrame(0x18A03B0, data);
    REQUIRE(msg.parse(frame));
    REQUIRE(msg.is_valid());
    
    REQUIRE(msg.get_grid_address() == 0x42);
    REQUIRE(msg.get_error_bits() == 0x03);
    
    REQUIRE(msg.has_error(MvecErrorType::INVALID_CONFIG));
    REQUIRE(msg.has_error(MvecErrorType::GRID_ID_CHANGED));
    REQUIRE_FALSE(msg.has_error(MvecErrorType::CAN_ADDRESS_CHANGED));
}

TEST_CASE("Status data structures", "[status_enums]")
{
    MvecFuseStatusMessage fuse_msg;
    MvecRelayStatusMessage relay_msg;
    MvecErrorStatusMessage error_msg;
    
    std::vector<uint8_t> fuse_data = {0x00, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    std::vector<uint8_t> relay_data = {0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    std::vector<uint8_t> error_data = {0x42, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    
    auto fuse_frame = createTestFrame(0x18A01B0, fuse_data);
    auto relay_frame = createTestFrame(0x18A02B0, relay_data);
    auto error_frame = createTestFrame(0x18A03B0, error_data);
    
    REQUIRE(fuse_msg.parse(fuse_frame));
    REQUIRE(relay_msg.parse(relay_frame));
    REQUIRE(error_msg.parse(error_frame));
    
    // Test direct status message access
    REQUIRE(fuse_msg.is_valid());
    REQUIRE(relay_msg.is_valid());
    REQUIRE(error_msg.is_valid());
    
    REQUIRE(fuse_msg.get_fuse_status(0) == MvecFuseStatus::BLOWN);
    REQUIRE(relay_msg.get_relay_status(1) == MvecRelayStatus::COIL_OPEN);
    REQUIRE(error_msg.get_grid_address() == 0x42);
    REQUIRE(error_msg.get_error_bits() == 0x03);
}