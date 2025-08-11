#include <catch2/catch.hpp>
#include "mvec_lib/mvec_relay.hpp"
#include "mvec_lib/mvec_constants.hpp"
#include "socketcan_adapter/can_frame.hpp"

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

TEST_CASE("MvecRelay constructor initializes correctly", "[mvec_relay]")
{
    auto relay = std::make_unique<MvecRelay>();
    
    REQUIRE_FALSE(relay->get_relay_state(0));
    REQUIRE_FALSE(relay->is_relay_populated(0));
    REQUIRE_FALSE(relay->is_fuse_populated(0));
    
    // Test that status messages are initialized
    REQUIRE_FALSE(relay->get_fuse_status_message().is_valid());
    REQUIRE_FALSE(relay->get_relay_status_message().is_valid());
    REQUIRE_FALSE(relay->get_error_status_message().is_valid());
    
    // Test enum methods
    REQUIRE(relay->get_fuse_status(0) == MvecFuseStatus::NOT_USED);
    REQUIRE(relay->get_relay_status(0) == MvecRelayStatus::RELAY_LOCATION_NOT_USED);
    REQUIRE(relay->get_error_bits() == 0);
}

TEST_CASE("MvecRelay set relay command with valid inputs", "[mvec_relay]")
{
    auto relay = std::make_unique<MvecRelay>();
    relay->set_relay_in_command(0, 1);
    
    auto command_frame = relay->getRelayCommandMessage();
    REQUIRE(command_frame.get_id_type() == socketcan::IdType::EXTENDED);
    
    auto data = command_frame.get_data();
    REQUIRE(data[0] == MvecMessageIds::RELAY_COMMAND_WITH_FEEDBACK);
    REQUIRE(data[1] == 0x00);
}

TEST_CASE("MvecRelay set relay command with invalid relay id", "[mvec_relay]")
{
    auto relay = std::make_unique<MvecRelay>();
    relay->set_relay_in_command(MvecHardware::MAX_RELAYS + 1, 1);
    
    auto command_frame = relay->getRelayCommandMessage();
    auto data = command_frame.get_data();
    REQUIRE(data[0] == MvecMessageIds::RELAY_COMMAND_WITH_FEEDBACK);
}

TEST_CASE("MvecRelay set relay command with invalid state", "[mvec_relay]")
{
    auto relay = std::make_unique<MvecRelay>();
    relay->set_relay_in_command(0, 2);
    
    auto command_frame = relay->getRelayCommandMessage();
    auto data = command_frame.get_data();
    REQUIRE(data[0] == MvecMessageIds::RELAY_COMMAND_WITH_FEEDBACK);
}

TEST_CASE("MvecRelay set high side output valid", "[mvec_relay]")
{
    auto relay = std::make_unique<MvecRelay>();
    relay->set_high_side_output_in_command(1);
    
    auto command_frame = relay->getRelayCommandMessage();
    REQUIRE(command_frame.get_id_type() == socketcan::IdType::EXTENDED);
}

TEST_CASE("MvecRelay set high side output invalid", "[mvec_relay]")
{
    auto relay = std::make_unique<MvecRelay>();
    relay->set_high_side_output_in_command(2);
    
    auto command_frame = relay->getRelayCommandMessage();
    REQUIRE(command_frame.get_id_type() == socketcan::IdType::EXTENDED);
}

TEST_CASE("MvecRelay get relay query message", "[mvec_relay]")
{
    auto relay = std::make_unique<MvecRelay>();
    auto query_frame = relay->getRelayQueryMessage();
    REQUIRE(query_frame.get_id_type() == socketcan::IdType::EXTENDED);
    
    auto data = query_frame.get_data();
    REQUIRE(data[0] == MvecMessageIds::RELAY_STATE_QUERY);
    REQUIRE(data[1] == 0x00);
}

TEST_CASE("MvecRelay parse message with non-extended id", "[mvec_relay]")
{
    auto relay = std::make_unique<MvecRelay>();
    socketcan::CanFrame frame;
    frame.set_can_id(0x123);
    frame.set_id_as_standard();
    
    REQUIRE_FALSE(relay->parseMessage(frame));
}

TEST_CASE("MvecRelay parse message with non-data frame", "[mvec_relay]")
{
    auto relay = std::make_unique<MvecRelay>();
    socketcan::CanFrame frame;
    frame.set_can_id(0x123);
    frame.set_id_as_extended();
    frame.set_frame_type(socketcan::FrameType::REMOTE);
    
    REQUIRE_FALSE(relay->parseMessage(frame));
}

TEST_CASE("MvecRelay parse fuse status message", "[mvec_relay]")
{
    auto relay = std::make_unique<MvecRelay>();
    J1939_ID fuse_status_id(6, 0, MvecProtocol::BROADCAST_PDU, 0x01 + 0xA0, 0xB0);
    std::vector<uint8_t> data = {0x00, 0x55, 0xAA, 0x33, 0xCC, 0x0F, 0xF0, 0x99};
    
    auto frame = createTestFrame(fuse_status_id.get_can_id(), data);
    REQUIRE(relay->parseMessage(frame));
    
    // Test that status message was updated
    REQUIRE(relay->get_fuse_status_message().is_valid());
    REQUIRE(relay->get_fuse_status(0) == MvecFuseStatus::BLOWN);
    REQUIRE(relay->get_fuse_status(1) == MvecFuseStatus::BLOWN);
    
    // Test direct status message access
    REQUIRE(relay->get_fuse_status_message().is_valid());
    REQUIRE(relay->get_fuse_status(0) == MvecFuseStatus::BLOWN);
}

TEST_CASE("MvecRelay parse relay status message", "[mvec_relay]")
{
    auto relay = std::make_unique<MvecRelay>();
    J1939_ID relay_status_id(6, 0, MvecProtocol::BROADCAST_PDU, 0x02 + 0xA0, 0xB0);
    std::vector<uint8_t> data = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77};
    
    auto frame = createTestFrame(relay_status_id.get_can_id(), data);
    REQUIRE(relay->parseMessage(frame));
    
    // Test that status message was updated
    REQUIRE(relay->get_relay_status_message().is_valid());
    REQUIRE(relay->get_relay_status(0) == MvecRelayStatus::COIL_OPEN);
    
    // Test direct status message access
    REQUIRE(relay->get_relay_status_message().is_valid());
    REQUIRE(relay->get_relay_status(0) == MvecRelayStatus::COIL_OPEN);
}

TEST_CASE("MvecRelay parse error status message", "[mvec_relay]")
{
    auto relay = std::make_unique<MvecRelay>();
    J1939_ID error_status_id(6, 0, MvecProtocol::BROADCAST_PDU, 0x03 + 0xA0, 0xB0);
    std::vector<uint8_t> data = {0xB0, 0xFF, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00};
    
    auto frame = createTestFrame(error_status_id.get_can_id(), data);
    REQUIRE(relay->parseMessage(frame));
    
    // Test that status message was updated
    REQUIRE(relay->get_error_status_message().is_valid());
    REQUIRE(relay->get_error_grid_address() == 0xB0);
    REQUIRE(relay->get_error_bits() > 0);
    
    // Test enum-based error checking
    REQUIRE(relay->has_error(MvecErrorType::INVALID_CONFIG));
    REQUIRE(relay->has_error(MvecErrorType::GRID_ID_CHANGED));
    
    // Test direct status message access
    REQUIRE(relay->get_error_status_message().is_valid());
    REQUIRE(relay->get_error_grid_address() == 0xB0);
}

TEST_CASE("MvecRelay parse specific response relay command reply", "[mvec_relay]")
{
    auto relay = std::make_unique<MvecRelay>();
    J1939_ID specific_response_id(6, 0, MvecProtocol::SPECIFIC_PDU, 0x00, 0xB0);
    std::vector<uint8_t> data = {MvecMessageIds::RESPONSE, 0x88, 0x01, 0x00, 0xFF, 0x0F, 0x00, 0x00};
    
    auto frame = createTestFrame(specific_response_id.get_can_id(), data);
    REQUIRE(relay->parseMessage(frame));
}

TEST_CASE("MvecRelay parse specific response population reply", "[mvec_relay]")
{
    auto relay = std::make_unique<MvecRelay>();
    J1939_ID specific_response_id(6, 0, MvecProtocol::SPECIFIC_PDU, 0x00, 0xB0);
    std::vector<uint8_t> data = {MvecMessageIds::POPULATION_RESPONSE, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x0F, 0x00};
    
    auto frame = createTestFrame(specific_response_id.get_can_id(), data);
    REQUIRE(relay->parseMessage(frame));
}

TEST_CASE("MvecRelay parse specific response relay query reply", "[mvec_relay]")
{
    auto relay = std::make_unique<MvecRelay>();
    J1939_ID specific_response_id(6, 0, MvecProtocol::SPECIFIC_PDU, 0x00, 0xB0);
    std::vector<uint8_t> data = {MvecMessageIds::RELAY_QUERY_RESPONSE, 0x00, 0xFF, 0x0F, 0xAA, 0x55, 0x00, 0x00};
    
    auto frame = createTestFrame(specific_response_id.get_can_id(), data);
    REQUIRE(relay->parseMessage(frame));
    
    for (int i = 0; i < MvecHardware::MAX_RELAYS; ++i) {
        REQUIRE(relay->get_relay_state(i));
    }
}

TEST_CASE("MvecRelay parse specific response unknown message id", "[mvec_relay]")
{
    auto relay = std::make_unique<MvecRelay>();
    J1939_ID specific_response_id(6, 0, MvecProtocol::SPECIFIC_PDU, 0x00, 0xB0);
    std::vector<uint8_t> data = {0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    
    auto frame = createTestFrame(specific_response_id.get_can_id(), data);
    REQUIRE_FALSE(relay->parseMessage(frame));
}

TEST_CASE("MvecRelay parse message with unknown id", "[mvec_relay]")
{
    auto relay = std::make_unique<MvecRelay>();
    J1939_ID unknown_id(6, 0, 0x00, 0x00, 0x00);
    std::vector<uint8_t> data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    
    auto frame = createTestFrame(unknown_id.get_can_id(), data);
    REQUIRE_FALSE(relay->parseMessage(frame));
}

TEST_CASE("MvecRelay boundary test max relays", "[mvec_relay]")
{
    auto relay = std::make_unique<MvecRelay>();
    for (uint8_t i = 0; i < MvecHardware::MAX_RELAYS; ++i) {
        relay->set_relay_in_command(i, 1);
        REQUIRE_FALSE(relay->get_relay_state(i));
        REQUIRE_FALSE(relay->is_relay_populated(i));
    }
}

TEST_CASE("MvecRelay boundary test max fuses", "[mvec_relay]")
{
    auto relay = std::make_unique<MvecRelay>();
    for (uint8_t i = 0; i < MvecHardware::MAX_FUSES; ++i) {
        REQUIRE(relay->get_fuse_status(i) == MvecFuseStatus::NOT_USED);
        REQUIRE_FALSE(relay->is_fuse_populated(i));
    }
}