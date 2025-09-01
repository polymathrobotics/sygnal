// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mvec_lib/mvec_relay.hpp"

#include <memory>
#include <vector>

#include <catch2/catch.hpp>

#include "mvec_lib/core/mvec_constants.hpp"
#include "socketcan_adapter/can_frame.hpp"

static polymath::socketcan::CanFrame createTestFrame(uint32_t can_id, const std::vector<uint8_t> & data)
{
  polymath::socketcan::CanFrame frame;
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
  auto relay = std::make_unique<polymath::sygnal::MvecRelay>();

  // Test that status messages are initialized
  REQUIRE_FALSE(relay->get_fuse_status_message().is_valid());
  REQUIRE_FALSE(relay->get_relay_status_message().is_valid());
  REQUIRE_FALSE(relay->get_error_status_message().is_valid());

  // Test accessing data through message objects
  REQUIRE(relay->get_fuse_status_message().get_fuse_status(0) == polymath::sygnal::MvecFuseStatus::NOT_USED);
  REQUIRE(
    relay->get_relay_status_message().get_relay_status(0) ==
    polymath::sygnal::MvecRelayStatus::RELAY_LOCATION_NOT_USED);
  REQUIRE(relay->get_error_status_message().get_error_bits() == 0);
}

TEST_CASE("MvecRelay set relay command with valid inputs", "[mvec_relay]")
{
  auto relay = std::make_unique<polymath::sygnal::MvecRelay>();
  relay->set_relay_in_command(0, 1);

  auto command_frame = relay->getRelayCommandMessage();
  REQUIRE(command_frame.get_id_type() == polymath::socketcan::IdType::EXTENDED);

  auto data = command_frame.get_data();
  REQUIRE(data[0] == polymath::sygnal::MvecCommandQueryIds::RELAY_COMMAND_WITH_FEEDBACK);
  REQUIRE(data[1] == 0x00);
}

TEST_CASE("MvecRelay set relay command with invalid relay id", "[mvec_relay]")
{
  auto relay = std::make_unique<polymath::sygnal::MvecRelay>();
  relay->set_relay_in_command(12 + 1, 1);

  auto command_frame = relay->getRelayCommandMessage();
  auto data = command_frame.get_data();
  REQUIRE(data[0] == polymath::sygnal::MvecCommandQueryIds::RELAY_COMMAND_WITH_FEEDBACK);
}

TEST_CASE("MvecRelay set relay command with invalid state", "[mvec_relay]")
{
  auto relay = std::make_unique<polymath::sygnal::MvecRelay>();
  relay->set_relay_in_command(0, 2);

  auto command_frame = relay->getRelayCommandMessage();
  auto data = command_frame.get_data();
  REQUIRE(data[0] == polymath::sygnal::MvecCommandQueryIds::RELAY_COMMAND_WITH_FEEDBACK);
}

TEST_CASE("MvecRelay set high side output valid", "[mvec_relay]")
{
  auto relay = std::make_unique<polymath::sygnal::MvecRelay>();
  relay->set_high_side_output_in_command(1);

  auto command_frame = relay->getRelayCommandMessage();
  REQUIRE(command_frame.get_id_type() == polymath::socketcan::IdType::EXTENDED);
}

TEST_CASE("MvecRelay set high side output invalid", "[mvec_relay]")
{
  auto relay = std::make_unique<polymath::sygnal::MvecRelay>();
  relay->set_high_side_output_in_command(2);

  auto command_frame = relay->getRelayCommandMessage();
  REQUIRE(command_frame.get_id_type() == polymath::socketcan::IdType::EXTENDED);
}

TEST_CASE("MvecRelay get relay query message", "[mvec_relay]")
{
  auto relay = std::make_unique<polymath::sygnal::MvecRelay>();
  auto query_frame = relay->getRelayQueryMessage();
  REQUIRE(query_frame.get_id_type() == polymath::socketcan::IdType::EXTENDED);

  auto data = query_frame.get_data();
  REQUIRE(data[0] == polymath::sygnal::MvecCommandQueryIds::RELAY_STATE_QUERY);
  REQUIRE(data[1] == 0x00);
}

TEST_CASE("MvecRelay parse message with non-extended id", "[mvec_relay]")
{
  auto relay = std::make_unique<polymath::sygnal::MvecRelay>();
  polymath::socketcan::CanFrame frame;
  frame.set_can_id(0x123);
  frame.set_id_as_standard();

  REQUIRE(relay->parseMessage(frame) == polymath::sygnal::MvecMessageType::UNSUPPORTED);
}

TEST_CASE("MvecRelay parse message with non-data frame", "[mvec_relay]")
{
  auto relay = std::make_unique<polymath::sygnal::MvecRelay>();
  polymath::socketcan::CanFrame frame;
  frame.set_can_id(0x123);
  frame.set_id_as_extended();
  // Note: Cannot set frame type directly, this test verifies extended ID requirement

  REQUIRE(relay->parseMessage(frame) == polymath::sygnal::MvecMessageType::UNSUPPORTED);
}

TEST_CASE("MvecRelay parse fuse status message", "[mvec_relay]")
{
  auto relay = std::make_unique<polymath::sygnal::MvecRelay>();
  J1939_ID fuse_status_id(6, 0, polymath::sygnal::MvecProtocol::STATUS_PDU, 0x00 + 0xA0, 0xB0);
  std::vector<uint8_t> data = {0x00, 0x55, 0xAA, 0x33, 0xCC, 0x0F, 0xF0, 0x99};

  auto frame = createTestFrame(fuse_status_id.get_can_id(), data);
  REQUIRE(relay->parseMessage(frame) == polymath::sygnal::MvecMessageType::FUSE_STATUS);

  // Test that status message was updated
  REQUIRE(relay->get_fuse_status_message().is_valid());
  REQUIRE(relay->get_fuse_status_message().get_fuse_status(0) == polymath::sygnal::MvecFuseStatus::BLOWN);
  REQUIRE(relay->get_fuse_status_message().get_fuse_status(1) == polymath::sygnal::MvecFuseStatus::BLOWN);

  // Test direct status message access
  REQUIRE(relay->get_fuse_status_message().is_valid());
  REQUIRE(relay->get_fuse_status_message().get_fuse_status(0) == polymath::sygnal::MvecFuseStatus::BLOWN);
}

TEST_CASE("MvecRelay parse relay status message", "[mvec_relay]")
{
  auto relay = std::make_unique<polymath::sygnal::MvecRelay>();
  J1939_ID relay_status_id(6, 0, polymath::sygnal::MvecProtocol::STATUS_PDU, 0x01 + 0xA0, 0xB0);
  std::vector<uint8_t> data = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77};

  auto frame = createTestFrame(relay_status_id.get_can_id(), data);
  REQUIRE(relay->parseMessage(frame) == polymath::sygnal::MvecMessageType::RELAY_STATUS);

  // Test that status message was updated
  REQUIRE(relay->get_relay_status_message().is_valid());
  REQUIRE(relay->get_relay_status_message().get_relay_status(0) == polymath::sygnal::MvecRelayStatus::COIL_OPEN);

  // Test direct status message access
  REQUIRE(relay->get_relay_status_message().is_valid());
  REQUIRE(relay->get_relay_status_message().get_relay_status(0) == polymath::sygnal::MvecRelayStatus::COIL_OPEN);
}

TEST_CASE("MvecRelay parse error status message", "[mvec_relay]")
{
  auto relay = std::make_unique<polymath::sygnal::MvecRelay>();
  J1939_ID error_status_id(6, 0, polymath::sygnal::MvecProtocol::STATUS_PDU, 0x02 + 0xA0, 0xB0);
  std::vector<uint8_t> data = {0xB0, 0xFF, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00};

  auto frame = createTestFrame(error_status_id.get_can_id(), data);
  REQUIRE(relay->parseMessage(frame) == polymath::sygnal::MvecMessageType::ERROR_STATUS);

  // Test that status message was updated
  REQUIRE(relay->get_error_status_message().is_valid());
  REQUIRE(relay->get_error_status_message().get_grid_address() == 0xB0);
  REQUIRE(relay->get_error_status_message().get_error_bits() > 0);

  // Test enum-based error checking
  REQUIRE(relay->get_error_status_message().has_error(polymath::sygnal::MvecErrorType::INVALID_CONFIG));
  REQUIRE(relay->get_error_status_message().has_error(polymath::sygnal::MvecErrorType::GRID_ID_CHANGED));

  // Test direct status message access
  REQUIRE(relay->get_error_status_message().is_valid());
  REQUIRE(relay->get_error_status_message().get_grid_address() == 0xB0);
}

TEST_CASE("MvecRelay parse specific response relay command reply", "[mvec_relay]")
{
  auto relay = std::make_unique<polymath::sygnal::MvecRelay>();
  J1939_ID specific_response_id(6, 0, polymath::sygnal::MvecProtocol::QUERY_PDU, 0x00, 0xB0);
  std::vector<uint8_t> data = {0x01, 0x88, 0x01, 0x00, 0xFF, 0x0F, 0x00, 0x00};

  auto frame = createTestFrame(specific_response_id.get_can_id(), data);
  REQUIRE(relay->parseMessage(frame) == polymath::sygnal::MvecMessageType::RELAY_COMMAND_RESPONSE);
}

TEST_CASE("MvecRelay parse specific response population reply", "[mvec_relay]")
{
  auto relay = std::make_unique<polymath::sygnal::MvecRelay>();
  J1939_ID specific_response_id(6, 0, polymath::sygnal::MvecProtocol::QUERY_PDU, 0x00, 0xB0);
  std::vector<uint8_t> data = {0x94, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x0F, 0x00};

  auto frame = createTestFrame(specific_response_id.get_can_id(), data);
  REQUIRE(relay->parseMessage(frame) == polymath::sygnal::MvecMessageType::POPULATION_RESPONSE);
}

TEST_CASE("MvecRelay parse specific response relay query reply", "[mvec_relay]")
{
  auto relay = std::make_unique<polymath::sygnal::MvecRelay>();
  J1939_ID specific_response_id(6, 0, polymath::sygnal::MvecProtocol::QUERY_PDU, 0x00, 0xB0);
  std::vector<uint8_t> data = {0x96, 0x00, 0xFF, 0x0F, 0xAA, 0x55, 0x00, 0x00};

  auto frame = createTestFrame(specific_response_id.get_can_id(), data);
  REQUIRE(relay->parseMessage(frame) == polymath::sygnal::MvecMessageType::RELAY_QUERY_RESPONSE);

  // Verify response parser received and parsed the data
  const auto & reply = relay->get_last_relay_query_reply();
  REQUIRE(reply.is_valid());
  for (int i = 0; i < 12; ++i) {
    REQUIRE(reply.get_relay_state(i));
  }
}

TEST_CASE("MvecRelay parse specific response unknown message id", "[mvec_relay]")
{
  auto relay = std::make_unique<polymath::sygnal::MvecRelay>();
  J1939_ID specific_response_id(6, 0, polymath::sygnal::MvecProtocol::QUERY_PDU, 0x00, 0xB0);
  std::vector<uint8_t> data = {0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  auto frame = createTestFrame(specific_response_id.get_can_id(), data);
  REQUIRE(relay->parseMessage(frame) == polymath::sygnal::MvecMessageType::UNSUPPORTED);
}

TEST_CASE("MvecRelay parse message with unknown id", "[mvec_relay]")
{
  auto relay = std::make_unique<polymath::sygnal::MvecRelay>();
  J1939_ID unknown_id(6, 0, 0x00, 0x00, 0x00);
  std::vector<uint8_t> data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  auto frame = createTestFrame(unknown_id.get_can_id(), data);
  REQUIRE(relay->parseMessage(frame) == polymath::sygnal::MvecMessageType::UNSUPPORTED);
}

TEST_CASE("MvecRelay boundary test max relays", "[mvec_relay]")
{
  auto relay = std::make_unique<polymath::sygnal::MvecRelay>();
  for (uint8_t i = 0; i < 12; ++i) {
    relay->set_relay_in_command(i, 1);
    // Test that command message generation works
    auto frame = relay->getRelayCommandMessage();
    REQUIRE(frame.get_id_type() == polymath::socketcan::IdType::EXTENDED);
  }
}

TEST_CASE("MvecRelay boundary test max fuses", "[mvec_relay]")
{
  auto relay = std::make_unique<polymath::sygnal::MvecRelay>();
  for (uint8_t i = 0; i < 24; ++i) {
    REQUIRE(relay->get_fuse_status_message().get_fuse_status(i) == polymath::sygnal::MvecFuseStatus::NOT_USED);
  }
}
