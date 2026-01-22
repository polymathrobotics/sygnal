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

#include "sygnal_can_interface_lib/sygnal_command_interface.hpp"

#include <string>
#include <vector>

#include <catch2/catch.hpp>

#include "socketcan_adapter/can_frame.hpp"

// Frame IDs from DBC
constexpr uint32_t CONTROL_ENABLE_FRAME_ID = 0x060;
constexpr uint32_t CONTROL_ENABLE_RESPONSE_FRAME_ID = 0x061;
constexpr uint32_t CONTROL_COMMAND_FRAME_ID = 0x160;
constexpr uint32_t CONTROL_COMMAND_RESPONSE_FRAME_ID = 0x161;
constexpr uint32_t RELAY_COMMAND_FRAME_ID = 0x0B0;
constexpr uint32_t RELAY_COMMAND_RESPONSE_FRAME_ID = 0x0B1;

static polymath::socketcan::CanFrame createTestFrame(uint32_t can_id, const std::vector<uint8_t> & data)
{
  polymath::socketcan::CanFrame frame;
  frame.set_can_id(can_id);
  std::array<unsigned char, CAN_MAX_DLC> frame_data;
  frame_data.fill(0);
  for (size_t i = 0; i < data.size() && i < CAN_MAX_DLC; ++i) {
    frame_data[i] = data[i];
  }
  frame.set_data(frame_data);
  frame.set_len(data.size());
  return frame;
}

TEST_CASE("SygnalControlInterface creates ControlEnable frame", "[sygnal_command_interface]")
{
  polymath::sygnal::SygnalControlInterface interface;
  std::string error_message;

  auto frame_opt =
    interface.createControlStateCommandFrame(1, 3, polymath::sygnal::SygnalControlState::MCM_CONTROL, error_message);

  REQUIRE(frame_opt.has_value());
  REQUIRE(error_message.empty());

  auto frame = frame_opt.value();
  REQUIRE(frame.get_id() == CONTROL_ENABLE_FRAME_ID);

  // Expected from DBC: {0x01, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00, 0xF0}
  auto data = frame.get_data();
  REQUIRE(data[0] == 0x01);  // BusAddress = 1
  REQUIRE((data[1] & 0x0F) == 0x00);  // InterfaceID lower bits
  REQUIRE(((data[1] >> 4) & 0x0F) == 0x06);  // InterfaceID = 3 encoded
}

TEST_CASE("SygnalControlInterface creates ControlCommand frame", "[sygnal_command_interface]")
{
  polymath::sygnal::SygnalControlInterface interface;
  std::string error_message;

  auto frame_opt = interface.createControlCommandFrame(1, 0, 0.5, error_message);

  REQUIRE(frame_opt.has_value());
  REQUIRE(error_message.empty());

  auto frame = frame_opt.value();
  REQUIRE(frame.get_id() == CONTROL_COMMAND_FRAME_ID);

  auto data = frame.get_data();
  REQUIRE(data[0] == 0x01);  // BusAddress = 1
}

TEST_CASE("SygnalControlInterface creates RelayCommand frame", "[sygnal_command_interface]")
{
  polymath::sygnal::SygnalControlInterface interface;
  std::string error_message;

  auto frame_opt = interface.createRelayCommandFrame(1, 0, true, error_message);

  REQUIRE(frame_opt.has_value());
  REQUIRE(error_message.empty());

  auto frame = frame_opt.value();
  REQUIRE(frame.get_id() == RELAY_COMMAND_FRAME_ID);

  // Expected from DBC: {0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0xBD}
  auto data = frame.get_data();
  REQUIRE(data[0] == 0x01);  // BusAddress = 1
  REQUIRE(data[2] == 0x01);  // Enable = 1
}

TEST_CASE("SygnalControlInterface parses ControlEnableResponse", "[sygnal_command_interface]")
{
  polymath::sygnal::SygnalControlInterface interface;

  // Generated from DBC: ControlEnableResponse (bus=1, interface=3, enable=1)
  std::vector<uint8_t> data = {0x01, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00, 0xF0};
  auto frame = createTestFrame(CONTROL_ENABLE_RESPONSE_FRAME_ID, data);

  auto response = interface.parseCommandResponseFrame(frame);

  REQUIRE(response.has_value());
  REQUIRE(response->response_type == polymath::sygnal::SygnalControlCommandResponseType::ENABLE);
  REQUIRE(response->bus_id == 1);
  REQUIRE(response->value == 1.0);
}

TEST_CASE("SygnalControlInterface parses ControlCommandResponse", "[sygnal_command_interface]")
{
  polymath::sygnal::SygnalControlInterface interface;

  // Generated from DBC: ControlCommandResponse (bus=1, interface=0, value=0.5)
  std::vector<uint8_t> data = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x62};
  auto frame = createTestFrame(CONTROL_COMMAND_RESPONSE_FRAME_ID, data);

  auto response = interface.parseCommandResponseFrame(frame);

  REQUIRE(response.has_value());
  REQUIRE(response->response_type == polymath::sygnal::SygnalControlCommandResponseType::CONTROL);
  REQUIRE(response->bus_id == 1);
  REQUIRE(response->interface_id == 0);
}

TEST_CASE("SygnalControlInterface parses RelayCommandResponse", "[sygnal_command_interface]")
{
  polymath::sygnal::SygnalControlInterface interface;

  // Generated from DBC: RelayCommandResponse (bus=1, subsystem=0, enable=1)
  std::vector<uint8_t> data = {0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0xBD};
  auto frame = createTestFrame(RELAY_COMMAND_RESPONSE_FRAME_ID, data);

  auto response = interface.parseCommandResponseFrame(frame);

  REQUIRE(response.has_value());
  REQUIRE(response->response_type == polymath::sygnal::SygnalControlCommandResponseType::RELAY);
  REQUIRE(response->bus_id == 1);
  REQUIRE(response->interface_id == 0);  // subsystem_id
  REQUIRE(response->value == 1.0);
}

TEST_CASE("SygnalControlInterface rejects unknown frame ID", "[sygnal_command_interface]")
{
  polymath::sygnal::SygnalControlInterface interface;

  std::vector<uint8_t> data = {0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0xBD};
  auto frame = createTestFrame(0x999, data);

  auto response = interface.parseCommandResponseFrame(frame);

  REQUIRE_FALSE(response.has_value());
}

TEST_CASE("SygnalControlInterface rejects bad CRC in response", "[sygnal_command_interface]")
{
  polymath::sygnal::SygnalControlInterface interface;

  // Valid data but corrupted CRC
  std::vector<uint8_t> data = {0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFF};
  auto frame = createTestFrame(RELAY_COMMAND_RESPONSE_FRAME_ID, data);

  auto response = interface.parseCommandResponseFrame(frame);

  REQUIRE_FALSE(response.has_value());
}
