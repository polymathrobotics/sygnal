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

#include "sygnal_can_interface_lib/sygnal_mcm_device.hpp"

#include <string>
#include <vector>

#if __has_include(<catch2/catch_all.hpp>)
  #include <catch2/catch_all.hpp>
#elif __has_include(<catch2/catch.hpp>)
  #include <catch2/catch.hpp>
#else
  #error "Catch2 headers not found. Please install Catch2 (v2 or v3)."
#endif

#include "socketcan_adapter/can_frame.hpp"

// Frame IDs from DBC
constexpr uint32_t HEARTBEAT_FRAME_ID = 0x170;
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

TEST_CASE("SygnalMcmDevice constructor initializes to FAIL_HARD for subsystem 0", "[sygnal_mcm_device]")
{
  polymath::sygnal::SygnalMcmDevice interface(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 0);

  REQUIRE(interface.get_mcm_state() == polymath::sygnal::SygnalSystemState::FAIL_HARD);

  auto states = interface.get_interface_states();
  for (const auto & state : states) {
    REQUIRE(state == polymath::sygnal::SygnalSystemState::FAIL_HARD);
  }
}

TEST_CASE("SygnalMcmDevice constructor initializes to FAIL_HARD for subsystem 1", "[sygnal_mcm_device]")
{
  polymath::sygnal::SygnalMcmDevice interface(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 1);

  REQUIRE(interface.get_mcm_state() == polymath::sygnal::SygnalSystemState::FAIL_HARD);

  auto states = interface.get_interface_states();
  for (const auto & state : states) {
    REQUIRE(state == polymath::sygnal::SygnalSystemState::FAIL_HARD);
  }
}

TEST_CASE("SygnalMcmDevice parses MCM_CONTROL heartbeat", "[sygnal_mcm_device]")
{
  polymath::sygnal::SygnalMcmDevice interface(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 0);

  // Generated from DBC: MCM_CONTROL state, subsystem 0, all interfaces MCM_CONTROL
  std::vector<uint8_t> data = {0x01, 0x00, 0x01, 0x9F, 0x00, 0x64, 0x00, 0x98};
  auto frame = createTestFrame(HEARTBEAT_FRAME_ID, data);

  REQUIRE(interface.parseMcmHeartbeatFrame(frame) == true);
  REQUIRE(interface.get_mcm_state() == polymath::sygnal::SygnalSystemState::MCM_CONTROL);

  auto states = interface.get_interface_states();
  for (size_t i = 0; i < 5; ++i) {
    REQUIRE(states[i] == polymath::sygnal::SygnalSystemState::MCM_CONTROL);
  }
}

TEST_CASE("SygnalMcmDevice parses HUMAN_CONTROL heartbeat", "[sygnal_mcm_device]")
{
  polymath::sygnal::SygnalMcmDevice interface(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 1);

  // Generated from DBC: HUMAN_CONTROL state, subsystem 1
  std::vector<uint8_t> data = {0x81, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00, 0x86};
  auto frame = createTestFrame(HEARTBEAT_FRAME_ID, data);

  REQUIRE(interface.parseMcmHeartbeatFrame(frame) == true);
  REQUIRE(interface.get_mcm_state() == polymath::sygnal::SygnalSystemState::HUMAN_CONTROL);

  auto states = interface.get_interface_states();
  for (size_t i = 0; i < 5; ++i) {
    REQUIRE(states[i] == polymath::sygnal::SygnalSystemState::HUMAN_CONTROL);
  }
}

TEST_CASE("SygnalMcmDevice rejects wrong frame ID", "[sygnal_mcm_device]")
{
  polymath::sygnal::SygnalMcmDevice interface(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 0);

  std::vector<uint8_t> data = {0x01, 0x00, 0x01, 0x9F, 0x00, 0x64, 0x00, 0x98};
  auto frame = createTestFrame(0x999, data);  // Wrong ID

  REQUIRE(interface.parseMcmHeartbeatFrame(frame) == false);
  // State should remain FAIL_HARD
  REQUIRE(interface.get_mcm_state() == polymath::sygnal::SygnalSystemState::FAIL_HARD);
}

TEST_CASE("SygnalMcmDevice rejects bad CRC", "[sygnal_mcm_device]")
{
  polymath::sygnal::SygnalMcmDevice interface(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 0);

  // Valid data but corrupted CRC (last byte)
  std::vector<uint8_t> data = {0x01, 0x00, 0x01, 0x9F, 0x00, 0x64, 0x00, 0xFF};
  auto frame = createTestFrame(HEARTBEAT_FRAME_ID, data);

  REQUIRE(interface.parseMcmHeartbeatFrame(frame) == false);
  REQUIRE(interface.get_mcm_state() == polymath::sygnal::SygnalSystemState::FAIL_HARD);
}

TEST_CASE("SygnalMcmDevice only parses matching subsystem_id", "[sygnal_mcm_device]")
{
  // Interface configured for subsystem 0 should ignore subsystem 1 frames
  polymath::sygnal::SygnalMcmDevice interface_0(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 0);

  // Heartbeat frame for subsystem 1
  std::vector<uint8_t> data_sub1 = {0x81, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00, 0x86};
  auto frame_sub1 = createTestFrame(HEARTBEAT_FRAME_ID, data_sub1);

  // Should return false and state should remain FAIL_HARD
  REQUIRE(interface_0.parseMcmHeartbeatFrame(frame_sub1) == false);
  REQUIRE(interface_0.get_mcm_state() == polymath::sygnal::SygnalSystemState::FAIL_HARD);

  // Interface configured for subsystem 1 should ignore subsystem 0 frames
  polymath::sygnal::SygnalMcmDevice interface_1(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 1);

  // Heartbeat frame for subsystem 0
  std::vector<uint8_t> data_sub0 = {0x01, 0x00, 0x01, 0x9F, 0x00, 0x64, 0x00, 0x98};
  auto frame_sub0 = createTestFrame(HEARTBEAT_FRAME_ID, data_sub0);

  // Should return false and state should remain FAIL_HARD
  REQUIRE(interface_1.parseMcmHeartbeatFrame(frame_sub0) == false);
  REQUIRE(interface_1.get_mcm_state() == polymath::sygnal::SygnalSystemState::FAIL_HARD);
}

TEST_CASE("sygnalSystemStateToString returns correct strings", "[sygnal_mcm_device]")
{
  using polymath::sygnal::SygnalSystemState;
  using polymath::sygnal::sygnalSystemStateToString;

  REQUIRE(sygnalSystemStateToString(SygnalSystemState::FAIL_HARD) == "FAIL_HARD");
  REQUIRE(sygnalSystemStateToString(SygnalSystemState::HUMAN_OVERRIDE) == "HUMAN_OVERRIDE");
  REQUIRE(sygnalSystemStateToString(SygnalSystemState::FAIL_OPERATIONAL_2) == "FAIL_OPERATIONAL_2");
  REQUIRE(sygnalSystemStateToString(SygnalSystemState::FAIL_OPERATIONAL_1) == "FAIL_OPERATIONAL_1");
  REQUIRE(sygnalSystemStateToString(SygnalSystemState::MCM_CONTROL) == "MCM_CONTROL");
  REQUIRE(sygnalSystemStateToString(SygnalSystemState::HUMAN_CONTROL) == "HUMAN_CONTROL");
}

TEST_CASE("SygnalMcmDevice creates ControlEnable frame", "[sygnal_mcm_device]")
{
  polymath::sygnal::SygnalMcmDevice interface;
  std::string error_message;

  auto frame_opt =
    interface.createControlStateCommandFrame(1, 3, 0, polymath::sygnal::SygnalControlState::MCM_CONTROL, error_message);

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

TEST_CASE("SygnalMcmDevice creates ControlCommand frame", "[sygnal_mcm_device]")
{
  polymath::sygnal::SygnalMcmDevice interface;
  std::string error_message;

  auto frame_opt = interface.createControlCommandFrame(1, 0, 0, 0.5, error_message);

  REQUIRE(frame_opt.has_value());
  REQUIRE(error_message.empty());

  auto frame = frame_opt.value();
  REQUIRE(frame.get_id() == CONTROL_COMMAND_FRAME_ID);

  auto data = frame.get_data();
  REQUIRE(data[0] == 0x01);  // BusAddress = 1
}

TEST_CASE("SygnalMcmDevice creates RelayCommand frame", "[sygnal_mcm_device]")
{
  polymath::sygnal::SygnalMcmDevice interface;
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

TEST_CASE("SygnalMcmDevice parses ControlEnableResponse", "[sygnal_mcm_device]")
{
  polymath::sygnal::SygnalMcmDevice interface;

  // Generated from DBC: ControlEnableResponse (bus=1, interface=3, enable=1)
  std::vector<uint8_t> data = {0x01, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00, 0xF0};
  auto frame = createTestFrame(CONTROL_ENABLE_RESPONSE_FRAME_ID, data);

  auto response = interface.parseCommandResponseFrame(frame);

  REQUIRE(response.has_value());
  REQUIRE(response->response_type == polymath::sygnal::SygnalControlCommandResponseType::ENABLE);
  REQUIRE(response->bus_id == 1);
  REQUIRE(response->value == 1.0);
}

TEST_CASE("SygnalMcmDevice parses ControlCommandResponse", "[sygnal_mcm_device]")
{
  polymath::sygnal::SygnalMcmDevice interface;

  // Generated from DBC: ControlCommandResponse (bus=1, interface=0, value=0.5)
  std::vector<uint8_t> data = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x62};
  auto frame = createTestFrame(CONTROL_COMMAND_RESPONSE_FRAME_ID, data);

  auto response = interface.parseCommandResponseFrame(frame);

  REQUIRE(response.has_value());
  REQUIRE(response->response_type == polymath::sygnal::SygnalControlCommandResponseType::CONTROL);
  REQUIRE(response->bus_id == 1);
  REQUIRE(response->interface_id == 0);
}

TEST_CASE("SygnalMcmDevice parses RelayCommandResponse", "[sygnal_mcm_device]")
{
  polymath::sygnal::SygnalMcmDevice interface;

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

TEST_CASE("SygnalMcmDevice rejects unknown frame ID", "[sygnal_mcm_device]")
{
  polymath::sygnal::SygnalMcmDevice interface;

  std::vector<uint8_t> data = {0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0xBD};
  auto frame = createTestFrame(0x999, data);

  auto response = interface.parseCommandResponseFrame(frame);

  REQUIRE_FALSE(response.has_value());
}

TEST_CASE("SygnalMcmDevice rejects bad CRC in response", "[sygnal_mcm_device]")
{
  polymath::sygnal::SygnalMcmDevice interface;

  // Valid data but corrupted CRC
  std::vector<uint8_t> data = {0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFF};
  auto frame = createTestFrame(RELAY_COMMAND_RESPONSE_FRAME_ID, data);

  auto response = interface.parseCommandResponseFrame(frame);

  REQUIRE_FALSE(response.has_value());
}
