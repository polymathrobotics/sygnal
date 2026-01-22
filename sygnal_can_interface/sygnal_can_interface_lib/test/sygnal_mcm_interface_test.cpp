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

#include "sygnal_can_interface_lib/sygnal_mcm_interface.hpp"

#include <string>
#include <vector>

#include <catch2/catch.hpp>

#include "socketcan_adapter/can_frame.hpp"

// Frame ID from DBC
constexpr uint32_t HEARTBEAT_FRAME_ID = 0x170;

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

TEST_CASE("SygnalMcmInterface constructor initializes to FAIL_HARD", "[sygnal_mcm_interface]")
{
  polymath::sygnal::SygnalMcmInterface interface;

  REQUIRE(interface.get_sygnal_mcm0_state() == polymath::sygnal::SygnalSystemState::FAIL_HARD);
  REQUIRE(interface.get_sygnal_mcm1_state() == polymath::sygnal::SygnalSystemState::FAIL_HARD);

  auto states = interface.get_interface_states();
  for (const auto & state : states) {
    REQUIRE(state == polymath::sygnal::SygnalSystemState::FAIL_HARD);
  }
}

TEST_CASE("SygnalMcmInterface parses MCM_CONTROL heartbeat", "[sygnal_mcm_interface]")
{
  polymath::sygnal::SygnalMcmInterface interface;

  // Generated from DBC: MCM_CONTROL state, subsystem 0, all interfaces MCM_CONTROL
  std::vector<uint8_t> data = {0x01, 0x00, 0x01, 0x9F, 0x00, 0x64, 0x00, 0x98};
  auto frame = createTestFrame(HEARTBEAT_FRAME_ID, data);

  REQUIRE(interface.parseMcmHeartbeatFrame(frame) == true);
  REQUIRE(interface.get_sygnal_mcm0_state() == polymath::sygnal::SygnalSystemState::MCM_CONTROL);

  auto states = interface.get_interface_states();
  for (size_t i = 0; i < 5; ++i) {
    REQUIRE(states[i] == polymath::sygnal::SygnalSystemState::MCM_CONTROL);
  }
}

TEST_CASE("SygnalMcmInterface parses HUMAN_CONTROL heartbeat", "[sygnal_mcm_interface]")
{
  polymath::sygnal::SygnalMcmInterface interface;

  // Generated from DBC: HUMAN_CONTROL state, subsystem 1
  std::vector<uint8_t> data = {0x81, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00, 0x86};
  auto frame = createTestFrame(HEARTBEAT_FRAME_ID, data);

  REQUIRE(interface.parseMcmHeartbeatFrame(frame) == true);
  REQUIRE(interface.get_sygnal_mcm1_state() == polymath::sygnal::SygnalSystemState::HUMAN_CONTROL);

  auto states = interface.get_interface_states();
  for (size_t i = 0; i < 5; ++i) {
    REQUIRE(states[i] == polymath::sygnal::SygnalSystemState::HUMAN_CONTROL);
  }
}

TEST_CASE("SygnalMcmInterface rejects wrong frame ID", "[sygnal_mcm_interface]")
{
  polymath::sygnal::SygnalMcmInterface interface;

  std::vector<uint8_t> data = {0x01, 0x00, 0x01, 0x9F, 0x00, 0x64, 0x00, 0x98};
  auto frame = createTestFrame(0x999, data);  // Wrong ID

  REQUIRE(interface.parseMcmHeartbeatFrame(frame) == false);
  // State should remain FAIL_HARD
  REQUIRE(interface.get_sygnal_mcm0_state() == polymath::sygnal::SygnalSystemState::FAIL_HARD);
}

TEST_CASE("SygnalMcmInterface rejects bad CRC", "[sygnal_mcm_interface]")
{
  polymath::sygnal::SygnalMcmInterface interface;

  // Valid data but corrupted CRC (last byte)
  std::vector<uint8_t> data = {0x01, 0x00, 0x01, 0x9F, 0x00, 0x64, 0x00, 0xFF};
  auto frame = createTestFrame(HEARTBEAT_FRAME_ID, data);

  REQUIRE(interface.parseMcmHeartbeatFrame(frame) == false);
  REQUIRE(interface.get_sygnal_mcm0_state() == polymath::sygnal::SygnalSystemState::FAIL_HARD);
}

TEST_CASE("sygnalSystemStateToString returns correct strings", "[sygnal_mcm_interface]")
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
