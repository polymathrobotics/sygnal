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

#if __has_include(<catch2/catch_all.hpp>)
  #include <catch2/catch_all.hpp>
#elif __has_include(<catch2/catch.hpp>)
  #include <catch2/catch.hpp>
#else
  #error "Catch2 headers not found. Please install Catch2 (v2 or v3)."
#endif

#include "socketcan_adapter/can_frame.hpp"
#include "sygnal_can_interface_lib/crc8.hpp"
#include "sygnal_dbc/mcm_fault.h"

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

/// @brief Pack a fault message via cantools, append a valid CRC8, and wrap it in a CanFrame.
template <typename T, typename PackFn>
static polymath::socketcan::CanFrame createFaultFrame(uint32_t can_id, PackFn pack, const T & msg)
{
  std::array<uint8_t, 8> packed{};
  packed.fill(0);
  pack(packed.data(), &msg, packed.size());
  // CRC8 lives in byte 7 over bytes 0..6.
  packed[7] = polymath::sygnal::generate_crc8(packed.data());
  std::vector<uint8_t> data(packed.begin(), packed.end());
  return createTestFrame(can_id, data);
}

TEST_CASE("SygnalMcmInterface constructor initializes to FAIL_HARD for subsystem 0", "[sygnal_mcm_interface]")
{
  polymath::sygnal::SygnalMcmInterface interface(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 0);

  REQUIRE(interface.get_mcm_state() == polymath::sygnal::SygnalSystemState::FAIL_HARD);

  auto states = interface.get_interface_states();
  for (const auto & state : states) {
    REQUIRE(state == polymath::sygnal::SygnalSystemState::FAIL_HARD);
  }
}

TEST_CASE("SygnalMcmInterface constructor initializes to FAIL_HARD for subsystem 1", "[sygnal_mcm_interface]")
{
  polymath::sygnal::SygnalMcmInterface interface(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 1);

  REQUIRE(interface.get_mcm_state() == polymath::sygnal::SygnalSystemState::FAIL_HARD);

  auto states = interface.get_interface_states();
  for (const auto & state : states) {
    REQUIRE(state == polymath::sygnal::SygnalSystemState::FAIL_HARD);
  }
}

TEST_CASE("SygnalMcmInterface parses MCM_CONTROL heartbeat", "[sygnal_mcm_interface]")
{
  polymath::sygnal::SygnalMcmInterface interface(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 0);

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

TEST_CASE("SygnalMcmInterface parses HUMAN_CONTROL heartbeat", "[sygnal_mcm_interface]")
{
  polymath::sygnal::SygnalMcmInterface interface(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 1);

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

TEST_CASE("SygnalMcmInterface rejects wrong frame ID", "[sygnal_mcm_interface]")
{
  polymath::sygnal::SygnalMcmInterface interface(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 0);

  std::vector<uint8_t> data = {0x01, 0x00, 0x01, 0x9F, 0x00, 0x64, 0x00, 0x98};
  auto frame = createTestFrame(0x999, data);  // Wrong ID

  REQUIRE(interface.parseMcmHeartbeatFrame(frame) == false);
  // State should remain FAIL_HARD
  REQUIRE(interface.get_mcm_state() == polymath::sygnal::SygnalSystemState::FAIL_HARD);
}

TEST_CASE("SygnalMcmInterface rejects bad CRC", "[sygnal_mcm_interface]")
{
  polymath::sygnal::SygnalMcmInterface interface(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 0);

  // Valid data but corrupted CRC (last byte)
  std::vector<uint8_t> data = {0x01, 0x00, 0x01, 0x9F, 0x00, 0x64, 0x00, 0xFF};
  auto frame = createTestFrame(HEARTBEAT_FRAME_ID, data);

  REQUIRE(interface.parseMcmHeartbeatFrame(frame) == false);
  REQUIRE(interface.get_mcm_state() == polymath::sygnal::SygnalSystemState::FAIL_HARD);
}

TEST_CASE("SygnalMcmInterface only parses matching subsystem_id", "[sygnal_mcm_interface]")
{
  // Interface configured for subsystem 0 should ignore subsystem 1 frames
  polymath::sygnal::SygnalMcmInterface interface_0(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 0);

  // Heartbeat frame for subsystem 1
  std::vector<uint8_t> data_sub1 = {0x81, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00, 0x86};
  auto frame_sub1 = createTestFrame(HEARTBEAT_FRAME_ID, data_sub1);

  // Should return false and state should remain FAIL_HARD
  REQUIRE(interface_0.parseMcmHeartbeatFrame(frame_sub1) == false);
  REQUIRE(interface_0.get_mcm_state() == polymath::sygnal::SygnalSystemState::FAIL_HARD);

  // Interface configured for subsystem 1 should ignore subsystem 0 frames
  polymath::sygnal::SygnalMcmInterface interface_1(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 1);

  // Heartbeat frame for subsystem 0
  std::vector<uint8_t> data_sub0 = {0x01, 0x00, 0x01, 0x9F, 0x00, 0x64, 0x00, 0x98};
  auto frame_sub0 = createTestFrame(HEARTBEAT_FRAME_ID, data_sub0);

  // Should return false and state should remain FAIL_HARD
  REQUIRE(interface_1.parseMcmHeartbeatFrame(frame_sub0) == false);
  REQUIRE(interface_1.get_mcm_state() == polymath::sygnal::SygnalSystemState::FAIL_HARD);
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

TEST_CASE("SygnalMcmInterface initializes fault state cleanly", "[sygnal_mcm_interface][fault]")
{
  polymath::sygnal::SygnalMcmInterface interface(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 0);

  REQUIRE(interface.get_last_fault_cause() == polymath::sygnal::NO_FAULT_CAUSE);
  REQUIRE(interface.get_last_root_cause().fail_op1 == polymath::sygnal::NO_FAULT_CAUSE);
  REQUIRE(interface.get_last_root_cause().fail_op2 == polymath::sygnal::NO_FAULT_CAUSE);
  REQUIRE(interface.get_last_root_cause().fail_hard == polymath::sygnal::NO_FAULT_CAUSE);
  REQUIRE(interface.get_fault_counts().empty());
}

TEST_CASE("SygnalMcmInterface parses FaultState frame", "[sygnal_mcm_interface][fault]")
{
  polymath::sygnal::SygnalMcmInterface interface(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 0);

  mcm_fault_fault_state_t msg{};
  msg.bus_address = polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS;
  msg.subsystem_id = 0;
  msg.fault_state = static_cast<uint8_t>(polymath::sygnal::SygnalSystemState::FAIL_OPERATIONAL_1);
  msg.fault_cause = 0x42;

  auto frame = createFaultFrame(MCM_FAULT_FAULT_STATE_FRAME_ID, mcm_fault_fault_state_pack, msg);

  REQUIRE(interface.parseFaultStateFrame(frame) == true);
  REQUIRE(interface.get_last_fault_cause() == 0x42);
  // FaultState also surfaces the system state byte, which the heartbeat parser would otherwise own.
  REQUIRE(interface.get_mcm_state() == polymath::sygnal::SygnalSystemState::FAIL_OPERATIONAL_1);
}

TEST_CASE(
  "SygnalMcmInterface FaultState rejects wrong frame ID, bad CRC, and mismatched address",
  "[sygnal_mcm_interface][fault]")
{
  polymath::sygnal::SygnalMcmInterface interface(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 0);

  mcm_fault_fault_state_t msg{};
  msg.bus_address = polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS;
  msg.subsystem_id = 0;
  msg.fault_state = static_cast<uint8_t>(polymath::sygnal::SygnalSystemState::FAIL_HARD);
  msg.fault_cause = 0x11;

  // Wrong frame ID
  auto wrong_id_frame = createFaultFrame(0x999, mcm_fault_fault_state_pack, msg);
  REQUIRE(interface.parseFaultStateFrame(wrong_id_frame) == false);
  REQUIRE(interface.get_last_fault_cause() == polymath::sygnal::NO_FAULT_CAUSE);

  // Bad CRC
  auto good_frame = createFaultFrame(MCM_FAULT_FAULT_STATE_FRAME_ID, mcm_fault_fault_state_pack, msg);
  auto bad_crc_data = good_frame.get_frame();
  bad_crc_data.data[7] = static_cast<uint8_t>(bad_crc_data.data[7] ^ 0xFFu);
  polymath::socketcan::CanFrame bad_crc_frame;
  bad_crc_frame.set_can_id(MCM_FAULT_FAULT_STATE_FRAME_ID);
  std::array<unsigned char, CAN_MAX_DLC> arr;
  arr.fill(0);
  for (size_t i = 0; i < 8; ++i) {
    arr[i] = bad_crc_data.data[i];
  }
  bad_crc_frame.set_data(arr);
  bad_crc_frame.set_len(8);
  REQUIRE(interface.parseFaultStateFrame(bad_crc_frame) == false);
  REQUIRE(interface.get_last_fault_cause() == polymath::sygnal::NO_FAULT_CAUSE);

  // Mismatched bus_address
  mcm_fault_fault_state_t wrong_bus = msg;
  wrong_bus.bus_address = polymath::sygnal::SECONDARY_MCM_BUS_ADDRESS;
  auto wrong_bus_frame = createFaultFrame(MCM_FAULT_FAULT_STATE_FRAME_ID, mcm_fault_fault_state_pack, wrong_bus);
  REQUIRE(interface.parseFaultStateFrame(wrong_bus_frame) == false);
  REQUIRE(interface.get_last_fault_cause() == polymath::sygnal::NO_FAULT_CAUSE);

  // Mismatched subsystem_id
  mcm_fault_fault_state_t wrong_sub = msg;
  wrong_sub.subsystem_id = 1;
  auto wrong_sub_frame = createFaultFrame(MCM_FAULT_FAULT_STATE_FRAME_ID, mcm_fault_fault_state_pack, wrong_sub);
  REQUIRE(interface.parseFaultStateFrame(wrong_sub_frame) == false);
  REQUIRE(interface.get_last_fault_cause() == polymath::sygnal::NO_FAULT_CAUSE);
}

TEST_CASE("SygnalMcmInterface FaultIncrement updates fault count map", "[sygnal_mcm_interface][fault]")
{
  polymath::sygnal::SygnalMcmInterface interface(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 0);

  mcm_fault_fault_increment_t msg{};
  msg.bus_address = polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS;
  msg.subsystem_id = 0;
  msg.fault_type = 0x05;
  msg.fault_count = 3;

  auto frame = createFaultFrame(MCM_FAULT_FAULT_INCREMENT_FRAME_ID, mcm_fault_fault_increment_pack, msg);
  REQUIRE(interface.parseFaultIncrementFrame(frame) == true);

  auto counts = interface.get_fault_counts();
  REQUIRE(counts.size() == 1);
  REQUIRE(counts.at(0x05) == 3);

  // A second message for the same cause overwrites (carries current total, not a delta)
  msg.fault_count = 7;
  auto frame_2 = createFaultFrame(MCM_FAULT_FAULT_INCREMENT_FRAME_ID, mcm_fault_fault_increment_pack, msg);
  REQUIRE(interface.parseFaultIncrementFrame(frame_2) == true);

  counts = interface.get_fault_counts();
  REQUIRE(counts.size() == 1);
  REQUIRE(counts.at(0x05) == 7);

  // A different cause adds a new entry
  msg.fault_type = 0x09;
  msg.fault_count = 2;
  auto frame_3 = createFaultFrame(MCM_FAULT_FAULT_INCREMENT_FRAME_ID, mcm_fault_fault_increment_pack, msg);
  REQUIRE(interface.parseFaultIncrementFrame(frame_3) == true);

  counts = interface.get_fault_counts();
  REQUIRE(counts.size() == 2);
  REQUIRE(counts.at(0x05) == 7);
  REQUIRE(counts.at(0x09) == 2);
}

TEST_CASE("SygnalMcmInterface FaultList updates fault count map", "[sygnal_mcm_interface][fault]")
{
  polymath::sygnal::SygnalMcmInterface interface(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 1);

  mcm_fault_fault_list_t msg{};
  msg.bus_address = polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS;
  msg.subsystem_id = 1;
  msg.fault_type = 0x12;
  msg.fault_count = 42;

  auto frame = createFaultFrame(MCM_FAULT_FAULT_LIST_FRAME_ID, mcm_fault_fault_list_pack, msg);
  REQUIRE(interface.parseFaultListFrame(frame) == true);

  auto counts = interface.get_fault_counts();
  REQUIRE(counts.size() == 1);
  REQUIRE(counts.at(0x12) == 42);
}

TEST_CASE("SygnalMcmInterface parses FaultRootCause", "[sygnal_mcm_interface][fault]")
{
  polymath::sygnal::SygnalMcmInterface interface(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 0);

  mcm_fault_fault_root_cause_t msg{};
  msg.bus_address = polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS;
  msg.subsystem_id = 0;
  msg.fail_op1_cause = 0x10;
  msg.fail_op2_cause = 0x20;
  msg.fail_hard_cause = 0x30;

  auto frame = createFaultFrame(MCM_FAULT_FAULT_ROOT_CAUSE_FRAME_ID, mcm_fault_fault_root_cause_pack, msg);
  REQUIRE(interface.parseFaultRootCauseFrame(frame) == true);

  auto root = interface.get_last_root_cause();
  REQUIRE(root.fail_op1 == 0x10);
  REQUIRE(root.fail_op2 == 0x20);
  REQUIRE(root.fail_hard == 0x30);
}

TEST_CASE("SygnalMcmInterface fault parsers respect subsystem filter", "[sygnal_mcm_interface][fault]")
{
  polymath::sygnal::SygnalMcmInterface interface_0(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 0);

  mcm_fault_fault_state_t msg{};
  msg.bus_address = polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS;
  msg.subsystem_id = 1;  // wrong subsystem
  msg.fault_state = static_cast<uint8_t>(polymath::sygnal::SygnalSystemState::FAIL_HARD);
  msg.fault_cause = 0x77;

  auto frame = createFaultFrame(MCM_FAULT_FAULT_STATE_FRAME_ID, mcm_fault_fault_state_pack, msg);
  REQUIRE(interface_0.parseFaultStateFrame(frame) == false);
  REQUIRE(interface_0.get_last_fault_cause() == polymath::sygnal::NO_FAULT_CAUSE);
}
