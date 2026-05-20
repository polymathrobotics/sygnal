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

#include "sygnal_can_interface_lib/sygnal_hpo_interface.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <string>

#if __has_include(<catch2/catch_all.hpp>)
  #include <catch2/catch_all.hpp>
#elif __has_include(<catch2/catch.hpp>)
  #include <catch2/catch.hpp>
#else
  #error "Catch2 headers not found. Please install Catch2 (v2 or v3)."
#endif

#include "socketcan_adapter/can_frame.hpp"
#include "sygnal_can_interface_lib/crc8.hpp"
#include "sygnal_dbc/hpo_control.h"
#include "sygnal_dbc/hpo_error.h"
#include "sygnal_dbc/hpo_heartbeat.h"

using polymath::sygnal::HPO_NUM_INTERFACES;
using polymath::sygnal::HpoControlResponse;
using polymath::sygnal::HpoErrorStatus;
using polymath::sygnal::SygnalHpoInterface;

namespace
{

constexpr uint8_t TEST_BUS_ADDRESS = 3;
constexpr uint8_t OTHER_BUS_ADDRESS = 4;

polymath::socketcan::CanFrame makeFrame(uint32_t can_id, const uint8_t * data, uint8_t len)
{
  polymath::socketcan::CanFrame frame;
  frame.set_can_id(can_id);
  std::array<unsigned char, CAN_MAX_DLC> bytes;
  bytes.fill(0);
  for (size_t i = 0; i < len && i < CAN_MAX_DLC; ++i) {
    bytes[i] = data[i];
  }
  frame.set_data(bytes);
  frame.set_len(len);
  return frame;
}

polymath::socketcan::CanFrame buildHpoHeartbeat(
  uint8_t bus_address, const std::array<bool, HPO_NUM_INTERFACES> & interface_states, bool overall_state)
{
  hpo_heartbeat_heartbeat_t msg;
  hpo_heartbeat_heartbeat_init(&msg);
  msg.bus_address = bus_address;
  msg.subsystem_id = 0;
  msg.system_state = 0;
  msg.interface0_state = interface_states[0] ? 1 : 0;
  msg.interface1_state = interface_states[1] ? 1 : 0;
  msg.interface2_state = interface_states[2] ? 1 : 0;
  msg.interface3_state = interface_states[3] ? 1 : 0;
  msg.interface4_state = interface_states[4] ? 1 : 0;
  msg.interface5_state = interface_states[5] ? 1 : 0;
  msg.interface6_state = interface_states[6] ? 1 : 0;
  msg.overall_interface_state = overall_state ? 1 : 0;
  msg.count16 = 0;
  msg.crc = 0;

  uint8_t buffer[CAN_MAX_DLC];
  hpo_heartbeat_heartbeat_pack(buffer, &msg, sizeof(buffer));
  msg.crc = polymath::sygnal::generate_crc8(buffer);
  hpo_heartbeat_heartbeat_pack(buffer, &msg, sizeof(buffer));
  return makeFrame(HPO_HEARTBEAT_HEARTBEAT_FRAME_ID, buffer, HPO_HEARTBEAT_HEARTBEAT_LENGTH);
}

polymath::socketcan::CanFrame buildHpoControlEnableResponse(uint8_t bus_address, uint8_t message_id, bool enable)
{
  hpo_control_control_enable_response_t msg;
  hpo_control_control_enable_response_init(&msg);
  msg.bus_address = bus_address;
  msg.message_id = message_id;
  msg.enable = enable ? 1 : 0;
  msg.crc = 0;

  uint8_t buffer[CAN_MAX_DLC];
  hpo_control_control_enable_response_pack(buffer, &msg, sizeof(buffer));
  msg.crc = polymath::sygnal::generate_crc8(buffer);
  hpo_control_control_enable_response_pack(buffer, &msg, sizeof(buffer));
  return makeFrame(HPO_CONTROL_CONTROL_ENABLE_RESPONSE_FRAME_ID, buffer, HPO_CONTROL_CONTROL_ENABLE_RESPONSE_LENGTH);
}

polymath::socketcan::CanFrame buildHpoControlCommandResponse(uint8_t bus_address, uint8_t message_id, double value)
{
  hpo_control_control_command_response_t msg;
  hpo_control_control_command_response_init(&msg);
  msg.bus_address = bus_address;
  msg.message_id = message_id;
  msg.count8 = 0;
  msg.value = hpo_control_control_command_response_value_encode(value);
  msg.crc = 0;

  uint8_t buffer[CAN_MAX_DLC];
  hpo_control_control_command_response_pack(buffer, &msg, sizeof(buffer));
  msg.crc = polymath::sygnal::generate_crc8(buffer);
  hpo_control_control_command_response_pack(buffer, &msg, sizeof(buffer));
  return makeFrame(HPO_CONTROL_CONTROL_COMMAND_RESPONSE_FRAME_ID, buffer, HPO_CONTROL_CONTROL_COMMAND_RESPONSE_LENGTH);
}

polymath::socketcan::CanFrame buildHpoErrorStatus(
  uint8_t bus_address,
  uint8_t subsystem_id,
  uint8_t error_type,
  uint16_t error_can_id,
  uint8_t section_id,
  uint8_t interface_id)
{
  hpo_error_error_status_t msg;
  hpo_error_error_status_init(&msg);
  msg.bus_address = bus_address;
  msg.subsystem_id = subsystem_id;
  msg.config_section_id = section_id;
  msg.config_interface_id = interface_id;
  msg.error_type = error_type;
  msg.error_canid = error_can_id;
  msg.crc = 0;

  uint8_t buffer[CAN_MAX_DLC];
  hpo_error_error_status_pack(buffer, &msg, sizeof(buffer));
  msg.crc = polymath::sygnal::generate_crc8(buffer);
  hpo_error_error_status_pack(buffer, &msg, sizeof(buffer));
  return makeFrame(HPO_ERROR_ERROR_STATUS_FRAME_ID, buffer, HPO_ERROR_ERROR_STATUS_LENGTH);
}

}  // namespace

TEST_CASE("SygnalHpoInterface default constructor zeroes interface bits", "[sygnal_hpo_interface]")
{
  SygnalHpoInterface hpo;
  REQUIRE(hpo.get_bus_address() == 0);
  REQUIRE_FALSE(hpo.get_overall_interface_state());
  for (bool state : hpo.get_interface_states()) {
    REQUIRE_FALSE(state);
  }
}

TEST_CASE("SygnalHpoInterface explicit constructor sets bus address", "[sygnal_hpo_interface]")
{
  SygnalHpoInterface hpo(TEST_BUS_ADDRESS);
  REQUIRE(hpo.get_bus_address() == TEST_BUS_ADDRESS);
  REQUIRE_FALSE(hpo.get_overall_interface_state());
  for (bool state : hpo.get_interface_states()) {
    REQUIRE_FALSE(state);
  }
}

TEST_CASE("SygnalHpoInterface parses heartbeat with all interfaces under HPO control", "[sygnal_hpo_interface]")
{
  SygnalHpoInterface hpo(TEST_BUS_ADDRESS);
  std::array<bool, HPO_NUM_INTERFACES> interfaces{true, true, true, true, true, true, true};
  auto frame = buildHpoHeartbeat(TEST_BUS_ADDRESS, interfaces, true);

  REQUIRE(hpo.parseHeartbeatFrame(frame));
  REQUIRE(hpo.get_overall_interface_state());
  for (bool state : hpo.get_interface_states()) {
    REQUIRE(state);
  }
}

TEST_CASE("SygnalHpoInterface parses heartbeat with mixed interface bits", "[sygnal_hpo_interface]")
{
  SygnalHpoInterface hpo(TEST_BUS_ADDRESS);
  std::array<bool, HPO_NUM_INTERFACES> interfaces{true, false, true, false, true, false, true};
  auto frame = buildHpoHeartbeat(TEST_BUS_ADDRESS, interfaces, false);

  REQUIRE(hpo.parseHeartbeatFrame(frame));
  REQUIRE_FALSE(hpo.get_overall_interface_state());
  auto states = hpo.get_interface_states();
  for (size_t i = 0; i < HPO_NUM_INTERFACES; ++i) {
    REQUIRE(states[i] == interfaces[i]);
  }
}

TEST_CASE("SygnalHpoInterface rejects heartbeat with wrong frame ID", "[sygnal_hpo_interface]")
{
  SygnalHpoInterface hpo(TEST_BUS_ADDRESS);
  std::array<bool, HPO_NUM_INTERFACES> interfaces{true, true, true, true, true, true, true};
  auto frame = buildHpoHeartbeat(TEST_BUS_ADDRESS, interfaces, true);
  frame.set_can_id(0x999);

  REQUIRE_FALSE(hpo.parseHeartbeatFrame(frame));
  REQUIRE_FALSE(hpo.get_overall_interface_state());
}

TEST_CASE("SygnalHpoInterface rejects heartbeat with bad CRC", "[sygnal_hpo_interface]")
{
  SygnalHpoInterface hpo(TEST_BUS_ADDRESS);
  std::array<bool, HPO_NUM_INTERFACES> interfaces{true, true, true, true, true, true, true};
  auto frame = buildHpoHeartbeat(TEST_BUS_ADDRESS, interfaces, true);

  // Corrupt the CRC byte.
  auto bytes = frame.get_data();
  bytes[7] ^= 0xFF;
  frame.set_data(bytes);

  REQUIRE_FALSE(hpo.parseHeartbeatFrame(frame));
}

TEST_CASE("SygnalHpoInterface rejects heartbeat addressed to a different bus", "[sygnal_hpo_interface]")
{
  SygnalHpoInterface hpo(TEST_BUS_ADDRESS);
  std::array<bool, HPO_NUM_INTERFACES> interfaces{true, true, true, true, true, true, true};
  auto frame = buildHpoHeartbeat(OTHER_BUS_ADDRESS, interfaces, true);

  REQUIRE_FALSE(hpo.parseHeartbeatFrame(frame));
  REQUIRE_FALSE(hpo.get_overall_interface_state());
}

TEST_CASE("SygnalHpoInterface creates a ControlEnable frame addressed to itself", "[sygnal_hpo_interface]")
{
  SygnalHpoInterface hpo(TEST_BUS_ADDRESS);
  std::string error_message;

  auto frame_opt = hpo.createControlEnableFrame(0x42, true, error_message);
  REQUIRE(frame_opt.has_value());
  REQUIRE(error_message.empty());

  auto frame = *frame_opt;
  REQUIRE(frame.get_id() == HPO_CONTROL_CONTROL_ENABLE_FRAME_ID);
  REQUIRE(frame.get_len() == HPO_CONTROL_CONTROL_ENABLE_LENGTH);

  auto bytes = frame.get_data();
  REQUIRE(polymath::sygnal::check_crc8(reinterpret_cast<uint8_t *>(bytes.data())));

  hpo_control_control_enable_t unpacked;
  hpo_control_control_enable_init(&unpacked);
  REQUIRE(
    hpo_control_control_enable_unpack(
      &unpacked, reinterpret_cast<uint8_t *>(bytes.data()), HPO_CONTROL_CONTROL_ENABLE_LENGTH) == 0);

  REQUIRE(unpacked.bus_address == TEST_BUS_ADDRESS);
  REQUIRE(unpacked.message_id == 0x42);
  REQUIRE(unpacked.enable == 1);
}

TEST_CASE("SygnalHpoInterface creates a ControlCommand frame addressed to itself", "[sygnal_hpo_interface]")
{
  SygnalHpoInterface hpo(TEST_BUS_ADDRESS);
  std::string error_message;

  auto frame_opt = hpo.createControlCommandFrame(0x07, 0.5, error_message);
  REQUIRE(frame_opt.has_value());
  REQUIRE(error_message.empty());

  auto frame = *frame_opt;
  REQUIRE(frame.get_id() == HPO_CONTROL_CONTROL_COMMAND_FRAME_ID);
  REQUIRE(frame.get_len() == HPO_CONTROL_CONTROL_COMMAND_LENGTH);

  auto bytes = frame.get_data();
  REQUIRE(polymath::sygnal::check_crc8(reinterpret_cast<uint8_t *>(bytes.data())));

  hpo_control_control_command_t unpacked;
  hpo_control_control_command_init(&unpacked);
  REQUIRE(
    hpo_control_control_command_unpack(
      &unpacked, reinterpret_cast<uint8_t *>(bytes.data()), HPO_CONTROL_CONTROL_COMMAND_LENGTH) == 0);

  REQUIRE(unpacked.bus_address == TEST_BUS_ADDRESS);
  REQUIRE(unpacked.message_id == 0x07);
  REQUIRE(unpacked.count8 == 0);
  const double decoded_value = hpo_control_control_command_value_decode(unpacked.value);
  REQUIRE(std::fabs(decoded_value - 0.5) < 1e-6);
}

TEST_CASE("SygnalHpoInterface parses ControlEnableResponse", "[sygnal_hpo_interface]")
{
  SygnalHpoInterface hpo(TEST_BUS_ADDRESS);
  auto frame = buildHpoControlEnableResponse(TEST_BUS_ADDRESS, 0x42, true);

  auto response = hpo.parseControlResponse(frame);
  REQUIRE(response.has_value());
  REQUIRE(response->is_enable_response);
  REQUIRE(response->bus_address == TEST_BUS_ADDRESS);
  REQUIRE(response->message_id == 0x42);
  REQUIRE(response->enable);
}

TEST_CASE("SygnalHpoInterface parses ControlCommandResponse", "[sygnal_hpo_interface]")
{
  SygnalHpoInterface hpo(TEST_BUS_ADDRESS);
  auto frame = buildHpoControlCommandResponse(TEST_BUS_ADDRESS, 0x07, 0.75);

  auto response = hpo.parseControlResponse(frame);
  REQUIRE(response.has_value());
  REQUIRE_FALSE(response->is_enable_response);
  REQUIRE(response->bus_address == TEST_BUS_ADDRESS);
  REQUIRE(response->message_id == 0x07);
  REQUIRE(std::fabs(response->value - 0.75) < 1e-6);
}

TEST_CASE("SygnalHpoInterface rejects control response for a different bus", "[sygnal_hpo_interface]")
{
  SygnalHpoInterface hpo(TEST_BUS_ADDRESS);
  auto frame = buildHpoControlEnableResponse(OTHER_BUS_ADDRESS, 0x42, true);

  REQUIRE_FALSE(hpo.parseControlResponse(frame).has_value());
}

TEST_CASE("SygnalHpoInterface rejects control response with unrelated frame ID", "[sygnal_hpo_interface]")
{
  SygnalHpoInterface hpo(TEST_BUS_ADDRESS);
  auto frame = buildHpoControlEnableResponse(TEST_BUS_ADDRESS, 0x42, true);
  frame.set_can_id(0x999);

  REQUIRE_FALSE(hpo.parseControlResponse(frame).has_value());
}

TEST_CASE("SygnalHpoInterface rejects control response with bad CRC", "[sygnal_hpo_interface]")
{
  SygnalHpoInterface hpo(TEST_BUS_ADDRESS);
  auto frame = buildHpoControlCommandResponse(TEST_BUS_ADDRESS, 0x07, 0.5);
  auto bytes = frame.get_data();
  bytes[7] ^= 0xFF;
  frame.set_data(bytes);

  REQUIRE_FALSE(hpo.parseControlResponse(frame).has_value());
}

TEST_CASE("SygnalHpoInterface parses ErrorStatus", "[sygnal_hpo_interface]")
{
  SygnalHpoInterface hpo(TEST_BUS_ADDRESS);
  auto frame = buildHpoErrorStatus(TEST_BUS_ADDRESS, 1, 5, 0x1234, 6, 2);

  auto err = hpo.parseErrorFrame(frame);
  REQUIRE(err.has_value());
  REQUIRE(err->bus_address == TEST_BUS_ADDRESS);
  REQUIRE(err->subsystem_id == 1);
  REQUIRE(err->error_type == 5);
  REQUIRE(err->error_can_id == 0x1234);
  REQUIRE(err->config_section_id == 6);
  REQUIRE(err->config_interface_id == 2);
}

TEST_CASE("SygnalHpoInterface rejects error frame for a different bus", "[sygnal_hpo_interface]")
{
  SygnalHpoInterface hpo(TEST_BUS_ADDRESS);
  auto frame = buildHpoErrorStatus(OTHER_BUS_ADDRESS, 1, 5, 0x1234, 6, 2);

  REQUIRE_FALSE(hpo.parseErrorFrame(frame).has_value());
}

TEST_CASE("SygnalHpoInterface rejects error frame with bad CRC", "[sygnal_hpo_interface]")
{
  SygnalHpoInterface hpo(TEST_BUS_ADDRESS);
  auto frame = buildHpoErrorStatus(TEST_BUS_ADDRESS, 1, 5, 0x1234, 6, 2);
  auto bytes = frame.get_data();
  bytes[7] ^= 0xFF;
  frame.set_data(bytes);

  REQUIRE_FALSE(hpo.parseErrorFrame(frame).has_value());
}
