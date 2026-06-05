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
#include <chrono>
#include <string>

#include "sygnal_can_interface_lib/crc8.hpp"
#include "sygnal_dbc/hpo_control.h"
#include "sygnal_dbc/hpo_error.h"
#include "sygnal_dbc/hpo_heartbeat.h"

namespace polymath::sygnal
{

SygnalHpoInterface::SygnalHpoInterface()
: bus_address_(0)
{
  hpo_interface_states_.fill(false);
}

SygnalHpoInterface::SygnalHpoInterface(uint8_t bus_address)
: bus_address_(bus_address)
{
  hpo_interface_states_.fill(false);
}

bool SygnalHpoInterface::parseHeartbeatFrame(const socketcan::CanFrame & frame)
{
  if (HPO_HEARTBEAT_HEARTBEAT_FRAME_ID != frame.get_id()) {
    return false;
  }

  auto frame_copy = frame.get_frame();

  if (!check_crc8(reinterpret_cast<uint8_t *>(frame_copy.data))) {
    return false;
  }

  hpo_heartbeat_heartbeat_t unpacked;
  if (0 != hpo_heartbeat_heartbeat_init(&unpacked)) {
    return false;
  }

  if (0 != hpo_heartbeat_heartbeat_unpack(&unpacked, frame_copy.data, frame_copy.len)) {
    return false;
  }

  if (bus_address_ != unpacked.bus_address) {
    return false;
  }

  last_heartbeat_timestamp_ = std::chrono::system_clock::now();

  hpo_interface_states_[0] = (0 != unpacked.interface0_state);
  hpo_interface_states_[1] = (0 != unpacked.interface1_state);
  hpo_interface_states_[2] = (0 != unpacked.interface2_state);
  hpo_interface_states_[3] = (0 != unpacked.interface3_state);
  // Interface{0-3}Value (2-bit pin-pull: HiZ/Pull_Down/Pull_Up) are also in this frame, not yet surfaced.

  return true;
}

std::optional<HpoControlResponse> SygnalHpoInterface::parseControlResponse(const socketcan::CanFrame & frame)
{
  auto frame_copy = frame.get_frame();
  const uint32_t frame_id = frame.get_id();

  if (
    HPO_CONTROL_CONTROL_ENABLE_RESPONSE_FRAME_ID != frame_id &&
    HPO_CONTROL_CONTROL_COMMAND_RESPONSE_FRAME_ID != frame_id)
  {
    return std::nullopt;
  }

  if (!check_crc8(reinterpret_cast<uint8_t *>(frame_copy.data))) {
    return std::nullopt;
  }

  HpoControlResponse response{};

  if (HPO_CONTROL_CONTROL_ENABLE_RESPONSE_FRAME_ID == frame_id) {
    hpo_control_control_enable_response_t unpacked;
    if (0 != hpo_control_control_enable_response_init(&unpacked)) {
      return std::nullopt;
    }
    if (0 != hpo_control_control_enable_response_unpack(&unpacked, frame_copy.data, frame_copy.len)) {
      return std::nullopt;
    }
    if (bus_address_ != unpacked.bus_address) {
      return std::nullopt;
    }

    response.is_enable_response = true;
    response.bus_address = unpacked.bus_address;
    response.message_id = unpacked.message_id;
    response.enable = (0 != unpacked.enable);
    response.value = 0.0;
    return response;
  }

  hpo_control_control_command_response_t unpacked;
  if (0 != hpo_control_control_command_response_init(&unpacked)) {
    return std::nullopt;
  }
  if (0 != hpo_control_control_command_response_unpack(&unpacked, frame_copy.data, frame_copy.len)) {
    return std::nullopt;
  }
  if (bus_address_ != unpacked.bus_address) {
    return std::nullopt;
  }

  response.is_enable_response = false;
  response.bus_address = unpacked.bus_address;
  response.message_id = unpacked.message_id;
  response.value = hpo_control_control_command_response_value_decode(unpacked.value);
  response.enable = false;
  return response;
}

std::optional<HpoErrorStatus> SygnalHpoInterface::parseErrorFrame(const socketcan::CanFrame & frame)
{
  if (HPO_ERROR_ERROR_STATUS_FRAME_ID != frame.get_id()) {
    return std::nullopt;
  }

  auto frame_copy = frame.get_frame();

  if (!check_crc8(reinterpret_cast<uint8_t *>(frame_copy.data))) {
    return std::nullopt;
  }

  hpo_error_error_status_t unpacked;
  if (0 != hpo_error_error_status_init(&unpacked)) {
    return std::nullopt;
  }
  if (0 != hpo_error_error_status_unpack(&unpacked, frame_copy.data, frame_copy.len)) {
    return std::nullopt;
  }
  if (bus_address_ != unpacked.bus_address) {
    return std::nullopt;
  }

  HpoErrorStatus status{};
  status.bus_address = unpacked.bus_address;
  status.subsystem_id = unpacked.subsystem_id;
  status.error_type = unpacked.error_type;
  status.error_can_id = unpacked.error_canid;
  status.config_section_id = unpacked.config_section_id;
  status.config_interface_id = unpacked.config_interface_id;
  return status;
}

std::optional<socketcan::CanFrame> SygnalHpoInterface::createControlEnableFrame(
  uint8_t message_id, bool enable, std::string & error_message)
{
  polymath::socketcan::CanFrame frame;
  uint8_t buffer[CAN_MAX_DLC];
  hpo_control_control_enable_t unpacked;

  if (0 != hpo_control_control_enable_init(&unpacked)) {
    error_message += "Unable to initialize hpo_control_control_enable_t. \n";
    return std::nullopt;
  }

  unpacked.bus_address = bus_address_;
  unpacked.message_id = message_id;
  unpacked.enable = enable ? 1 : 0;
  unpacked.crc = 0;

  if (!hpo_control_control_enable_pack(buffer, &unpacked, sizeof(buffer))) {
    error_message += "Could not pack hpo_control_control_enable_t. \n";
    return std::nullopt;
  }

  unpacked.crc = generate_crc8(buffer);
  if (!hpo_control_control_enable_pack(buffer, &unpacked, sizeof(buffer))) {
    error_message += "Could not pack hpo_control_control_enable_t. \n";
    return std::nullopt;
  }

  std::array<unsigned char, CAN_MAX_DLC> packed_data;
  for (size_t i = 0; i < packed_data.size(); ++i) {
    packed_data[i] = buffer[i];
  }

  frame.set_can_id(HPO_CONTROL_CONTROL_ENABLE_FRAME_ID);
  frame.set_len(HPO_CONTROL_CONTROL_ENABLE_LENGTH);
  frame.set_data(packed_data);

  return frame;
}

std::optional<socketcan::CanFrame> SygnalHpoInterface::createControlCommandFrame(
  uint8_t message_id, double value, std::string & error_message)
{
  polymath::socketcan::CanFrame frame;
  uint8_t buffer[CAN_MAX_DLC];
  hpo_control_control_command_t unpacked;

  if (0 != hpo_control_control_command_init(&unpacked)) {
    error_message += "Unable to initialize hpo_control_control_command_t. \n";
    return std::nullopt;
  }

  unpacked.bus_address = bus_address_;
  unpacked.message_id = message_id;
  // Count8 rolling counter is hard-coded to 0 for now (design doc open question #1).
  unpacked.count8 = hpo_control_control_command_count8_encode(0.0);
  unpacked.value = hpo_control_control_command_value_encode(value);
  unpacked.crc = 0;

  if (!hpo_control_control_command_pack(buffer, &unpacked, sizeof(buffer))) {
    error_message += "Could not pack hpo_control_control_command_t. \n";
    return std::nullopt;
  }

  unpacked.crc = generate_crc8(buffer);
  if (!hpo_control_control_command_pack(buffer, &unpacked, sizeof(buffer))) {
    error_message += "Could not pack hpo_control_control_command_t. \n";
    return std::nullopt;
  }

  std::array<unsigned char, CAN_MAX_DLC> packed_data;
  for (size_t i = 0; i < packed_data.size(); ++i) {
    packed_data[i] = buffer[i];
  }

  frame.set_can_id(HPO_CONTROL_CONTROL_COMMAND_FRAME_ID);
  frame.set_len(HPO_CONTROL_CONTROL_COMMAND_LENGTH);
  frame.set_data(packed_data);

  return frame;
}

}  // namespace polymath::sygnal
