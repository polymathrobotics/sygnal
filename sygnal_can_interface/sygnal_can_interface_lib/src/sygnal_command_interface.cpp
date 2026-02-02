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

#include "sygnal_can_interface_lib/crc8.hpp"
#include "sygnal_dbc/mcm_control.h"
#include "sygnal_dbc/mcm_relay.h"

namespace polymath::sygnal
{

SygnalControlInterface::SygnalControlInterface()
{}

std::optional<polymath::socketcan::CanFrame> SygnalControlInterface::createControlStateCommandFrame(
  const uint8_t bus_id,
  const uint8_t interface_id,
  const uint8_t subsystem_id,
  const SygnalControlState control_state,
  std::string & error_message)
{
  if (interface_id >= MAX_SYGNAL_INTERFACES) {
    error_message = error_message + "Interface ID exceeds maximum allowed interfaces. \n";
    return std::nullopt;
  }

  if (subsystem_id >= MAX_SYGNAL_SUBSYSTEMS) {
    error_message = error_message + "Subsystem ID exceeds maximum allowed subsystems. \n";
    return std::nullopt;
  }

  polymath::socketcan::CanFrame frame;
  uint8_t control_enable_buffer[CAN_MAX_DLC];
  mcm_control_control_enable_t mcm_unpacked_control_enable_t;

  if (mcm_control_control_enable_init(&mcm_unpacked_control_enable_t) != 0) {
    error_message = error_message + "Unable to initialize mcm_unpacked_control_enable_t. \n";
    return std::nullopt;
  }

  mcm_unpacked_control_enable_t.bus_address = bus_id;
  mcm_unpacked_control_enable_t.interface_id = interface_id;
  mcm_unpacked_control_enable_t.enable = static_cast<uint8_t>(control_state);
  mcm_unpacked_control_enable_t.crc = 0;

  if (!mcm_control_control_enable_pack(
        control_enable_buffer, &mcm_unpacked_control_enable_t, sizeof(control_enable_buffer)))
  {
    error_message = error_message + "Could not pack mcm_unpacked_control_enable_t. \n";
    return std::nullopt;
  }

  // Get checksum and pack it again
  mcm_unpacked_control_enable_t.crc = generate_crc8(control_enable_buffer);
  if (!mcm_control_control_enable_pack(
        control_enable_buffer, &mcm_unpacked_control_enable_t, sizeof(control_enable_buffer)))
  {
    error_message = error_message + "Could not pack mcm_unpacked_control_enable_t. \n";
    return std::nullopt;
  }

  std::array<unsigned char, CAN_MAX_DLC> control_enable_packed_data;
  for (size_t i = 0; i < control_enable_packed_data.size(); ++i) {
    control_enable_packed_data[i] = control_enable_buffer[i];
  }

  frame.set_can_id(MCM_CONTROL_CONTROL_ENABLE_FRAME_ID);
  frame.set_len(MCM_CONTROL_CONTROL_ENABLE_LENGTH);
  frame.set_data(control_enable_packed_data);

  return frame;
}

std::optional<polymath::socketcan::CanFrame> SygnalControlInterface::createControlCommandFrame(
  const uint8_t bus_id,
  const uint8_t interface_id,
  const uint8_t subsystem_id,
  const double value,
  std::string & error_message)
{
  if (interface_id >= MAX_SYGNAL_INTERFACES) {
    error_message = error_message + "Interface ID exceeds maximum allowed interfaces. \n";
    return std::nullopt;
  }

  if (subsystem_id >= MAX_SYGNAL_SUBSYSTEMS) {
    error_message = error_message + "Subsystem ID exceeds maximum allowed subsystems. \n";
    return std::nullopt;
  }

  polymath::socketcan::CanFrame frame;
  uint8_t control_command_buffer[CAN_MAX_DLC];
  mcm_control_control_command_t mcm_unpacked_control_command_t;

  if (mcm_control_control_command_init(&mcm_unpacked_control_command_t) != 0) {
    error_message = error_message + "Unable to initialize mcm_unpacked_control_command_t. \n";
    return std::nullopt;
  }

  mcm_unpacked_control_command_t.bus_address = bus_id;
  mcm_unpacked_control_command_t.interface_id = interface_id;
  mcm_unpacked_control_command_t.count8 = mcm_control_control_command_count8_encode(0.0);
  mcm_unpacked_control_command_t.value = mcm_control_control_command_value_encode(value);
  mcm_unpacked_control_command_t.crc = 0;

  if (!mcm_control_control_command_pack(
        control_command_buffer, &mcm_unpacked_control_command_t, sizeof(control_command_buffer)))
  {
    error_message = error_message + "Could not pack mcm_unpacked_control_command_t. \n";
    return std::nullopt;
  }

  // Get checksum and pack it again
  mcm_unpacked_control_command_t.crc = generate_crc8(control_command_buffer);
  if (!mcm_control_control_command_pack(
        control_command_buffer, &mcm_unpacked_control_command_t, sizeof(control_command_buffer)))
  {
    error_message = error_message + "Could not pack mcm_unpacked_control_command_t. \n";
    return std::nullopt;
  }

  std::array<unsigned char, CAN_MAX_DLC> control_command_packed_data;
  for (size_t i = 0; i < control_command_packed_data.size(); ++i) {
    control_command_packed_data[i] = control_command_buffer[i];
  }

  frame.set_can_id(MCM_CONTROL_CONTROL_COMMAND_FRAME_ID);
  frame.set_len(MCM_CONTROL_CONTROL_COMMAND_LENGTH);
  frame.set_data(control_command_packed_data);

  return frame;
}

std::optional<polymath::socketcan::CanFrame> SygnalControlInterface::createRelayCommandFrame(
  const uint8_t bus_id, const uint8_t subsystem_id, const bool relay_state, std::string & error_message)
{
  polymath::socketcan::CanFrame frame;
  uint8_t relay_command_buffer[CAN_MAX_DLC];
  mcm_relay_relay_command_t mcm_unpacked_relay_command_t;

  if (mcm_relay_relay_command_init(&mcm_unpacked_relay_command_t) != 0) {
    error_message = error_message + "Unable to initialize mcm_unpacked_relay_command_t. \n";
    return std::nullopt;
  }

  mcm_unpacked_relay_command_t.bus_address = bus_id;
  mcm_unpacked_relay_command_t.subsystem_id = subsystem_id;
  mcm_unpacked_relay_command_t.enable = static_cast<uint8_t>(relay_state);
  mcm_unpacked_relay_command_t.crc = 0;

  if (!mcm_relay_relay_command_pack(relay_command_buffer, &mcm_unpacked_relay_command_t, sizeof(relay_command_buffer)))
  {
    error_message = error_message + "Could not pack mcm_unpacked_relay_command_t. \n";
    return std::nullopt;
  }

  // Get checksum and pack it again
  mcm_unpacked_relay_command_t.crc = generate_crc8(relay_command_buffer);
  if (!mcm_relay_relay_command_pack(relay_command_buffer, &mcm_unpacked_relay_command_t, sizeof(relay_command_buffer)))
  {
    error_message = error_message + "Could not pack mcm_unpacked_relay_command_t. \n";
    return std::nullopt;
  }

  std::array<unsigned char, CAN_MAX_DLC> mcm_relay_command_packed_data;
  for (size_t i = 0; i < mcm_relay_command_packed_data.size(); ++i) {
    mcm_relay_command_packed_data[i] = relay_command_buffer[i];
  }

  frame.set_can_id(MCM_RELAY_RELAY_COMMAND_FRAME_ID);
  frame.set_len(MCM_RELAY_RELAY_COMMAND_LENGTH);
  frame.set_data(mcm_relay_command_packed_data);

  return frame;
}

std::optional<SygnalControlCommandResponse> SygnalControlInterface::parseCommandResponseFrame(
  const socketcan::CanFrame & frame)
{
  auto frame_copy = frame.get_frame();
  uint32_t frame_id = frame.get_id();

  // Check CRC first for all supported frame types
  if (!check_crc8(reinterpret_cast<uint8_t *>(frame_copy.data))) {
    return std::nullopt;
  }

  SygnalControlCommandResponse response;

  switch (frame_id) {
    case MCM_CONTROL_CONTROL_ENABLE_RESPONSE_FRAME_ID: {
      mcm_control_control_enable_response_t unpacked;
      if (mcm_control_control_enable_response_init(&unpacked) != 0) {
        return std::nullopt;
      }
      if (mcm_control_control_enable_response_unpack(&unpacked, frame_copy.data, frame_copy.len) != 0) {
        return std::nullopt;
      }
      response.response_type = SygnalControlCommandResponseType::ENABLE;
      response.bus_id = unpacked.bus_address;
      response.interface_id = unpacked.interface_id;
      response.value = static_cast<double>(unpacked.enable);
      return response;
    }

    case MCM_CONTROL_CONTROL_COMMAND_RESPONSE_FRAME_ID: {
      mcm_control_control_command_response_t unpacked;
      if (mcm_control_control_command_response_init(&unpacked) != 0) {
        return std::nullopt;
      }
      if (mcm_control_control_command_response_unpack(&unpacked, frame_copy.data, frame_copy.len) != 0) {
        return std::nullopt;
      }
      response.response_type = SygnalControlCommandResponseType::CONTROL;
      response.bus_id = unpacked.bus_address;
      response.interface_id = unpacked.interface_id;
      response.value = mcm_control_control_command_response_value_decode(unpacked.value);
      return response;
    }

    case MCM_RELAY_RELAY_COMMAND_RESPONSE_FRAME_ID: {
      mcm_relay_relay_command_response_t unpacked;
      if (mcm_relay_relay_command_response_init(&unpacked) != 0) {
        return std::nullopt;
      }
      if (mcm_relay_relay_command_response_unpack(&unpacked, frame_copy.data, frame_copy.len) != 0) {
        return std::nullopt;
      }
      response.response_type = SygnalControlCommandResponseType::RELAY;
      response.bus_id = unpacked.bus_address;
      response.interface_id = unpacked.subsystem_id;
      response.value = static_cast<double>(unpacked.enable);
      return response;
    }

    default:
      return std::nullopt;
  }
}

}  // namespace polymath::sygnal
