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

#include <chrono>
#include <string>

#include "mcm_control/mcm_control.hpp"
#include "mcm_heartbeat/mcm_heartbeat.hpp"
#include "mcm_relay/mcm_relay.hpp"

namespace polymath::sygnal
{

SygnalMcmDevice::SygnalMcmDevice()
: SygnalMcmDevice(0, 0)
{}

SygnalMcmDevice::SygnalMcmDevice(const uint8_t bus_address, const uint8_t subsystem_id)
: SygnalDevice(bus_address)
, subsystem_id_(subsystem_id)
, sygnal_mcm_state_(SygnalSystemState::FAIL_HARD)
{
  sygnal_interface_states_.fill(SygnalSystemState::FAIL_HARD);
}

bool SygnalMcmDevice::parseMcmHeartbeatFrame(const socketcan::CanFrame & frame)
{
  if (mcm_heartbeat::Heartbeat::Id != frame.get_id() || !is_valid_frame(frame)) {
    return false;
  }

  const mcm_heartbeat::Heartbeat msg(frame.get_frame());

  // Only parse if this frame is for our subsystem
  if (subsystem_id_ != static_cast<uint8_t>(msg.subsystem_id)) {
    return false;
  }

  if (get_bus_address() != static_cast<uint8_t>(msg.bus_address)) {
    return false;
  }

  last_heartbeat_timestamp_ = std::chrono::system_clock::now();

  // Set the MCM state
  sygnal_mcm_state_ = static_cast<SygnalSystemState>(msg.system_state);

  // Get interface states
  sygnal_interface_states_[0] = static_cast<SygnalSystemState>(msg.interface0_state);
  sygnal_interface_states_[1] = static_cast<SygnalSystemState>(msg.interface1_state);
  sygnal_interface_states_[2] = static_cast<SygnalSystemState>(msg.interface2_state);
  sygnal_interface_states_[3] = static_cast<SygnalSystemState>(msg.interface3_state);
  sygnal_interface_states_[4] = static_cast<SygnalSystemState>(msg.interface4_state);

  return true;
}

std::optional<socketcan::CanFrame> SygnalMcmDevice::createControlStateCommandFrame(
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

  mcm_control::ControlEnable msg;
  msg.bus_address = bus_id;
  msg.sub_system_id = subsystem_id;
  msg.interface_id = interface_id;
  msg.enable = static_cast<uint8_t>(control_state);
  return to_frame_with_crc(msg);
}

std::optional<socketcan::CanFrame> SygnalMcmDevice::createControlCommandFrame(
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

  mcm_control::ControlCommand msg;
  msg.bus_address = bus_id;
  msg.sub_system_id = subsystem_id;
  msg.interface_id = interface_id;
  msg.count8 = 0;
  msg.value = value;
  return to_frame_with_crc(msg);
}

std::optional<socketcan::CanFrame> SygnalMcmDevice::createRelayCommandFrame(
  const uint8_t bus_id, const uint8_t subsystem_id, const bool relay_state, std::string &)
{
  mcm_relay::RelayCommand msg;
  msg.bus_address = bus_id;
  msg.subsystem_id = subsystem_id;
  msg.enable = relay_state ? 1 : 0;
  return to_frame_with_crc(msg);
}

std::optional<SygnalControlCommandResponse> SygnalMcmDevice::parseCommandResponseFrame(
  const socketcan::CanFrame & frame)
{
  if (!is_valid_frame(frame)) {
    return std::nullopt;
  }

  SygnalControlCommandResponse response;

  switch (frame.get_id()) {
    case mcm_control::ControlEnableResponse::Id: {
      const mcm_control::ControlEnableResponse msg(frame.get_frame());
      response.response_type = SygnalControlCommandResponseType::ENABLE;
      response.bus_id = static_cast<uint8_t>(msg.bus_address);
      response.subsystem_id = static_cast<uint8_t>(msg.sub_system_id);
      response.interface_id = static_cast<uint8_t>(msg.interface_id);
      response.value = msg.enable;
      return response;
    }

    case mcm_control::ControlCommandResponse::Id: {
      const mcm_control::ControlCommandResponse msg(frame.get_frame());
      response.response_type = SygnalControlCommandResponseType::CONTROL;
      response.bus_id = static_cast<uint8_t>(msg.bus_address);
      response.subsystem_id = static_cast<uint8_t>(msg.sub_system_id);
      response.interface_id = static_cast<uint8_t>(msg.interface_id);
      response.value = msg.value;
      return response;
    }

    case mcm_relay::RelayCommandResponse::Id: {
      const mcm_relay::RelayCommandResponse msg(frame.get_frame());
      response.response_type = SygnalControlCommandResponseType::RELAY;
      response.bus_id = static_cast<uint8_t>(msg.bus_address);
      response.subsystem_id = static_cast<uint8_t>(msg.subsystem_id);
      response.interface_id = static_cast<uint8_t>(msg.subsystem_id);
      response.value = msg.enable;
      return response;
    }

    default:
      return std::nullopt;
  }
}

}  // namespace polymath::sygnal
