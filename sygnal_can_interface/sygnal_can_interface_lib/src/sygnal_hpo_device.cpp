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

#include "sygnal_can_interface_lib/sygnal_hpo_device.hpp"

#include <array>
#include <chrono>
#include <string>

#include "hpo_control/hpo_control.hpp"
#include "hpo_error/hpo_error.hpp"
#include "hpo_heartbeat/hpo_heartbeat.hpp"

namespace polymath::sygnal
{

SygnalHpoDevice::SygnalHpoDevice()
: SygnalHpoDevice(0)
{}

SygnalHpoDevice::SygnalHpoDevice(uint8_t bus_address)
: SygnalDevice(bus_address)
{
  hpo_interface_states_.fill(false);
}

bool SygnalHpoDevice::parseHeartbeatFrame(const socketcan::CanFrame & frame)
{
  if (hpo_heartbeat::Heartbeat::Id != frame.get_id() || !is_valid_frame(frame)) {
    return false;
  }

  const hpo_heartbeat::Heartbeat msg(frame.get_frame());
  if (get_bus_address() != static_cast<uint8_t>(msg.bus_address)) {
    return false;
  }

  last_heartbeat_timestamp_ = std::chrono::system_clock::now();

  hpo_interface_states_[0] = (0 != msg.interface0_state);
  hpo_interface_states_[1] = (0 != msg.interface1_state);
  hpo_interface_states_[2] = (0 != msg.interface2_state);
  hpo_interface_states_[3] = (0 != msg.interface3_state);
  // Interface{0-3}Value (2-bit pin-pull: HiZ/Pull_Down/Pull_Up) are also in this frame, not yet surfaced.

  return true;
}

std::optional<HpoControlResponse> SygnalHpoDevice::parseControlResponse(const socketcan::CanFrame & frame)
{
  const uint32_t frame_id = frame.get_id();

  if (hpo_control::ControlEnableResponse::Id != frame_id && hpo_control::ControlCommandResponse::Id != frame_id) {
    return std::nullopt;
  }

  if (!is_valid_frame(frame)) {
    return std::nullopt;
  }

  HpoControlResponse response{};

  if (hpo_control::ControlEnableResponse::Id == frame_id) {
    const hpo_control::ControlEnableResponse msg(frame.get_frame());
    if (get_bus_address() != static_cast<uint8_t>(msg.bus_address)) {
      return std::nullopt;
    }

    response.is_enable_response = true;
    response.bus_address = static_cast<uint8_t>(msg.bus_address);
    response.message_id = static_cast<uint8_t>(msg.message_id);
    response.enable = (0 != msg.enable);
    response.value = 0.0;
    return response;
  }

  const hpo_control::ControlCommandResponse msg(frame.get_frame());
  if (get_bus_address() != static_cast<uint8_t>(msg.bus_address)) {
    return std::nullopt;
  }

  response.is_enable_response = false;
  response.bus_address = static_cast<uint8_t>(msg.bus_address);
  response.message_id = static_cast<uint8_t>(msg.message_id);
  response.value = msg.value;
  response.enable = false;
  return response;
}

std::optional<HpoErrorStatus> SygnalHpoDevice::parseErrorFrame(const socketcan::CanFrame & frame)
{
  if (hpo_error::ErrorStatus::Id != frame.get_id() || !is_valid_frame(frame)) {
    return std::nullopt;
  }

  const hpo_error::ErrorStatus msg(frame.get_frame());
  if (get_bus_address() != static_cast<uint8_t>(msg.bus_address)) {
    return std::nullopt;
  }

  HpoErrorStatus status{};
  status.bus_address = static_cast<uint8_t>(msg.bus_address);
  status.subsystem_id = static_cast<uint8_t>(msg.subsystem_id);
  status.error_type = static_cast<uint8_t>(msg.error_type);
  status.error_can_id = static_cast<uint16_t>(msg.error_canid);
  status.config_section_id = static_cast<uint8_t>(msg.config_section_id);
  status.config_interface_id = static_cast<uint8_t>(msg.config_interface_id);
  return status;
}

std::optional<socketcan::CanFrame> SygnalHpoDevice::createControlEnableFrame(
  uint8_t message_id, bool enable, std::string &)
{
  hpo_control::ControlEnable msg;
  msg.bus_address = get_bus_address();
  msg.message_id = message_id;
  msg.enable = enable ? 1 : 0;
  return to_frame_with_crc(msg);
}

std::optional<socketcan::CanFrame> SygnalHpoDevice::createControlCommandFrame(
  uint8_t message_id, double value, std::string &)
{
  hpo_control::ControlCommand msg;
  msg.bus_address = get_bus_address();
  msg.message_id = message_id;
  // Count8 rolling counter is hard-coded to 0 for now (design doc open question #1).
  msg.count8 = 0;
  msg.value = value;
  return to_frame_with_crc(msg);
}

}  // namespace polymath::sygnal
