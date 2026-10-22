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

#include "sygnal_can_interface_lib/sygnal_device.hpp"

#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "sygnal_can_interface_lib/crc8.hpp"

namespace polymath::sygnal
{

SygnalDevice::SygnalDevice(
  std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter,
  SygnalControlDevice device,
  std::chrono::steady_clock::duration command_timeout)
: socketcan_adapter_(std::move(socketcan_adapter))
, command_timeout_(command_timeout)
, control_enable_responses_(command_timeout_)
, control_command_responses_(command_timeout_)
, device_(device)
{}

bool SygnalDevice::handle(
  const can_frame & frame,
  std::chrono::steady_clock::time_point receive_time,
  std::chrono::system_clock::time_point bus_time)
{
  // Reject frames that are not 8 bytes since every sygnal message puts the CRC in the 8th byte.
  // Then check the CRC of the frame.
  if (CAN_MAX_DLC != frame.len || !check_crc8(frame.data)) {
    return false;
  }

  // Pass the frame to can handler.
  return can_handler_.handle(frame, receive_time, bus_time);
}

const SygnalControlDevice & SygnalDevice::device() const
{
  return device_;
}

Stamped<SygnalSystemState> SygnalDevice::get_device_system_state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return system_state_;
}

Stamped<SygnalSystemState> SygnalDevice::get_interface_control_state(const SygnalEndpoint & endpoint) const
{
  throw std::invalid_argument(
    "Sygnal Endpoint Type " + std::string(magic_enum::enum_name(endpoint.type)) +
    " does not support Interface Control State feedback.");
}

Stamped<double> SygnalDevice::get_control_command_value(const SygnalEndpoint & endpoint) const
{
  throw std::invalid_argument(
    "Sygnal Endpoint Type " + std::string(magic_enum::enum_name(endpoint.type)) +
    " does not support Control Command Value feedback.");
}

ControlEnableCommandResult SygnalDevice::send_control_enable_command(const SygnalEndpoint & endpoint, bool, bool)
{
  throw std::invalid_argument(
    "Sygnal Endpoint Type " + std::string(magic_enum::enum_name(endpoint.type)) +
    " does not support Control Enable commands.");
}

ControlCommandResult SygnalDevice::send_control_command(const SygnalEndpoint & endpoint, double, bool)
{
  throw std::invalid_argument(
    "Sygnal Endpoint Type " + std::string(magic_enum::enum_name(endpoint.type)) +
    " does not support Control commands.");
}

}  // namespace polymath::sygnal
