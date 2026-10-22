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


#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <vector>

#include "socketcan_adapter/socketcan_adapter.hpp"
#include "sygnal_can_interface_lib/sygnal_device.hpp"
#include "sygnal_can_interface_lib/sygnal_types.hpp"

namespace polymath::sygnal
{

constexpr std::chrono::seconds DEFAULT_COMMAND_TIMEOUT{1};

/// @brief A single interface for all Sygnal devices.
class SygnalInterfaceSocketcan
{
public:
  /// @param socketcan_adapter Adapter that devices send commands on.
  /// @param control_devices Devices to manage.
  /// @param command_timeout How long a command waits for its response before its promise is dropped.
  /// @throws std::invalid_argument on a bus_id above 127, an MCM subsystem_id above 1, or duplicate device addresses.
  SygnalInterfaceSocketcan(
    std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter,
    const std::vector<SygnalControlDevice> & control_devices,
    std::chrono::steady_clock::duration command_timeout = DEFAULT_COMMAND_TIMEOUT);

  /// @brief Route a received frame to the device it addresses.
  /// @return true if a configured device consumed the frame.
  bool parse(const socketcan::CanFrame & frame);

  /// @brief Get SystemState from the device's latest stashed heartbeat.
  ///        Returns FAIL_HARD until the first one.
  /// @throws std::invalid_argument if the device was not passed into to the constructor, i.e. unknown device.
  Stamped<SygnalSystemState> get_device_system_state(const SygnalControlDevice & device) const;

  /// @brief Get the endpoint's control state from the device's latest stashed heartbeat or enable response.
  /// @throws std::invalid_argument if the device is unknown or has no such endpoint.
  Stamped<SygnalSystemState> get_interface_control_state(const SygnalEndpoint & endpoint) const;

  /// @brief Value from the endpoint's latest command response; 0 until the first command response.
  /// @throws std::invalid_argument if the device is unknown or has no such endpoint.
  Stamped<double> get_control_command_value(const SygnalEndpoint & endpoint) const;

  /// @brief Fault status of the device.
  /// @throws std::invalid_argument if the device is unknown or does not report faults.
  Stamped<McmFaultStatus> get_device_faults(const SygnalControlDevice & device) const;

  /// @brief Enable or disable control of an endpoint.
  /// @param expect_reply Return a future that resolves from the response message.
  /// @return A future. Resolves immediately if expect_reply is false,
  ///         otherwise resolves from the response message.
  /// @throws std::invalid_argument if the device is unknown or does not support the endpoint.
  ControlEnableCommandResult send_control_enable_command(
    const SygnalEndpoint & endpoint, bool enable, bool expect_reply);

  /// @brief Set an endpoint's value.
  /// @param expect_reply Return a future that resolves from the endpoint's command response.
  /// @return A future. Resolves immediately if expect_reply is false,
  ///         otherwise resolves from the endpoint's command response.
  /// @throws std::invalid_argument if the device is unknown or does not support the endpoint.
  ControlCommandResult send_control_command(const SygnalEndpoint & endpoint, double value, bool expect_reply);

private:
  /// @throws std::invalid_argument if no device of device.type is configured at its address.
  SygnalDevice & device_at(const SygnalControlDevice & device) const;

  /// @brief All known devices managed passed in to the constructor.
  std::vector<std::unique_ptr<SygnalDevice>> devices_;

  /// @brief A lookup table for devices by their Bus ID ( and Subsystem ID if applicable).
  std::array<SygnalDevice *, UINT8_MAX + 1> id_to_device_{};
};

}  // namespace polymath::sygnal
