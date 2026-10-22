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
#include <cstdint>
#include <memory>

#include "sygnal_can_interface_lib/sygnal_device.hpp"

namespace polymath::sygnal
{

/// @brief Max number of interfaces an IO board reports in its heartbeat.
constexpr uint8_t IO_NUM_INTERFACES = 7;

/// @brief One IO board: IO_INTERFACE endpoints (interface ID 0-6).
///        Adds IO-specific handling for interface states and command values.
///        Receives frames with either subsystem bit value.
class SygnalIoDevice : public SygnalDevice
{
public:
  /// @param socketcan_adapter Adapter that commands go out on.
  /// @param device IO descriptor: bus_id.
  /// @param command_timeout How long a command waits for its response before its promise is dropped.
  SygnalIoDevice(
    std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter,
    const SygnalControlDevice & device,
    std::chrono::steady_clock::duration command_timeout);

  /// @brief State of an IO_INTERFACE endpoint, from the latest heartbeat or enable response; FAIL_HARD until the first.
  /// @throws std::invalid_argument for type != IO_INTERFACE, or an interface ID of 7 or above.
  Stamped<SygnalSystemState> get_interface_control_state(const SygnalEndpoint & endpoint) const override;

  /// @brief Latest command response value of an IO_INTERFACE endpoint; 0 until the first.
  /// @throws std::invalid_argument for type != IO_INTERFACE, or an interface ID of 7 or above.
  Stamped<double> get_control_command_value(const SygnalEndpoint & endpoint) const override;

  /// @brief Enable or disable control of an IO_INTERFACE endpoint.
  /// @throws std::invalid_argument for an interface ID of 7 or above.
  ControlEnableCommandResult send_control_enable_command(
    const SygnalEndpoint & endpoint, bool enable, bool expect_reply) override;

  /// @brief Set an IO_INTERFACE endpoint's value.
  /// @throws std::invalid_argument for an interface ID of 7 or above.
  ControlCommandResult send_control_command(const SygnalEndpoint & endpoint, double value, bool expect_reply) override;

private:
  /// @brief Register callbacks for handling incoming CAN messages.
  ///        This defines the logic for handling all IO CAN messages.
  void initializeCANHandlerCallbacks();

  /// @brief State of the IO interfaces based on the last heartbeat or enable response.
  std::array<Stamped<SygnalSystemState>, IO_NUM_INTERFACES> interface_states_;

  /// @brief State of the IO interface values based on the last command response.
  std::array<Stamped<double>, IO_NUM_INTERFACES> values_{};
};

}  // namespace polymath::sygnal
