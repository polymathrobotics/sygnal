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

/// @brief One CB board: CB_INTERFACE endpoints, where endpoint_id is the CB message ID (0-255).
///        Adds CB-specific handling for command values.
///        The CB reports no per-interface control state, so get_interface_control_state() throws.
///        Receives frames with either subsystem bit value.
class SygnalCbDevice : public SygnalDevice
{
public:
  /// @param socketcan_adapter Adapter that commands go out on.
  /// @param device CB descriptor: bus_id.
  /// @param command_timeout How long a command waits for its response before its promise is dropped.
  SygnalCbDevice(
    std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter,
    const SygnalControlDevice & device,
    std::chrono::steady_clock::duration command_timeout);

  /// @brief Latest command response value of a CB_INTERFACE endpoint; 0 until the first.
  /// @throws std::invalid_argument for type != CB_INTERFACE.
  Stamped<double> get_control_command_value(const SygnalEndpoint & endpoint) const override;

  /// @brief Enable or disable control of a CB_INTERFACE endpoint.
  ControlEnableCommandResult send_control_enable_command(
    const SygnalEndpoint & endpoint, bool enable, bool expect_reply) override;

  /// @brief Set a CB_INTERFACE endpoint's value.
  ControlCommandResult send_control_command(const SygnalEndpoint & endpoint, double value, bool expect_reply) override;

private:
  /// @brief Register callbacks for handling incoming CAN messages.
  ///        This defines the logic for handling all CB CAN messages.
  void initializeCANHandlerCallbacks();

  /// @brief State of the CB values based on the last command response, indexed by message ID.
  std::array<Stamped<double>, UINT8_MAX + 1> values_{};
};

}  // namespace polymath::sygnal
