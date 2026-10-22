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
#include <optional>
#include <string>

#include "sygnal_can_interface_lib/sygnal_device.hpp"

namespace polymath::sygnal
{

/// @brief Max number of interfaces an HPO reports in its heartbeat.
constexpr uint8_t HPO_NUM_INTERFACES = 4;

/// @brief One HPO board: HPO_INTERFACE endpoints, where endpoint_id is the HPO message ID (0-255).
///        Adds HPO-specific handling for interface states and command values.
///        Interface state is only reported for message IDs 0-3.
///        Receives frames with either subsystem bit value.
class SygnalHpoDevice : public SygnalDevice
{
public:
  /// @param socketcan_adapter Adapter that commands go out on.
  /// @param device HPO descriptor: bus_id.
  /// @param command_timeout How long a command waits for its response before its promise is dropped.
  SygnalHpoDevice(
    std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter,
    const SygnalControlDevice & device,
    std::chrono::steady_clock::duration command_timeout);

  /// @brief State of an HPO_INTERFACE endpoint, from the latest heartbeat or enable response; FAIL_HARD until the first.
  /// @throws std::invalid_argument for type != HPO_INTERFACE, or a message ID of 4 or above.
  Stamped<SygnalSystemState> get_interface_control_state(const SygnalEndpoint & endpoint) const override;

  /// @brief Latest value of an HPO_INTERFACE endpoint; 0 until the first.
  ///        Message IDs 0-3 also take the heartbeat's pin state: 0 HiZ, 1 Pull_Down, 2 Pull_Up.
  ///        Message IDs 4-255 come from command responses only.
  /// @throws std::invalid_argument for type != HPO_INTERFACE.
  Stamped<double> get_control_command_value(const SygnalEndpoint & endpoint) const override;

  /// @brief Enable or disable control of an HPO_INTERFACE endpoint.
  ControlEnableCommandResult send_control_enable_command(
    const SygnalEndpoint & endpoint, bool enable, bool expect_reply) override;

  /// @brief Set an HPO_INTERFACE endpoint's value.
  ControlCommandResult send_control_command(const SygnalEndpoint & endpoint, double value, bool expect_reply) override;

private:
  /// @brief Register callbacks for handling incoming CAN messages.
  ///        This defines the logic for handling all HPO CAN messages.
  void initializeCANHandlerCallbacks();

  /// @brief State of the HPO interfaces based on the last heartbeat or enable response.
  std::array<Stamped<SygnalSystemState>, HPO_NUM_INTERFACES> interface_states_;

  /// @brief State of the HPO values based on the last heartbeat or command response, indexed by message ID.
  std::array<Stamped<double>, UINT8_MAX + 1> values_{};
};

}  // namespace polymath::sygnal
