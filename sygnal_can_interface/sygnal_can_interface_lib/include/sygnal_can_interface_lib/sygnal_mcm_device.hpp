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
#include <mutex>
#include <optional>
#include <string>

#include "sygnal_can_interface_lib/sygnal_device.hpp"

namespace polymath::sygnal
{

/// @brief Max number of Interfaces an MCM subsystem can have.
constexpr uint8_t MCM_NUM_INTERFACES = 7;

/// @brief One MCM subsystem: MCM_INTERFACE endpoints (interface ID 0-6) and one MCM_RELAY endpoint.
///        Adds MCM-specific handling for interfaces, relays, and fault states.
class SygnalMcmDevice : public SygnalDevice
{
public:
  /// @param socketcan_adapter Adapter that commands go out on.
  /// @param device MCM descriptor: bus_id and subsystem_id.
  /// @param command_timeout How long a command waits for its response before its promise is dropped.
  SygnalMcmDevice(
    std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter,
    const SygnalControlDevice & device,
    std::chrono::steady_clock::duration command_timeout);

  /// @brief MCM_CONTROL or HUMAN_CONTROL for an MCM_INTERFACE endpoint, from the latest heartbeat or enable response.
  /// @throws std::invalid_argument for type != MCM_INTERFACE, or an interface ID of 7 or above.
  Stamped<SygnalSystemState> get_interface_control_state(const SygnalEndpoint & endpoint) const override;

  /// @brief Latest command response value of an MCM_INTERFACE endpoint, or the relay state of MCM_RELAY; 0 until the first.
  /// @throws std::invalid_argument for type != MCM_INTERFACE or MCM_RELAY, or an interface ID of 7 or above.
  Stamped<double> get_control_command_value(const SygnalEndpoint & endpoint) const override;

  /// @brief Enable or disable control of an MCM_INTERFACE endpoint.
  ControlEnableCommandResult send_control_enable_command(
    const SygnalEndpoint & endpoint, bool enable, bool expect_reply) override;

  /// @brief Set an MCM_INTERFACE endpoint's value, or close the MCM_RELAY on a nonzero value and open it on zero.
  ControlCommandResult send_control_command(const SygnalEndpoint & endpoint, double value, bool expect_reply) override;

  /// @brief Fault status; timestamp is the latest of the fault messages and the heartbeat.
  Stamped<McmFaultStatus> get_device_faults() const override;

protected:
  /// @brief Pending relay replies, always on key 0; RelayCommandResponse carries no ID.
  ///        Kept separate from interface responses to avoid fulfilling the wrong promises.
  ResponseQueues<uint8_t, Stamped<double>> relay_responses_;

private:
  /// @brief Register callbacks for handling incoming CAN messages.
  ///        This defines the logic for handling all MCM CAN messages.
  void initializeCANHandlerCallbacks();

  /// @brief Helper to send a command to an MCM interface endpoint.
  ControlCommandResult send_interface_command(const SygnalEndpoint & endpoint, double value, bool expect_reply);

  /// @brief Helper to send a command to the MCM relay endpoint.
  ControlCommandResult send_relay_command(double value, bool expect_reply);

  /// @brief State of the MCM interfaces and relay based on the last heartbeat or command response.
  std::array<Stamped<bool>, MCM_NUM_INTERFACES> interface_enabled_{};

  /// @brief State of the MCM interface values based on the last command response.
  std::array<Stamped<double>, MCM_NUM_INTERFACES> interface_values_{};

  /// @brief State of the MCM relay based on the last command response.
  Stamped<double> relay_value_{};

  /// @brief Last fault status based on the latest fault message.
  Stamped<McmFaultStatus> faults_{};
};

}  // namespace polymath::sygnal
