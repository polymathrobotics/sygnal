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
#include <cstdint>
#include <memory>
#include <string>

#include "sygnal_can_interface_lib/response_queues.hpp"
#include "sygnal_can_interface_lib/sygnal_device.hpp"
#include "sygnal_can_interface_lib/sygnal_types.hpp"

namespace polymath::sygnal
{

constexpr uint8_t DEFAULT_MCM_BUS_ADDRESS = 1;
constexpr uint8_t SECONDARY_MCM_BUS_ADDRESS = 2;

constexpr uint32_t MAX_SYGNAL_INTERFACES = 6;
constexpr uint32_t MAX_SYGNAL_SUBSYSTEMS = 2;

/// @brief One MCM subsystem: interface endpoints (interface ID 0-5) and one relay.
class SygnalMcmDevice : public SygnalDevice
{
public:
  /// @param socketcan_adapter Adapter that commands go out on.
  /// @param bus_address Bus address of the MCM.
  /// @param subsystem_id Subsystem of the MCM, 0 or 1.
  /// @param command_timeout How long a command waits for its response before its promise is dropped.
  SygnalMcmDevice(
    std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter,
    uint8_t bus_address,
    uint8_t subsystem_id,
    std::chrono::steady_clock::duration command_timeout);

  uint8_t get_subsystem_id() const;

  /// @brief Interface states from the latest heartbeat; FAIL_HARD until the first.
  std::array<SygnalSystemState, 5> get_interface_states() const;

  /// @brief System state from the latest heartbeat; FAIL_HARD until the first.
  SygnalSystemState get_mcm_state() const;

  /// @brief Enable or disable control of an interface.
  /// @param expect_reply Return a future that resolves from the interface's enable response.
  /// @param[out] error_message Populated on failure.
  SendCommandResult send_control_state_command(
    uint8_t interface_id, SygnalControlState control_state, bool expect_reply, std::string & error_message);

  /// @brief Set an interface's value.
  /// @param expect_reply Return a future that resolves from the interface's command response.
  /// @param[out] error_message Populated on failure.
  SendCommandResult send_control_command(
    uint8_t interface_id, double value, bool expect_reply, std::string & error_message);

  /// @brief Close the relay when relay_state is true, open it otherwise.
  /// @param expect_reply Return a future that resolves from the relay command response.
  /// @param[out] error_message Populated on failure.
  SendCommandResult send_relay_command(bool relay_state, bool expect_reply, std::string & error_message);

protected:
  /// @brief Pending replies, keyed by interface ID. Push before sending, withdraw_newest if the send fails,
  ///        and fulfill from the reply's handler.
  ResponseQueues<uint8_t, SygnalControlCommandResponse> control_enable_responses_;
  ResponseQueues<uint8_t, SygnalControlCommandResponse> control_command_responses_;

  /// @brief Pending relay replies, always on key 0; RelayCommandResponse carries no ID.
  ResponseQueues<uint8_t, SygnalControlCommandResponse> relay_responses_;

private:
  /// @brief Register callbacks for handling incoming CAN messages.
  ///        This defines the logic for handling all MCM CAN messages.
  void initializeCANHandlerCallbacks();

  const uint8_t subsystem_id_;
  SygnalSystemState mcm_state_{SygnalSystemState::FAIL_HARD};
  std::array<SygnalSystemState, 5> interface_states_;
};

}  // namespace polymath::sygnal
