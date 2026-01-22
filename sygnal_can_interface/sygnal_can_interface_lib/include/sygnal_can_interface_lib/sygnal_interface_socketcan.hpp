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

#ifndef SYGNAL_CAN_INTERFACE_LIB__SYGNAL_INTERFACE_SOCKETCAN_HPP_
#define SYGNAL_CAN_INTERFACE_LIB__SYGNAL_INTERFACE_SOCKETCAN_HPP_

#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>

#include "socketcan_adapter/socketcan_adapter.hpp"
#include "sygnal_can_interface_lib/sygnal_command_interface.hpp"
#include "sygnal_can_interface_lib/sygnal_mcm_interface.hpp"

namespace polymath::sygnal
{

/// @brief Result of a send command operation
struct SendCommandResult
{
  bool success;
  std::optional<std::future<SygnalControlCommandResponse>> response_future;
};

/// @brief Combined Sygnal MCM and Control interface with SocketCAN communication
/// Provides thread-safe async communication with promise/future pattern for responses
class SygnalInterfaceSocketcan
{
public:
  /// @brief Constructor
  /// @param socketcan_adapter Shared pointer to socketcan adapter for CAN communication
  explicit SygnalInterfaceSocketcan(std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter);

  /// @brief Parse incoming CAN frame for MCM heartbeat and command responses
  /// @param frame CAN frame to parse
  void parse(const socketcan::CanFrame & frame);

  /// @brief Get interface states array
  std::array<SygnalSystemState, 5> get_interface_states() const;

  /// @brief Get MCM 0 system state
  SygnalSystemState get_sygnal_mcm0_state() const;

  /// @brief Get MCM 1 system state
  SygnalSystemState get_sygnal_mcm1_state() const;

  /// @brief Send control state (enable/disable) command
  /// @param bus_id Bus address
  /// @param interface_id Interface to enable/disable
  /// @param control_state MCM_CONTROL or HUMAN_CONTROL
  /// @param expect_reply If true, returns future for response; if false, fire-and-forget
  /// @param error_message Populated on failure
  /// @return Result with success flag and optional response future
  SendCommandResult sendControlStateCommand(
    uint8_t bus_id,
    uint8_t interface_id,
    SygnalControlState control_state,
    bool expect_reply,
    std::string & error_message);

  /// @brief Send control command with value
  /// @param bus_id Bus address
  /// @param interface_id Interface to control
  /// @param value Control value
  /// @param expect_reply If true, returns future for response; if false, fire-and-forget
  /// @param error_message Populated on failure
  /// @return Result with success flag and optional response future
  SendCommandResult sendControlCommand(
    uint8_t bus_id, uint8_t interface_id, double value, bool expect_reply, std::string & error_message);

  /// @brief Send relay command
  /// @param bus_id Bus address
  /// @param subsystem_id Subsystem/relay to control
  /// @param relay_state Enable or disable
  /// @param expect_reply If true, returns future for response; if false, fire-and-forget
  /// @param error_message Populated on failure
  /// @return Result with success flag and optional response future
  SendCommandResult sendRelayCommand(
    uint8_t bus_id, uint8_t subsystem_id, bool relay_state, bool expect_reply, std::string & error_message);

private:
  std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter_;
  SygnalMcmInterface mcm_interface_;
  SygnalControlInterface control_interface_;

  // Promise queues for each response type
  std::queue<std::promise<SygnalControlCommandResponse>> enable_response_promises_;
  std::queue<std::promise<SygnalControlCommandResponse>> control_response_promises_;
  std::queue<std::promise<SygnalControlCommandResponse>> relay_response_promises_;

  std::mutex promises_mutex_;
};

}  // namespace polymath::sygnal

#endif  // SYGNAL_CAN_INTERFACE_LIB__SYGNAL_INTERFACE_SOCKETCAN_HPP_
