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

/// @brief Max number of interfaces an HPO reports in its heartbeat.
constexpr uint8_t HPO_NUM_INTERFACES = 4;

/// @brief One HPO board, where each endpoint is an HPO message ID (0-255).
///        Interface state is only reported for message IDs 0-3.
///        Receives frames with either subsystem bit value.
class SygnalHpoDevice : public SygnalDevice
{
public:
  /// @param socketcan_adapter Adapter that commands go out on.
  /// @param bus_address Bus address of the HPO.
  /// @param command_timeout How long a command waits for its response before its promise is dropped.
  SygnalHpoDevice(
    std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter,
    uint8_t bus_address,
    std::chrono::steady_clock::duration command_timeout);

  /// @brief Interface bits from the latest heartbeat (true = HPO in control); false until the first.
  std::array<bool, HPO_NUM_INTERFACES> get_interface_states() const;

  /// @brief Enable or disable control of a message ID.
  /// @param expect_reply Return a future that resolves from the message's enable response.
  /// @param[out] error_message Populated on failure.
  SendHpoCommandResult send_control_enable(
    uint8_t message_id, bool enable, bool expect_reply, std::string & error_message);

  /// @brief Set a message ID's value.
  /// @param expect_reply Return a future that resolves from the message's command response.
  /// @param[out] error_message Populated on failure.
  SendHpoCommandResult send_control_command(
    uint8_t message_id, double value, bool expect_reply, std::string & error_message);

protected:
  /// @brief Pending replies, keyed by message ID. Push before sending, withdraw_newest if the send fails,
  ///        and fulfill from the reply's handler.
  ResponseQueues<uint8_t, HpoControlResponse> control_enable_responses_;
  ResponseQueues<uint8_t, HpoControlResponse> control_command_responses_;

private:
  /// @brief Register callbacks for handling incoming CAN messages.
  ///        This defines the logic for handling all HPO CAN messages.
  void initializeCANHandlerCallbacks();

  std::array<bool, HPO_NUM_INTERFACES> interface_states_{};
};

}  // namespace polymath::sygnal
