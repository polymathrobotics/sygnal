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

#ifndef MVEC_LIB__MVEC_RELAY_SOCKETCAN_HPP_
#define MVEC_LIB__MVEC_RELAY_SOCKETCAN_HPP_

#include <future>
#include <memory>
#include <mutex>
#include <queue>

#include "mvec_lib/mvec_relay.hpp"
#include "socketcan_adapter/socketcan_adapter.hpp"

namespace polymath::sygnal
{

/// @brief MVEC relay controller with async SocketCAN communication
/// Provides high-level interface for MVEC relay control with thread-safe promise/future pattern
class MvecRelaySocketcan
{
public:
  /// @brief Constructor
  /// @param socketcan_adapter Shared pointer to socketcan adapter for CAN communication
  explicit MvecRelaySocketcan(std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter);

  /// @brief Parse incoming CAN frame and fulfill waiting promises
  /// @param frame CAN frame to parse
  /// @return Message type that was parsed
  MvecMessageType parse(const socketcan::CanFrame & frame);

  /// @brief Set relay command state
  /// @param relay_id Relay ID (0-11)
  /// @param relay_state Relay state (0=off, 1=on)
  void set_relay_in_command(uint8_t relay_id, uint8_t relay_state);

  /// @brief Clear all relay commands
  void clear_relay();

  /// @brief Query current relay states asynchronously
  /// @return Future that will contain relay query reply
  std::future<MvecRelayQueryReply> get_relay_state();

  /// @brief Send relay command and wait for confirmation
  /// @return Future that will contain command reply
  std::future<MvecRelayCommandReply> send_relay_command();

  /// @brief Query device population (which relays/fuses are installed)
  /// @return Future that will contain population reply
  std::future<MvecPopulationReply> get_relay_population();

  /// @brief Get last received fuse status message
  /// @return Optional containing fuse status if valid data available
  const std::optional<MvecFuseStatusMessage> get_last_fuse_status();

  /// @brief Get last received relay status message
  /// @return Optional containing relay status if valid data available
  const std::optional<MvecRelayStatusMessage> get_last_relay_status();

  /// @brief Get last received error status message
  /// @return Optional containing error status if valid data available
  const std::optional<MvecErrorStatusMessage> get_last_error_status();

private:
  /// @brief SocketCAN adapter for CAN communication
  std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter_;
  /// @brief Core MVEC relay implementation
  MvecRelay relay_impl_;

  /// @brief Queue of promises waiting for relay query responses
  std::queue<std::promise<MvecRelayQueryReply>> query_reply_promises_;
  /// @brief Queue of promises waiting for relay command responses
  std::queue<std::promise<MvecRelayCommandReply>> command_reply_promises_;
  /// @brief Queue of promises waiting for population query responses
  std::queue<std::promise<MvecPopulationReply>> population_reply_promises_;

  /// @brief Mutex protecting promise queues for thread safety
  std::mutex promises_mutex_;
};

}  // namespace polymath::sygnal
#endif  // MVEC_LIB__MVEC_RELAY_SOCKETCAN_HPP_
