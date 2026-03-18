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
#include <optional>

#include "mvec_lib/mvec_relay.hpp"
#include "socketcan_adapter/socketcan_adapter.hpp"

namespace polymath::sygnal
{

/// @brief MVEC relay controller with async SocketCAN communication.
/// Each request method (query/command/population) abandons any in-flight request of the same type,
/// sends a new CAN frame, and returns a future for the response.
/// If a previous request was still pending, its promise is destroyed and the caller's future
/// throws broken_promise on get(). Callers use wait_for() to handle response timeouts.
class MvecRelaySocketcan
{
public:
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
  /// Abandons any in-flight query (caller's future throws broken_promise).
  /// @return Future containing relay query reply. Use wait_for() to handle timeouts.
  std::future<MvecRelayQueryReply> get_relay_state();

  /// @brief Send relay command and get confirmation asynchronously
  /// Abandons any in-flight command (caller's future throws broken_promise).
  /// @return Future containing command reply. Use wait_for() to handle timeouts.
  std::future<MvecRelayCommandReply> send_relay_command();

  /// @brief Query device population (which relays/fuses are installed)
  /// Abandons any in-flight query (caller's future throws broken_promise).
  /// @return Future containing population reply. Use wait_for() to handle timeouts.
  std::future<MvecPopulationReply> get_relay_population();

  /// @brief Get last received fuse status message
  /// @return Optional containing fuse status if valid data available
  std::optional<MvecFuseStatusMessage> get_last_fuse_status() const;

  /// @brief Get last received relay status message
  /// @return Optional containing relay status if valid data available
  std::optional<MvecRelayStatusMessage> get_last_relay_status() const;

  /// @brief Get last received error status message
  /// @return Optional containing error status if valid data available
  std::optional<MvecErrorStatusMessage> get_last_error_status() const;

private:
  std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter_;
  MvecRelay relay_impl_;

  std::optional<std::promise<MvecRelayQueryReply>> query_reply_promise_;
  std::mutex query_mutex_;

  std::optional<std::promise<MvecRelayCommandReply>> command_reply_promise_;
  std::mutex command_mutex_;

  std::optional<std::promise<MvecPopulationReply>> population_reply_promise_;
  std::mutex population_mutex_;
};

}  // namespace polymath::sygnal
#endif  // MVEC_LIB__MVEC_RELAY_SOCKETCAN_HPP_
