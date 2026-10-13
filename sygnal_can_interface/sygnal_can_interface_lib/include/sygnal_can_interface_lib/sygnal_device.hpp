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

#include <linux/can.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include "dbc_gen_cpp/can_handler.hpp"
#include "socketcan_adapter/can_frame.hpp"
#include "socketcan_adapter/socketcan_adapter.hpp"
#include "sygnal_can_interface_lib/crc8.hpp"
#include "sygnal_can_interface_lib/response_queues.hpp"
#include "sygnal_can_interface_lib/sygnal_types.hpp"

namespace polymath::sygnal
{

struct EndpointStateCommandResult
{
  bool success;
  std::string error_message;  ///< Empty on success.
  std::optional<std::future<SygnalSystemState>> response_future;
};

struct EndpointValueCommandResult
{
  bool success;
  std::string error_message;  ///< Empty on success.
  std::optional<std::future<double>> response_future;
};

/// @brief One Sygnal device subsystem: parses frames addressed to it and sends its commands.
///        All public methods are thread-safe.
class SygnalDevice
{
public:
  virtual ~SygnalDevice() = default;
  SygnalDevice(const SygnalDevice &) = delete;
  SygnalDevice & operator=(const SygnalDevice &) = delete;

  /// @brief Dispatch a frame addressed to this device to its registered message handler.
  /// @return true if the frame is 8 bytes, passes its CRC, and a handler consumed it.
  bool handle(
    const can_frame & frame,
    std::chrono::steady_clock::time_point receive_time = {},
    std::chrono::system_clock::time_point bus_time = {});

  /// @brief Get SystemState from this device's latest stashed heartbeat.
  ///        Returns FAIL_HARD until the first one.
  SygnalSystemState get_system_state() const;

  /// @brief Get whether the endpoint is in MCM_CONTROL or HUMAN_CONTROL based on this device's latest stashed heartbeat.
  ///        Returns HUMAN_CONTROL until the first stashed heartbeat is received.
  /// @throws std::invalid_argument if this device has no such endpoint.
  virtual SygnalSystemState get_endpoint_state(const SygnalEndpoint & endpoint) const = 0;

  /// @brief Value from the endpoint's latest command response;
  ///        Returns 0 or false until the first command response is received.
  /// @throws std::invalid_argument if this device has no such endpoint.
  virtual double get_endpoint_value(const SygnalEndpoint & endpoint) const = 0;

  /// @brief Enable or disable control of an endpoint.
  /// @param expect_reply Return a future that resolves from the endpoint's enable response.
  virtual EndpointStateCommandResult send_endpoint_state_command(
    const SygnalEndpoint & endpoint, bool enable, bool expect_reply) = 0;

  /// @brief Set an endpoint's value.
  /// @param expect_reply Return a future that resolves from the endpoint's command response.
  virtual EndpointValueCommandResult send_endpoint_value_command(
    const SygnalEndpoint & endpoint, double value, bool expect_reply) = 0;

  /// @brief Future for the endpoint's next enable response. Call before sending the command.
  std::future<SygnalSystemState> expect_endpoint_state_response(const SygnalEndpoint & endpoint);

  /// @brief Future for the endpoint's next value response. Call before sending the command.
  std::future<double> expect_endpoint_value_response(const SygnalEndpoint & endpoint);

  /// @brief Withdraw the newest promise queued by expect_endpoint_state_response, for a command that failed to send.
  ///        Assumes one sender at a time per endpoint.
  void cancel_endpoint_state_response(const SygnalEndpoint & endpoint);

  /// @brief Withdraw the newest promise queued by expect_endpoint_value_response, for a command that failed to send.
  ///        Assumes one sender at a time per endpoint.
  void cancel_endpoint_value_response(const SygnalEndpoint & endpoint);

protected:
  /// @param socketcan_adapter Adapter that commands go out on.
  explicit SygnalDevice(std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter);

  /// @brief Pack msg into a frame, writing its CRC over the msg.crc placeholder.
  template <typename T>
  static socketcan::CanFrame to_frame(const T & msg)
  {
    const can_frame packed = msg;
    std::array<unsigned char, CAN_MAX_DLC> data;
    std::copy(packed.data, packed.data + CAN_MAX_DLC, data.begin());
    data[CAN_MAX_DLC - 1] = generate_crc8(data.data());

    socketcan::CanFrame frame;
    frame.set_can_id(T::Id);
    frame.set_len(T::DataLength);
    frame.set_data(data);
    return frame;
  }

  /// @brief Register the callback for message type T. Call from subclass constructors only.
  template <typename T>
  void set_handler(std::function<void(const T &)> callback)
  {
    handler_.set_handler<T>(std::move(callback));
  }

  void fulfill_endpoint_state_response(SygnalControlEndpointType type, uint8_t endpoint_id, SygnalSystemState state);
  void fulfill_endpoint_value_response(SygnalControlEndpointType type, uint8_t endpoint_id, double value);

  const std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter_;

  /// Guards system_state_, the response queues and subclass state.
  /// Subclass handlers must not hold it while calling fulfill_* or get_system_state().
  mutable std::mutex mutex_;

  /// Written by subclass message handlers on the reception thread.
  SygnalSystemState system_state_{SygnalSystemState::FAIL_HARD};

private:
  using ResponseKey = std::pair<SygnalControlEndpointType, uint8_t>;

  dbc_gen_cpp::CANHandler handler_;
  ResponseQueues<ResponseKey, SygnalSystemState> state_responses_;
  ResponseQueues<ResponseKey, double> value_responses_;
};

}  // namespace polymath::sygnal
