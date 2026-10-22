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

#include <chrono>
#include <cstdint>
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

/// @brief Result of sending a control enable command to a Sygnal endpoint.
/// @details success: true if the command was sent successfully, false otherwise.
///          error_message: Empty on success, reason for failure otherwise.
///          response_future:
///              If expect_reply was true, contains a future that resolves with the endpoint's enable response.
///              If expect_reply was false, response_future will be std::nullopt.
struct ControlEnableCommandResult
{
  bool success;
  std::string error_message;
  std::optional<std::future<Stamped<SygnalSystemState>>> response_future;  ///< Stamped with the response's bus time.
};

/// @brief Result of sending a control command to a Sygnal endpoint.
/// @details success: true if the command was sent successfully, false otherwise.
///          error_message: Empty on success, reason for failure otherwise.
///          response_future:
///              If expect_reply was true, contains a future that resolves with the endpoint's command response.
///              If expect_reply was false, response_future will be std::nullopt.
struct ControlCommandResult
{
  bool success;
  std::string error_message;  ///< Empty on success.
  std::optional<std::future<Stamped<double>>> response_future;  ///< Stamped with the response's bus time.
};

/// @brief A class representing a Sygnal Device.
///        Is meant to be subclassed for specific Sygnal device types.
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

  /// @brief The descriptor this device was configured with.
  const SygnalControlDevice & device() const;

  /// @brief Get SystemState from this device's latest stashed heartbeat.
  ///        Returns FAIL_HARD until the first one.
  Stamped<SygnalSystemState> get_device_system_state() const;

  /// @brief Control state of an endpoint from the latest heartbeat or enable response.
  /// @throws std::invalid_argument if this device has no such endpoint.
  virtual Stamped<SygnalSystemState> get_interface_control_state(const SygnalEndpoint & endpoint) const;

  /// @brief Value from the endpoint's latest command response; 0 until the first.
  /// @throws std::invalid_argument if this device has no such endpoint.
  virtual Stamped<double> get_control_command_value(const SygnalEndpoint & endpoint) const;

  /// @brief Enable or disable control of an endpoint.
  /// @param expect_reply Return a future that resolves from the endpoint's enable response.
  /// @return success false with the reason if the send fails.
  /// @throws std::invalid_argument if this device does not support the endpoint.
  virtual ControlEnableCommandResult send_control_enable_command(
    const SygnalEndpoint & endpoint, bool enable, bool expect_reply);

  /// @brief Set an endpoint's value.
  /// @param expect_reply Return a future that resolves from the endpoint's command response.
  /// @return success false with the reason if the send fails.
  /// @throws std::invalid_argument if this device does not support the endpoint.
  virtual ControlCommandResult send_control_command(const SygnalEndpoint & endpoint, double value, bool expect_reply);

protected:
  /// @param socketcan_adapter Adapter that commands go out on.
  /// @param device Descriptor of this device.
  /// @param command_timeout How long a command waits for its response before its promise is dropped.
  SygnalDevice(
    std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter,
    SygnalControlDevice device,
    std::chrono::steady_clock::duration command_timeout);

  /// @brief Pack msg into a frame. A message with a crc signal gets the CRC8 of bytes 0-6 written to it;
  ///        callers leave crc unset.
  template <typename T>
  static socketcan::CanFrame to_frame_with_crc(T msg)
  {
    if constexpr (requires { msg.crc; }) {
      msg.crc = 0;
      can_frame packed = msg;
      msg.crc = generate_crc8(packed.data);
    }
    return socketcan::CanFrame(static_cast<can_frame>(msg));
  }

  const std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter_;

  /// @brief handlers that handle() dispatches to. Register them in subclass constructors only.
  dbc_gen_cpp::CANHandler can_handler_;

  /// @brief Guards system_state_ and subclass state.
  mutable std::mutex state_mutex_;

  /// @brief Written by subclass message handlers on the reception thread.
  Stamped<SygnalSystemState> system_state_{SygnalSystemState::FAIL_HARD, {}};

  /// @brief How long any command waits for its response. Declared before the queues that are built from it.
  const std::chrono::steady_clock::duration command_timeout_;

  /// @brief Pending replies, keyed by the endpoint ID in the response frame.
  ///        Push before sending a command that expects a reply, withdraw_newest if the send fails,
  ///        and fulfill from the reply's handler.
  ResponseQueues<uint8_t, Stamped<SygnalSystemState>> control_enable_responses_;
  ResponseQueues<uint8_t, Stamped<double>> control_command_responses_;

private:
  const SygnalControlDevice device_;
};

}  // namespace polymath::sygnal
