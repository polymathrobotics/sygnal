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
#include <memory>
#include <mutex>

#include "dbc_gen_cpp/can_handler.hpp"
#include "socketcan_adapter/can_frame.hpp"
#include "socketcan_adapter/socketcan_adapter.hpp"
#include "sygnal_can_interface_lib/crc8.hpp"

namespace polymath::sygnal
{

/// @brief One Sygnal board on the bus. Subclassed per board type.
class SygnalDevice
{
public:
  virtual ~SygnalDevice() = default;
  SygnalDevice(const SygnalDevice &) = delete;
  SygnalDevice & operator=(const SygnalDevice &) = delete;

  /// @brief Dispatch a frame addressed to this device to its registered message handler.
  /// @return true if the frame is 8 bytes, passes its CRC, and a handler consumed it.
  bool handle(const can_frame & frame);

  uint8_t get_bus_address() const;

protected:
  /// @param socketcan_adapter Adapter that commands go out on.
  /// @param bus_address Bus address of this device.
  /// @param command_timeout How long a command waits for its response before its promise is dropped.
  SygnalDevice(
    std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter,
    uint8_t bus_address,
    std::chrono::steady_clock::duration command_timeout);

  /// @brief Pack msg into a frame with the CRC8 of bytes 0-6 written to its crc signal.
  template <typename T>
  static socketcan::CanFrame to_frame_with_crc(T msg)
  {
    msg.crc = 0;
    can_frame packed = msg;
    msg.crc = generate_crc8(packed.data);
    return socketcan::CanFrame(static_cast<can_frame>(msg));
  }

  const std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter_;

  /// @brief Handlers that handle() dispatches to. Register them in subclass constructors only.
  dbc_gen_cpp::CANHandler can_handler_;

  /// @brief Guards subclass state.
  mutable std::mutex state_mutex_;

  /// @brief How long any command waits for its response. Declared before the queues that are built from it.
  const std::chrono::steady_clock::duration command_timeout_;

private:
  const uint8_t bus_address_;
};

}  // namespace polymath::sygnal
