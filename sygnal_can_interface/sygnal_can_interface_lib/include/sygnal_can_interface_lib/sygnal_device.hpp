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

#include <cstdint>

#include "socketcan_adapter/can_frame.hpp"
#include "sygnal_can_interface_lib/crc8.hpp"

namespace polymath::sygnal
{

/// @brief One Sygnal board on the bus. Subclassed per board type.
class SygnalDevice
{
public:
  virtual ~SygnalDevice() = default;

  uint8_t get_bus_address() const
  {
    return bus_address_;
  }

protected:
  explicit SygnalDevice(uint8_t bus_address);

  /// @brief true if the frame is 8 bytes and passes its CRC8.
  static bool is_valid_frame(const socketcan::CanFrame & frame);

  /// @brief Pack msg into a frame with the CRC8 of bytes 0-6 written to its crc signal.
  template <typename T>
  static socketcan::CanFrame to_frame_with_crc(T msg)
  {
    msg.crc = 0;
    can_frame packed = msg;
    msg.crc = generate_crc8(packed.data);
    return socketcan::CanFrame(static_cast<can_frame>(msg));
  }

private:
  uint8_t bus_address_;
};

}  // namespace polymath::sygnal
