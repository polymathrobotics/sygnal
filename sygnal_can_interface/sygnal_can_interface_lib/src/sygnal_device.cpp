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

#include "sygnal_can_interface_lib/sygnal_device.hpp"

#include <memory>
#include <utility>

namespace polymath::sygnal
{

SygnalDevice::SygnalDevice(
  std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter,
  uint8_t bus_address,
  std::chrono::steady_clock::duration command_timeout)
: socketcan_adapter_(std::move(socketcan_adapter))
, command_timeout_(command_timeout)
, bus_address_(bus_address)
{}

bool SygnalDevice::handle(const can_frame & frame)
{
  // Reject frames that are not 8 bytes since every sygnal message puts the CRC in the 8th byte.
  // Then check the CRC of the frame.
  if (CAN_MAX_DLC != frame.len || !check_crc8(frame.data)) {
    return false;
  }

  // Pass the frame to can handler.
  return can_handler_.handle(frame);
}

uint8_t SygnalDevice::get_bus_address() const
{
  return bus_address_;
}

}  // namespace polymath::sygnal
