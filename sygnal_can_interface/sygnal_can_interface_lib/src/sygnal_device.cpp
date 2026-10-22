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

namespace polymath::sygnal
{

SygnalDevice::SygnalDevice(uint8_t bus_address)
: bus_address_(bus_address)
{}

bool SygnalDevice::is_valid_frame(const socketcan::CanFrame & frame)
{
  const can_frame raw = frame.get_frame();
  return CAN_MAX_DLC == raw.len && check_crc8(raw.data);
}

}  // namespace polymath::sygnal
