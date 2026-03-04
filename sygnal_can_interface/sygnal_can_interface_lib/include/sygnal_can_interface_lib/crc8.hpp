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

#ifndef SYGNAL_CAN_INTERFACE_LIB__CRC8_HPP_
#define SYGNAL_CAN_INTERFACE_LIB__CRC8_HPP_

#include <cstdint>

namespace polymath::sygnal
{

/// @brief Generate CRC8 checksum for 8-byte CAN data (checksum at data[7])
/// @param data Data buffer to calculate the checksum for
/// @return CRC8 checksum
uint8_t generate_crc8(uint8_t * data);

/// @brief Verify CRC8 checksum of 8-byte CAN data (checksum at data[7])
/// @param data Data buffer to check
/// @return true if checksum matches, false otherwise
bool check_crc8(uint8_t * data);

}  // namespace polymath::sygnal

#endif  // SYGNAL_CAN_INTERFACE_LIB__CRC8_HPP_
