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

#include "sygnal_can_interface_lib/crc8.hpp"

#include <cstddef>

namespace polymath::sygnal
{

uint8_t generate_crc8(uint8_t * data)
{
  uint8_t crc = 0x00;
  // Assumes 8-byte data with checksum at data[7]
  for (size_t i = 0; i < 7; i++) {
    crc ^= data[i];
    for (size_t j = 0; j < 8; j++) {
      if ((crc & 0x80) != 0) {
        crc = static_cast<uint8_t>((crc << 1) ^ 0x07);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

bool check_crc8(uint8_t * data)
{
  return data[7] == generate_crc8(data);
}

}  // namespace polymath::sygnal
