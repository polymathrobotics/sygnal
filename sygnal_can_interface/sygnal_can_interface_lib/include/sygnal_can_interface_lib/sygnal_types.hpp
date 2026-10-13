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

#include <cstdint>
#include <string>
#include <string_view>

namespace polymath::sygnal
{

/// @brief Sygnal system state, as reported in Heartbeat (0x170) and MCM FaultState (0x20).
enum class SygnalSystemState : uint8_t
{
  FAIL_HARD = 254,
  HUMAN_OVERRIDE = 253,
  FAIL_OPERATIONAL_2 = 242,
  FAIL_OPERATIONAL_1 = 241,
  MCM_CONTROL = 1,
  HUMAN_CONTROL = 0
};

/// @brief Get the string of the Sygnal System State
/// @param state input SygnalSystemState
/// @return string of the state value
inline std::string sygnalSystemStateToString(SygnalSystemState state)
{
  switch (state) {
    case SygnalSystemState::FAIL_HARD:
      return "FAIL_HARD";
    case SygnalSystemState::HUMAN_OVERRIDE:
      return "HUMAN_OVERRIDE";
    case SygnalSystemState::FAIL_OPERATIONAL_2:
      return "FAIL_OPERATIONAL_2";
    case SygnalSystemState::FAIL_OPERATIONAL_1:
      return "FAIL_OPERATIONAL_1";
    case SygnalSystemState::MCM_CONTROL:
      return "MCM_CONTROL";
    case SygnalSystemState::HUMAN_CONTROL:
      return "HUMAN_CONTROL";
    default:
      return "UNKNOWN_STATE";
  }
}

/// @brief Sygnal device kinds. Enumerators are named for `magic_enum` string conversion.
enum class SygnalControlDeviceType : uint8_t
{
  CB,
  HPO,
  IO,
  MCM,
  NONE  ///< Not a device type: marks an unmapped bus address, and counts the real types.
};

enum class SygnalControlEndpointType : uint8_t
{
  CB_IF,
  HPO_IF,
  IO_IF,
  MCM_IF,
  MCM_RELAY
};

/// @brief One device subsystem on the bus.
struct SygnalControlDevice
{
  SygnalControlDeviceType type;
  uint8_t bus_id;
  uint8_t subsystem_id = 0;
};

/// @brief One controllable endpoint on a device.
///        endpoint_id is the MCM interface ID or the HPO message ID; MCM relays ignore it.
///        Endpoint values are float32 on the wire; an MCM relay takes and reports 1.0 or 0.0.
struct SygnalEndpoint
{
  SygnalControlEndpointType type;
  SygnalControlDevice device;
  uint8_t endpoint_id;
  int min_value;
  int max_value;
  bool is_int;
  std::string_view name;
};

}  // namespace polymath::sygnal
