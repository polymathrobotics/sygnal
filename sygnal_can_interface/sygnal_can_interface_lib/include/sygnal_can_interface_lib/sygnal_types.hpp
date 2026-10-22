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
#include <future>
#include <optional>
#include <string>
#include <string_view>

namespace polymath::sygnal
{

/// @brief Enum representing the overall state of the system
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

/// @brief Enum representing whether the system is under MCM or human control
enum class SygnalControlState : uint8_t
{
  MCM_CONTROL = 1,
  HUMAN_CONTROL = 0
};

/// @brief Enum representing whether the relay is enabled or disabled
enum class SygnalRelayState : uint8_t
{
  ENABLE = 1,
  DISABLE = 0
};

enum class SygnalControlCommandResponseType : uint8_t
{
  ENABLE = 0,
  CONTROL = 1,
  RELAY = 2
};

struct SygnalControlCommandResponse
{
  SygnalControlCommandResponseType response_type;
  uint8_t interface_id;
  uint8_t bus_id;
  uint8_t subsystem_id;
  // Value can represent different things based on response type
  double value;
};

/// @brief Parsed HPO ControlEnableResponse / ControlCommandResponse.
///        is_enable_response disambiguates which payload is meaningful:
///        when true, the `enable` field carries the response; when false,
///        the `value` field carries the float command response.
struct HpoControlResponse
{
  uint8_t bus_address;
  uint8_t message_id;
  double value;
  bool enable;
  bool is_enable_response;
};

/// @brief Parsed HPO ErrorStatus frame fields.
struct HpoErrorStatus
{
  uint8_t bus_address;
  uint8_t subsystem_id;
  uint8_t error_type;
  uint16_t error_can_id;
  uint8_t config_section_id;
  uint8_t config_interface_id;
};

/// @brief Result of a send command operation
struct SendCommandResult
{
  bool success;
  std::optional<std::future<SygnalControlCommandResponse>> response_future;
};

/// @brief Identifies one MCM endpoint by its CAN bus and subsystem ID.
struct McmId
{
  uint8_t bus_id;
  uint8_t subsystem_id;
};

/// @brief Identifies one HPO endpoint by its CAN bus address.
struct HpoId
{
  uint8_t bus_id;
};

/// @brief Result of an HPO send operation. Mirrors SendCommandResult but is typed for HpoControlResponse
///        because the HPO response carries a `message_id` instead of MCM's interface_id/subsystem_id.
struct SendHpoCommandResult
{
  bool success;
  std::optional<std::future<HpoControlResponse>> response_future;
};

/// @brief Represents a single control interface in Sygnal's System.
///        Interfaces can either take floats(default) or ints as inputs.
struct InterfaceEndpoint
{
  uint8_t bus_id;
  uint8_t subsystem_id;
  uint8_t interface_id;
  int min_value;
  int max_value;
  bool is_int;
  std::string_view name;
};

/// @brief Represents a single relay in Sygnal's System.
struct RelayEndpoint
{
  uint8_t bus_id;
  uint8_t subsystem_id;
  uint8_t relay_id;
  std::string_view name;
};

}  // namespace polymath::sygnal
