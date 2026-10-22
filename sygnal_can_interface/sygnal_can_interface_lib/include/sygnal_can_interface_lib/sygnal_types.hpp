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

#include <chrono>
#include <cstdint>
#include <map>
#include <string>
#include <string_view>

#include "magic_enum.hpp"
namespace polymath::sygnal
{

/// @brief Types of Sygnal control devices.
enum class SygnalControlDeviceType : uint8_t
{
  MCM,
  HPO,
  CB,
  IO
};

/// @brief Types of Sygnal control endpoints. Separate from device types since MCM has both interfaces and relays.
enum class SygnalControlEndpointType : uint8_t
{
  MCM_INTERFACE,
  MCM_RELAY,
  HPO_INTERFACE,
  CB_INTERFACE,
  IO_INTERFACE
};

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

/// @brief MCM fault cause, as reported in FaultState, FaultIncrement, FaultList and FaultRootCause.
enum class McmFaultCause : uint8_t
{
  NONE = 0x00,
  EMERGENCY_STOP = 0x01,
  EXTERNAL_FAULT = 0x02,
  CAN_FAIL_OPERATIONAL_1 = 0x03,
  CAN_FAIL_OPERATIONAL_2 = 0x04,
  CAN_FAIL_HARD = 0x05,
  INVALID_SUBSYSTEM_ID = 0x10,
  INVALID_CONFIGURATION_CRC = 0x11,
  PHYSICAL_INTERFACE_TRANSITION = 0x1B,
  INVALID_INPUT_SIGNAL = 0x20,
  INVALID_OUTPUT_SIGNAL = 0x21,
  HEARTBEAT_TIMEOUT = 0x30,
  HEARTBEAT_COUNT = 0x31,
  CONTROL_TIMEOUT = 0x32,
  CONTROL_COUNT = 0x33,
  CONTROL_OUT_OF_BOUNDS = 0x34,
  CLOSED_LOOP_TIMEOUT = 0x35,
  CLOSED_LOOP_COUNT = 0x36,
  CLOSED_LOOP_OUT_OF_BOUNDS = 0x37,
  INVALID_CONFIGURATION_OPTION = 0x38,
  SW_INDEX_OUT_OF_BOUNDS = 0x40,
  SW_INVALID_STATE = 0x41,
  SW_COPROCESSOR_OVERFLOW = 0x42,
};

/// @brief Error type, as reported in ErrorStatus (0x30).
/// @details OVERRIDE is in the Sygnal docs, not the DBC.
enum class SygnalErrorType : uint8_t
{
  NO_ERROR = 0,
  CRC = 1,
  SECTION_ID = 2,
  INTERFACE_ID = 3,
  PARAMETER_ID = 4,
  PERMISSIONS = 5,
  STATE = 6,
  FEEDBACK_TYPE = 7,
  COUNT = 8,
  KEY = 9,
  PASSWORD = 10,
  COMMAND = 11,
  OVERRIDE = 12,
  OTHER = 255,
};

/// @brief One ErrorStatus report: the device could not process a frame it received.
/// @details type: Kind of error.
///          can_id: CAN ID of the frame the device could not process.
///          config_section_id: Configuration section of a configuration error; IDs differ per device type.
///          config_interface_id: Interface of a configuration error.
struct SygnalError
{
  SygnalErrorType type{SygnalErrorType::NO_ERROR};
  uint16_t can_id{0};
  uint8_t config_section_id{0};
  uint8_t config_interface_id{0};
};

/// @brief One device on the bus. subsystem_id applies to only MCMs; Other devices ignore it.
///        name label is used in errors and diagnostics only;
struct SygnalControlDevice
{
  SygnalControlDeviceType type;
  uint8_t bus_id;
  uint8_t subsystem_id = 0;
  std::string_view name{};
};

/// @brief One controllable endpoint on a device.
///        endpoint_id is generic for InterfaceID on MCMs and IOs, or for MessageID on HPOs and CBs.
///        Values are float32 on the wire; an MCM relay takes and reports 1.0 or 0.0.
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

/// @brief Fault status of one MCM subsystem since its faults were last cleared.
struct McmFaultStatus
{
  SygnalSystemState system_state{SygnalSystemState::FAIL_HARD};
  /// Cause of the current system_state; NONE when not faulted.
  McmFaultCause state_cause{McmFaultCause::NONE};
  /// Cause that tripped each fault level; NONE if that level was never entered.
  McmFaultCause fail_op1_cause{McmFaultCause::NONE};
  McmFaultCause fail_op2_cause{McmFaultCause::NONE};
  McmFaultCause fail_hard_cause{McmFaultCause::NONE};
  /// Occurrences of each cause.
  std::map<McmFaultCause, uint16_t> counts;
};

/// @brief Struct to help bundle a feedback value with its associated timestamp.
template <typename T>
struct Stamped
{
  T value;
  std::chrono::system_clock::time_point timestamp;
};

/// @brief Helper to generate a human-readable description of a Sygnal control device.
inline std::string describe(const SygnalControlDevice & device)
{
  std::string text = std::string(magic_enum::enum_name(device.type)) + " bus_id " + std::to_string(device.bus_id);
  if (SygnalControlDeviceType::MCM == device.type) {
    text += " subsystem_id " + std::to_string(device.subsystem_id);
  }
  return device.name.empty() ? text : std::string(device.name) + " (" + text + ")";
}

}  // namespace polymath::sygnal

/// @brief Change magic_enum to use the range 0..255 for SygnalSystemState (which goes up to 255),
///        instead of the default -128..128.
template <>
struct magic_enum::customize::enum_range<polymath::sygnal::SygnalSystemState>
{
  static constexpr int min = 0;
  static constexpr int max = 255;
};

/// @brief Same for SygnalErrorType, whose OTHER is 255.
template <>
struct magic_enum::customize::enum_range<polymath::sygnal::SygnalErrorType>
{
  static constexpr int min = 0;
  static constexpr int max = 255;
};
