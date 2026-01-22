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

#ifndef SYGNAL_CAN_INTERFACE_LIB__SYGNAL_COMMAND_INTERFACE_HPP_
#define SYGNAL_CAN_INTERFACE_LIB__SYGNAL_COMMAND_INTERFACE_HPP_

#include <optional>
#include <string>

#include "socketcan_adapter/can_frame.hpp"
#include "socketcan_adapter/socketcan_adapter.hpp"

namespace polymath::sygnal
{

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
  // Value can represent different things based on response type
  double value;
};

class SygnalControlInterface
{
public:
  SygnalControlInterface();
  ~SygnalControlInterface() = default;

  /// @brief Set the control state (enable/disable MCM control)
  /// @param interface_id Sygnal Interface to enable/disable
  /// @param control_state Sygnal Control State (Human or MCM control)
  /// @param[out] error_message Error message in case of failure
  /// @return can frame if succesful
  std::optional<polymath::socketcan::CanFrame> createControlStateCommandFrame(
    const uint8_t bus_id,
    const uint8_t interface_id,
    const SygnalControlState control_state,
    std::string & error_message);

  /// @brief Generate a control command frame to send to the vehicle
  /// @param interface_id Sygnal Interface ID to send command to
  /// @param value Value to set interface control command to
  /// @param error_message Error in case of failure
  /// @return can frame if succesful
  std::optional<polymath::socketcan::CanFrame> createControlCommandFrame(
    const uint8_t bus_id, const uint8_t interface_id, const double value, std::string & error_message);

  /// @brief Set the relay state for engine control or gears
  /// @param subsystem_id Sygnal Subsystem Relay to enable/disable
  /// @param relay_state Sygnal Relay state (Enable or Disable)
  /// @param[out] error_message Error message in case of failure
  /// @return can frame if succesful
  std::optional<polymath::socketcan::CanFrame> createRelayCommandFrame(
    const uint8_t bus_id, const uint8_t subsystem_id, const bool relay_state, std::string & error_message);

  /// @brief Parse a command response frame if it is covered by this interface
  /// @param frame Raw cran frame to check and parse
  /// @return Optional, if the frame is a valid response, return the response
  std::optional<SygnalControlCommandResponse> parseCommandResponseFrame(const socketcan::CanFrame & frame);
};

}  // namespace polymath::sygnal

#endif  // SYGNAL_CAN_INTERFACE_LIB__SYGNAL_COMMAND_INTERFACE_HPP_
