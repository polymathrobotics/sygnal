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

#include <array>
#include <chrono>
#include <optional>
#include <string>

#include "socketcan_adapter/can_frame.hpp"
#include "sygnal_can_interface_lib/sygnal_device.hpp"
#include "sygnal_can_interface_lib/sygnal_types.hpp"

namespace polymath::sygnal
{

constexpr uint8_t DEFAULT_MCM_BUS_ADDRESS = 1;
constexpr uint8_t SECONDARY_MCM_BUS_ADDRESS = 2;

constexpr uint32_t MAX_SYGNAL_INTERFACES = 6;
constexpr uint32_t MAX_SYGNAL_SUBSYSTEMS = 2;

/// @brief One MCM subsystem: heartbeat state, plus the MCM control and relay frames.
class SygnalMcmDevice : public SygnalDevice
{
public:
  SygnalMcmDevice();
  explicit SygnalMcmDevice(const uint8_t bus_address, const uint8_t subsystem_id);

  std::array<SygnalSystemState, 5> get_interface_states() const
  {
    return sygnal_interface_states_;
  }

  uint8_t get_subsystem_id() const
  {
    return subsystem_id_;
  }

  SygnalSystemState get_mcm_state() const
  {
    return sygnal_mcm_state_;
  }

  std::chrono::system_clock::time_point get_last_heartbeat_timestamp() const
  {
    return last_heartbeat_timestamp_;
  }

  bool parseMcmHeartbeatFrame(const socketcan::CanFrame & frame);

  /// @brief Set the control state (enable/disable MCM control)
  /// @param interface_id Sygnal Interface to enable/disable
  /// @param control_state Sygnal Control State (Human or MCM control)
  /// @param[out] error_message Error message in case of failure
  /// @return can frame if succesful
  static std::optional<socketcan::CanFrame> createControlStateCommandFrame(
    const uint8_t bus_id,
    const uint8_t interface_id,
    const uint8_t subsystem_id,
    const SygnalControlState control_state,
    std::string & error_message);

  /// @brief Generate a control command frame to send to the vehicle
  /// @param interface_id Sygnal Interface ID to send command to
  /// @param value Value to set interface control command to
  /// @param error_message Error in case of failure
  /// @return can frame if succesful
  static std::optional<socketcan::CanFrame> createControlCommandFrame(
    const uint8_t bus_id,
    const uint8_t interface_id,
    const uint8_t subsystem_id,
    const double value,
    std::string & error_message);

  /// @brief Set the relay state for engine control or gears
  /// @param subsystem_id Sygnal Subsystem Relay to enable/disable
  /// @param relay_state Sygnal Relay state (Enable or Disable)
  /// @param[out] error_message Error message in case of failure
  /// @return can frame if succesful
  static std::optional<socketcan::CanFrame> createRelayCommandFrame(
    const uint8_t bus_id, const uint8_t subsystem_id, const bool relay_state, std::string & error_message);

  /// @brief Parse a command response frame if it is covered by this interface
  /// @param frame Raw cran frame to check and parse
  /// @return Optional, if the frame is a valid response, return the response
  static std::optional<SygnalControlCommandResponse> parseCommandResponseFrame(const socketcan::CanFrame & frame);

private:
  uint8_t subsystem_id_;
  SygnalSystemState sygnal_mcm_state_;
  std::array<SygnalSystemState, 5> sygnal_interface_states_;
  std::chrono::system_clock::time_point last_heartbeat_timestamp_;
};

}  // namespace polymath::sygnal
