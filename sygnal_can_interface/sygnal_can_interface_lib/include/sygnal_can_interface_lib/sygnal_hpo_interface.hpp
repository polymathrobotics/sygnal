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

#ifndef SYGNAL_CAN_INTERFACE_LIB__SYGNAL_HPO_INTERFACE_HPP_
#define SYGNAL_CAN_INTERFACE_LIB__SYGNAL_HPO_INTERFACE_HPP_

#include <array>
#include <chrono>
#include <optional>
#include <string>

#include "socketcan_adapter/can_frame.hpp"

namespace polymath::sygnal
{

// HPO hardware exposes 4 interfaces (0-3). The heartbeat reports a 1-bit control-state per
// interface (Interface{N}State) plus a 2-bit pin-pull value (Interface{N}Value:
// HiZ/Pull_Down/Pull_Up); the pull values are not yet surfaced (TODO: vehicleFeedback).
constexpr uint8_t HPO_NUM_INTERFACES = 4;

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

/// @brief Self-contained HPO board representation.
///
/// Owns the per-interface control-state bits (one per HPO interface, 0-3) and produces /
/// consumes the CAN frames the HPO understands. Frames are routed to the right
/// instance by `bus_address_`: every parse helper rejects frames that do not match
/// this device's bus address. Command frames are populated with `bus_address_`
/// automatically so callers cannot mismatch addresses.
///
/// Unlike the MCM, the HPO does not hold a Sygnal state machine. Each per-interface
/// heartbeat signal is a 1-bit boolean (true = HPO in control, false = released). The
/// MCM's SystemState byte is not tracked here.
class SygnalHpoInterface
{
public:
  SygnalHpoInterface();
  explicit SygnalHpoInterface(uint8_t bus_address);
  ~SygnalHpoInterface() = default;

  /// @brief Try to parse a CAN frame as an HPO heartbeat addressed to this device.
  /// @param frame Raw CAN frame.
  /// @return true if the frame matched and internal state was updated.
  bool parseHeartbeatFrame(const socketcan::CanFrame & frame);

  /// @brief Try to parse a CAN frame as a ControlEnableResponse or ControlCommandResponse
  ///        addressed to this device.
  /// @param frame Raw CAN frame.
  /// @return Populated HpoControlResponse on success, std::nullopt otherwise.
  std::optional<HpoControlResponse> parseControlResponse(const socketcan::CanFrame & frame);

  /// @brief Try to parse a CAN frame as an ErrorStatus addressed to this device.
  /// @param frame Raw CAN frame.
  /// @return Populated HpoErrorStatus on success, std::nullopt otherwise.
  std::optional<HpoErrorStatus> parseErrorFrame(const socketcan::CanFrame & frame);

  /// @brief Build a ControlEnable frame for this device.
  /// @param message_id 8-bit HPO MessageID.
  /// @param enable true to grant HPO control, false to release.
  /// @param[out] error_message Populated on failure.
  /// @return Packed CAN frame on success.
  std::optional<socketcan::CanFrame> createControlEnableFrame(
    uint8_t message_id, bool enable, std::string & error_message);

  /// @brief Build a ControlCommand frame for this device.
  /// @param message_id 8-bit HPO MessageID.
  /// @param value Command value (encoded as float per the DBC).
  /// @param[out] error_message Populated on failure.
  /// @return Packed CAN frame on success.
  std::optional<socketcan::CanFrame> createControlCommandFrame(
    uint8_t message_id, double value, std::string & error_message);

  uint8_t get_bus_address() const
  {
    return bus_address_;
  }

  std::array<bool, HPO_NUM_INTERFACES> get_interface_states() const
  {
    return hpo_interface_states_;
  }

  std::chrono::system_clock::time_point get_last_heartbeat_timestamp() const
  {
    return last_heartbeat_timestamp_;
  }

private:
  uint8_t bus_address_;
  std::array<bool, HPO_NUM_INTERFACES> hpo_interface_states_;
  std::chrono::system_clock::time_point last_heartbeat_timestamp_;
};

}  // namespace polymath::sygnal

#endif  // SYGNAL_CAN_INTERFACE_LIB__SYGNAL_HPO_INTERFACE_HPP_
