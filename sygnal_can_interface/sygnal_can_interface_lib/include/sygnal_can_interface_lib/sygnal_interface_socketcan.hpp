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

#ifndef SYGNAL_CAN_INTERFACE_LIB__SYGNAL_INTERFACE_SOCKETCAN_HPP_
#define SYGNAL_CAN_INTERFACE_LIB__SYGNAL_INTERFACE_SOCKETCAN_HPP_

#include <array>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "socketcan_adapter/socketcan_adapter.hpp"
#include "sygnal_can_interface_lib/sygnal_device.hpp"
#include "sygnal_can_interface_lib/sygnal_hpo_device.hpp"
#include "sygnal_can_interface_lib/sygnal_mcm_device.hpp"
#include "sygnal_can_interface_lib/sygnal_types.hpp"

namespace polymath::sygnal
{

constexpr std::chrono::seconds DEFAULT_COMMAND_TIMEOUT{1};

/// @brief Combined Sygnal MCM and Control interface with SocketCAN communication
/// Provides thread-safe async communication with promise/future pattern for responses
class SygnalInterfaceSocketcan
{
public:
  /// @brief Constructor
  /// @param socketcan_adapter Shared pointer to socketcan adapter for CAN communication
  /// @param mcm_ids Flat list of MCM endpoints to manage, each identified by bus and subsystem ID
  /// @param hpo_ids Flat list of HPO endpoints to manage, each identified by bus ID. Empty by
  ///                default; an empty list disables all HPO behavior.
  /// @param command_timeout How long a command waits for its response before its promise is dropped.
  ///        Callers waiting on a response future should time out no sooner than this.
  /// @throws std::invalid_argument on a bus_id above 127, a subsystem_id above 1, or two devices at one address.
  ///         An HPO takes both subsystem addresses of its bus.
  SygnalInterfaceSocketcan(
    std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter,
    const std::vector<McmId> & mcm_ids,
    const std::vector<HpoId> & hpo_ids = {},
    std::chrono::steady_clock::duration command_timeout = DEFAULT_COMMAND_TIMEOUT);

  /// @brief Route a received frame to the device it addresses.
  /// @return true if a configured device consumed the frame.
  bool parse(const socketcan::CanFrame & frame);

  /// @brief Get interface states array from MCM 0
  std::optional<std::array<SygnalSystemState, 5>> get_interface_state(
    const uint8_t bus_address, const uint8_t subsystem_id) const;

  /// @brief Get MCM 0 system state
  std::optional<SygnalSystemState> get_sygnal_mcm_state(const uint8_t bus_address, const uint8_t subsystem_id) const;

  /// @brief Send control state (enable/disable) command
  /// @param bus_id Bus address
  /// @param interface_id Interface to enable/disable
  /// @param control_state MCM_CONTROL or HUMAN_CONTROL
  /// @param expect_reply If true, returns future for response; if false, fire-and-forget
  /// @param error_message Populated on failure
  /// @return Result with success flag and optional response future
  SendCommandResult sendControlStateCommand(
    uint8_t bus_id,
    uint8_t interface_id,
    uint8_t subsystem_id,
    SygnalControlState control_state,
    bool expect_reply,
    std::string & error_message);

  /// @brief Send control state (enable/disable) command
  /// @param interface Interface endpoint to control
  /// @param control_state MCM_CONTROL or HUMAN_CONTROL
  /// @param expect_reply If true, returns future for response; if false, fire-and-forget
  /// @param error_message Populated on failure
  /// @return Result with success flag and optional response future
  SendCommandResult sendControlStateCommand(
    InterfaceEndpoint interface, SygnalControlState control_state, bool expect_reply, std::string & error_message);

  /// @brief Send control command with value
  /// @param bus_id Bus address
  /// @param interface_id Interface to control
  /// @param value Control value
  /// @param expect_reply If true, returns future for response; if false, fire-and-forget
  /// @param error_message Populated on failure
  /// @return Result with success flag and optional response future
  SendCommandResult sendControlCommand(
    uint8_t bus_id,
    uint8_t interface_id,
    uint8_t subsystem_id,
    double value,
    bool expect_reply,
    std::string & error_message);

  /// @brief Send control command with value
  /// @param interface Interface endpoint to control
  /// @param value Control value
  /// @param expect_reply If true, returns future for response; if false, fire-and-forget
  /// @param error_message Populated on failure
  /// @return Result with success flag and optional response future
  SendCommandResult sendControlCommand(
    InterfaceEndpoint interface, double value, bool expect_reply, std::string & error_message);

  /// @brief Send relay command
  /// @param bus_id Bus address
  /// @param subsystem_id Subsystem/relay to control
  /// @param relay_state Enable or disable
  /// @param expect_reply If true, returns future for response; if false, fire-and-forget
  /// @param error_message Populated on failure
  /// @return Result with success flag and optional response future
  SendCommandResult sendRelayCommand(
    uint8_t bus_id, uint8_t subsystem_id, bool relay_state, bool expect_reply, std::string & error_message);

  /// @brief Send relay command
  /// @param interface Interface endpoint to control
  /// @param relay_state Enable or disable
  /// @param expect_reply If true, returns future for response; if false, fire-and-forget
  /// @param error_message Populated on failure
  /// @return Result with success flag and optional response future
  SendCommandResult sendRelayCommand(
    RelayEndpoint relay, bool relay_state, bool expect_reply, std::string & error_message);

  /// @brief Send an HPO ControlEnable command.
  /// @param bus_id Bus address of the target HPO. Must match one of the hpo_ids passed at construction.
  /// @param message_id HPO MessageID (8-bit identifier of the specific signal/interface on the HPO).
  /// @param enable true to grant HPO control of the signal, false to release back to human control.
  /// @param expect_reply If true, returns a future for the ControlEnableResponse; fire-and-forget otherwise.
  /// @param[out] error_message Populated on failure.
  /// @return Result with success flag and optional response future.
  SendHpoCommandResult sendHpoControlEnable(
    uint8_t bus_id, uint8_t message_id, bool enable, bool expect_reply, std::string & error_message);

  /// @brief Send an HPO ControlCommand with a float value.
  /// @param bus_id Bus address of the target HPO. Must match one of the hpo_ids passed at construction.
  /// @param message_id HPO MessageID (8-bit identifier of the specific signal/interface on the HPO).
  /// @param value Control value (encoded per DBC SIG_VALTYPE float).
  /// @param expect_reply If true, returns a future for the ControlCommandResponse; fire-and-forget otherwise.
  /// @param[out] error_message Populated on failure.
  /// @return Result with success flag and optional response future.
  SendHpoCommandResult sendHpoControlCommand(
    uint8_t bus_id, uint8_t message_id, double value, bool expect_reply, std::string & error_message);

  /// @brief Read the cached interface bits from the named HPO's latest heartbeat.
  /// @return std::nullopt if no HPO with this bus_address has been registered.
  std::optional<std::array<bool, HPO_NUM_INTERFACES>> get_hpo_interface_states(uint8_t bus_address) const;

private:
  /// @return nullptr if no MCM is configured at this bus and subsystem.
  SygnalMcmDevice * find_mcm(uint8_t bus_address, uint8_t subsystem_id) const;

  /// @return nullptr if no HPO is configured at this bus.
  SygnalHpoDevice * find_hpo(uint8_t bus_address) const;

  /// @brief All devices passed in to the constructor.
  std::vector<std::unique_ptr<SygnalDevice>> devices_;

  /// @brief A lookup table for devices by byte 0 of their frames: bus ID, with the subsystem ID in bit 7.
  std::array<SygnalDevice *, UINT8_MAX + 1> id_to_device_{};
};

}  // namespace polymath::sygnal

#endif  // SYGNAL_CAN_INTERFACE_LIB__SYGNAL_INTERFACE_SOCKETCAN_HPP_
