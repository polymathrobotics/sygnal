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
#include <future>
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

/// @brief Sygnal MCMs and HPOs on one SocketCAN bus.
///        Received frames are dispatched through a lookup table keyed on data byte 0
///        (subsystem ID bit 7, bus ID bits 0-6).
///        Thread-safe: parse() may run on the reception thread while other methods run elsewhere.
class SygnalInterfaceSocketcan
{
public:
  /// @param socketcan_adapter Adapter that devices send commands on.
  /// @param control_devices Devices to manage. MCM and HPO are supported.
  /// @throws std::invalid_argument on an unsupported type, a bus_id above 127, a subsystem_id above 1,
  ///         a duplicate device, or a bus_id shared by two device types.
  SygnalInterfaceSocketcan(
    std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter,
    const std::vector<SygnalControlDevice> & control_devices);

  /// @brief Route a received frame to the device it addresses.
  /// @return true if a configured device consumed the frame.
  bool parse(const socketcan::CanFrame & frame);

  /// @brief Get SystemState from the device's latest stashed heartbeat.
  ///        Returns FAIL_HARD until the first one.
  /// @throws std::invalid_argument if the device is not configured.
  SygnalSystemState get_device_system_state(const SygnalControlDevice & device) const;

  /// @brief Get whether the endpoint is in MCM_CONTROL or HUMAN_CONTROL based on the device's latest stashed heartbeat.
  ///        Returns FAIL_HARD until the first stashed heartbeat is received.
  /// @throws std::invalid_argument if the device is not configured or has no such endpoint.
  SygnalSystemState get_endpoint_system_state(const SygnalEndpoint & endpoint) const;

  /// @brief Value from the endpoint's latest command response;
  ///        Returns 0 or false until the first command is sent.
  /// @throws std::invalid_argument if the device is not configured or has no such endpoint.
  double get_endpoint_value(const SygnalEndpoint & endpoint) const;

  /// @brief Fault status of an MCM subsystem.
  /// @throws std::invalid_argument if no MCM is configured at the device's address.
  McmFaultStatus get_device_faults(const SygnalControlDevice & device) const;

  /// @brief Enable or disable control of an endpoint.
  /// @param expect_reply Return a future that resolves from the endpoint's enable response.
  /// @throws std::invalid_argument if the device is not configured.
  EndpointStateCommandResult send_endpoint_state_command(
    const SygnalEndpoint & endpoint, bool enable, bool expect_reply);

  /// @brief Set an endpoint's value.
  /// @param expect_reply Return a future that resolves from the endpoint's command response.
  /// @throws std::invalid_argument if the device is not configured.
  EndpointValueCommandResult send_endpoint_value_command(
    const SygnalEndpoint & endpoint, double value, bool expect_reply);

private:
  /// @brief Helper to compute the BUS_ID + SUBSYSTEM_ID combination used as the lookup table key.
  /// @return std::nullopt if bus_id or subsystem_id is out of range.
  static std::optional<uint8_t> address_of(const SygnalControlDevice & device);

  /// @brief Helper to create human-readable description of the device.
  static std::string describe(const SygnalControlDevice & device);

  /// @brief Helper to lookup the device instance for the given device.
  /// @throws std::invalid_argument if the lookup table has no entry for the device's ID.
  SygnalDevice & device_at(const SygnalControlDevice & device) const;

  static constexpr uint8_t BUS_ADDRESS_MASK = 0x7F;
  static constexpr size_t NUM_ADDRESSES = UINT8_MAX + 1;

  /// @brief Vector of all device instances managed by this interface.
  ///        One per MCM, HPO, and other supported device types.
  std::vector<std::unique_ptr<SygnalDevice>> devices_;

  /// @brief Lookup table from ID (BUS_ID + SUBSYSTEM_ID) to device instance
  ///        nullptr where none is configured.
  std::array<SygnalDevice *, NUM_ADDRESSES> id_to_device_{};
};

}  // namespace polymath::sygnal

#endif  // SYGNAL_CAN_INTERFACE_LIB__SYGNAL_INTERFACE_SOCKETCAN_HPP_
