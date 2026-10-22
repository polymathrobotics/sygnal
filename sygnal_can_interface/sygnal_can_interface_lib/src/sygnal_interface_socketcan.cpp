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


#include "sygnal_can_interface_lib/sygnal_interface_socketcan.hpp"

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "sygnal_can_interface_lib/sygnal_hpo_device.hpp"
#include "sygnal_can_interface_lib/sygnal_mcm_device.hpp"

namespace polymath::sygnal
{

namespace
{

constexpr uint8_t BUS_ADDRESS_MASK = 0x7F;
constexpr uint8_t SUBSYSTEM_BIT = 0x80;

/// @brief returns the address of the device for the lookup table.
///        The address is composed of the bus_id (and subsystem_id if type is MCM).
/// @return std::nullopt if bus_id or subsystem_id is out of range.
std::optional<uint8_t> address_of(const SygnalControlDevice & device)
{
  // Retrieve subsystem if type is MCM. Otherwise, subsystem_id is 0.
  const uint8_t subsystem_id = SygnalControlDeviceType::MCM == device.type ? device.subsystem_id : 0;

  // Return std::nullopt if bus_id or subsystem_id is out of range.
  if (BUS_ADDRESS_MASK < device.bus_id || 1 < subsystem_id) {
    return std::nullopt;
  }

  // Compose the address with the bus_id and subsystem_id.
  return static_cast<uint8_t>(subsystem_id << 7 | device.bus_id);
}

}  // namespace

SygnalInterfaceSocketcan::SygnalInterfaceSocketcan(
  std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter,
  const std::vector<SygnalControlDevice> & control_devices,
  std::chrono::steady_clock::duration command_timeout)
{
  // Loop over all control devices passed.
  for (const SygnalControlDevice & device : control_devices) {
    // Get the address for the lookup table.
    const std::optional<uint8_t> address = address_of(device);

    // Throw if the address is invalid.
    if (!address) {
      throw std::invalid_argument(describe(device) + ": bus_id exceeds 7 bits or subsystem_id exceeds 1 bit");
    }

    // Create an instance of the appropriate SygnalDevice subclass based on the device type.
    std::unique_ptr<SygnalDevice> instance;
    std::vector<uint8_t> addresses{*address};
    switch (device.type) {
      case SygnalControlDeviceType::MCM:
        instance = std::make_unique<SygnalMcmDevice>(socketcan_adapter, device, command_timeout);
        break;
      case SygnalControlDeviceType::HPO:
        instance = std::make_unique<SygnalHpoDevice>(socketcan_adapter, device, command_timeout);
        // Non-MCM devices ignore the subsystem bit, so they take the slots for both subsystem IDs.
        // That way, arbitration can index the lookup table with byte 0 from the CAN message and
        // still find the device, regardless of the subsystem bit.
        addresses.push_back(static_cast<uint8_t>(*address ^ SUBSYSTEM_BIT));
        break;
    }

    // Throw if any slot this device needs is already used in the lookup table.
    for (const uint8_t slot : addresses) {
      if (nullptr != id_to_device_[slot]) {
        throw std::invalid_argument(
          describe(device) + ": address already used by " + describe(id_to_device_[slot]->device()));
      }
    }

    // Assign the instance to all of its slots in the lookup table.
    for (const uint8_t slot : addresses) {
      id_to_device_[slot] = instance.get();
    }

    // Add the instance to the list of managed devices.
    devices_.push_back(std::move(instance));
  }
}

bool SygnalInterfaceSocketcan::parse(const socketcan::CanFrame & frame)
{
  // Get the raw CAN frame.
  const can_frame raw = frame.get_frame();

  // Ignore empty frames since we need the first byte for arbitration.
  if (0 == raw.len) {
    return false;
  }

  // Byte 0 holds the BUS_ID + SUBSYSTEM_ID; an HPO fills both subsystem slots for its bus.
  SygnalDevice * device = id_to_device_[raw.data[0]];
  return nullptr != device && device->handle(raw, frame.get_receive_time(), frame.get_bus_time());
}

Stamped<SygnalSystemState> SygnalInterfaceSocketcan::get_device_system_state(const SygnalControlDevice & device) const
{
  // Pass through to the device function.
  return device_at(device).get_device_system_state();
}

Stamped<SygnalSystemState> SygnalInterfaceSocketcan::get_interface_control_state(const SygnalEndpoint & endpoint) const
{
  // Pass through to the device function.
  return device_at(endpoint.device).get_interface_control_state(endpoint);
}

Stamped<double> SygnalInterfaceSocketcan::get_control_command_value(const SygnalEndpoint & endpoint) const
{
  // Pass through to the device function.
  return device_at(endpoint.device).get_control_command_value(endpoint);
}

Stamped<McmFaultStatus> SygnalInterfaceSocketcan::get_device_faults(const SygnalControlDevice & device) const
{
  // Pass through to the device function.
  return device_at(device).get_device_faults();
}

std::vector<Stamped<SygnalError>> SygnalInterfaceSocketcan::get_new_device_errors(const SygnalControlDevice & device)
{
  // Pass through to the device function.
  return device_at(device).get_new_device_errors();
}

ControlEnableCommandResult SygnalInterfaceSocketcan::send_control_enable_command(
  const SygnalEndpoint & endpoint, bool enable, bool expect_reply)
{
  // Pass through to the device function.
  return device_at(endpoint.device).send_control_enable_command(endpoint, enable, expect_reply);
}

ControlCommandResult SygnalInterfaceSocketcan::send_control_command(
  const SygnalEndpoint & endpoint, double value, bool expect_reply)
{
  // Pass through to the device function.
  return device_at(endpoint.device).send_control_command(endpoint, value, expect_reply);
}

SygnalDevice & SygnalInterfaceSocketcan::device_at(const SygnalControlDevice & device) const
{
  // Get the device address
  const std::optional<uint8_t> address = address_of(device);

  // Throw if no device is found at the given address.
  SygnalDevice * found = address ? id_to_device_[*address] : nullptr;
  if (nullptr == found) {
    throw std::invalid_argument(describe(device) + " is not configured");
  }

  // Throw if the device type does not match the expected type.
  if (found->device().type != device.type) {
    throw std::invalid_argument(
      describe(device) + " is not configured; that address is " + describe(found->device()));
  }

  // Return the found device instance.
  return *found;
}

}  // namespace polymath::sygnal
