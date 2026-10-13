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

namespace polymath::sygnal
{
SygnalInterfaceSocketcan::SygnalInterfaceSocketcan(
  std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter,
  const std::vector<SygnalControlDevice> & control_devices)
{
  for (const SygnalControlDevice & device : control_devices) {
    // Throw if the device's bus_id or subsystem_id is out of range.
    const std::optional<uint8_t> address = address_of(device);
    if (!address) {
      throw std::invalid_argument(describe(device) + ": bus_id exceeds 7 bits or subsystem_id exceeds 1 bit");
    }

    // Build a lookup table called id_to_device_ to arbitrate messages messages later.
    // The map key is the devices BUS_ID (7 bits) + SUBSYSTEM_ID (1 bit) combination.
    // MCM uses the subsystem bit, so it only occupies one slot in the address map.
    // All other devices don't use the subsystem bit, so we just put them in both possible slots for their bus to make the arbitration work correctly.

    // Throw if the address is already in use.
    if (nullptr != id_to_device_[*address]) {
      throw std::invalid_argument(describe(device) + ": address already in use");
    }

    // Create the device instance based on its type.
    switch (device.type) {
      case SygnalControlDeviceType::MCM: {
        devices_.push_back(std::make_unique<SygnalMcmDevice>(socketcan_adapter, device.bus_id, device.subsystem_id));
        break;
      }
      case SygnalControlDeviceType::HPO:
        devices_.push_back(std::make_unique<SygnalHpoDevice>(socketcan_adapter, device.bus_id));
        break;
      case SygnalControlDeviceType::CB:
      case SygnalControlDeviceType::IO:
      case SygnalControlDeviceType::NONE:
        throw std::invalid_argument(describe(device) + ": unsupported device type");
    }

    // Assign the newly created device instance to it's slot in the lookup table
    id_to_device_[*address] = devices_.back().get();
    if (SygnalControlDeviceType::MCM != device.type) {
      // Push the address with the subsystem bit inverted.
      id_to_device_[static_cast<uint8_t>(*address ^ 0x80)] = devices_.back().get();
    }
  }
}

bool SygnalInterfaceSocketcan::parse(const socketcan::CanFrame & frame)
{
  const can_frame raw = frame.get_frame();
  // Ignore empty frames since we need the first byte for arbitration.
  if (0 == raw.len) {
    return false;
  }

  // Lookup the device based on the BUS_ID + SUBSYSTEM_ID combination in the first data byte.
  // NOTE: For devices that don't subsystem, this works anyway because we have populated both
  //       possible slots for their bus in the lookup table.
  SygnalDevice * device = id_to_device_[raw.data[0]];

  // Call the device's handle method if it exists. Return false if the device is not found.
  return nullptr != device && device->handle(raw, frame.get_receive_time(), frame.get_bus_time());
}

SygnalSystemState SygnalInterfaceSocketcan::get_device_system_state(const SygnalControlDevice & device) const
{
  return device_at(device).get_system_state();
}

SygnalSystemState SygnalInterfaceSocketcan::get_endpoint_system_state(const SygnalEndpoint & endpoint) const
{
  return device_at(endpoint.device).get_endpoint_state(endpoint);
}

double SygnalInterfaceSocketcan::get_endpoint_value(const SygnalEndpoint & endpoint) const
{
  return device_at(endpoint.device).get_endpoint_value(endpoint);
}

McmFaultStatus SygnalInterfaceSocketcan::get_device_faults(const SygnalControlDevice & device) const
{}

EndpointStateCommandResult SygnalInterfaceSocketcan::send_endpoint_state_command(
  const SygnalEndpoint & endpoint, bool enable, bool expect_reply)
{
  return device_at(endpoint.device).send_endpoint_state_command(endpoint, enable, expect_reply);
}

EndpointValueCommandResult SygnalInterfaceSocketcan::send_endpoint_value_command(
  const SygnalEndpoint & endpoint, double value, bool expect_reply)
{
  return device_at(endpoint.device).send_endpoint_value_command(endpoint, value, expect_reply);
}

SygnalDevice & SygnalInterfaceSocketcan::device_at(const SygnalControlDevice & device) const
{
  const std::optional<uint8_t> address = address_of(device);
  SygnalDevice * found = address ? id_to_device_[*address] : nullptr;
  if (nullptr == found) {
    throw std::invalid_argument(describe(device) + ": not a configured device");
  }
  return *found;
}

std::optional<uint8_t> SygnalInterfaceSocketcan::address_of(const SygnalControlDevice & device)
{
  if (BUS_ADDRESS_MASK < device.bus_id || 1 < device.subsystem_id) {
    return std::nullopt;
  }
  return static_cast<uint8_t>(device.subsystem_id << 7 | device.bus_id);
}

std::string SygnalInterfaceSocketcan::describe(const SygnalControlDevice & device)
{
  return "bus_id " + std::to_string(device.bus_id) + ", subsystem_id " + std::to_string(device.subsystem_id);
}

}  // namespace polymath::sygnal
