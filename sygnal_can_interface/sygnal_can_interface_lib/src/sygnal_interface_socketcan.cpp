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

namespace
{

constexpr uint8_t BUS_ADDRESS_MASK = 0x7F;

/// @brief Lookup table address of a device: bus ID, with the subsystem ID in bit 7.
uint8_t address_of(uint8_t bus_id, uint8_t subsystem_id)
{
  return static_cast<uint8_t>(subsystem_id << 7 | bus_id);
}

}  // namespace

SygnalInterfaceSocketcan::SygnalInterfaceSocketcan(
  std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter,
  const std::vector<McmId> & mcm_ids,
  const std::vector<HpoId> & hpo_ids,
  std::chrono::steady_clock::duration command_timeout)
{
  // Add a device to every lookup table slot it needs, throwing if a slot is already used.
  const auto add = [this](std::unique_ptr<SygnalDevice> device, const std::vector<uint8_t> & slots) {
    for (const uint8_t slot : slots) {
      if (nullptr != id_to_device_[slot]) {
        throw std::invalid_argument("Sygnal bus address " + std::to_string(slot & BUS_ADDRESS_MASK) + " is used twice");
      }
    }
    for (const uint8_t slot : slots) {
      id_to_device_[slot] = device.get();
    }
    devices_.push_back(std::move(device));
  };

  for (const McmId & id : mcm_ids) {
    if (BUS_ADDRESS_MASK < id.bus_id || MAX_SYGNAL_SUBSYSTEMS <= id.subsystem_id) {
      throw std::invalid_argument("MCM bus_id exceeds 7 bits or subsystem_id exceeds 1 bit");
    }
    add(
      std::make_unique<SygnalMcmDevice>(socketcan_adapter, id.bus_id, id.subsystem_id, command_timeout),
      {address_of(id.bus_id, id.subsystem_id)});
  }

  for (const HpoId & id : hpo_ids) {
    if (BUS_ADDRESS_MASK < id.bus_id) {
      throw std::invalid_argument("HPO bus_id exceeds 7 bits");
    }
    // HPOs ignore the subsystem bit, so they take the slots for both subsystem IDs.
    add(
      std::make_unique<SygnalHpoDevice>(socketcan_adapter, id.bus_id, command_timeout),
      {address_of(id.bus_id, 0), address_of(id.bus_id, 1)});
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
  return nullptr != device && device->handle(raw);
}

std::optional<std::array<SygnalSystemState, 5>> SygnalInterfaceSocketcan::get_interface_state(
  const uint8_t bus_address, const uint8_t subsystem_id) const
{
  const SygnalMcmDevice * mcm = find_mcm(bus_address, subsystem_id);
  if (nullptr == mcm) {
    return std::nullopt;
  }
  return mcm->get_interface_states();
}

std::optional<SygnalSystemState> SygnalInterfaceSocketcan::get_sygnal_mcm_state(
  const uint8_t bus_address, const uint8_t subsystem_id) const
{
  const SygnalMcmDevice * mcm = find_mcm(bus_address, subsystem_id);
  if (nullptr == mcm) {
    return std::nullopt;
  }
  return mcm->get_mcm_state();
}

SendCommandResult SygnalInterfaceSocketcan::sendControlStateCommand(
  uint8_t bus_id,
  uint8_t interface_id,
  uint8_t subsystem_id,
  SygnalControlState control_state,
  bool expect_reply,
  std::string & error_message)
{
  SygnalMcmDevice * mcm = find_mcm(bus_id, subsystem_id);
  if (nullptr == mcm) {
    error_message += "No MCM registered at bus address " + std::to_string(bus_id) + " subsystem " +
                     std::to_string(subsystem_id) + "\n";
    return {false, std::nullopt};
  }
  return mcm->send_control_state_command(interface_id, control_state, expect_reply, error_message);
}

SendCommandResult SygnalInterfaceSocketcan::sendControlStateCommand(
  InterfaceEndpoint interface, SygnalControlState control_state, bool expect_reply, std::string & error_message)
{
  return sendControlStateCommand(
    interface.bus_id, interface.interface_id, interface.subsystem_id, control_state, expect_reply, error_message);
}

SendCommandResult SygnalInterfaceSocketcan::sendControlCommand(
  uint8_t bus_id,
  uint8_t interface_id,
  uint8_t subsystem_id,
  double value,
  bool expect_reply,
  std::string & error_message)
{
  SygnalMcmDevice * mcm = find_mcm(bus_id, subsystem_id);
  if (nullptr == mcm) {
    error_message += "No MCM registered at bus address " + std::to_string(bus_id) + " subsystem " +
                     std::to_string(subsystem_id) + "\n";
    return {false, std::nullopt};
  }
  return mcm->send_control_command(interface_id, value, expect_reply, error_message);
}

SendCommandResult SygnalInterfaceSocketcan::sendControlCommand(
  InterfaceEndpoint interface, double value, bool expect_reply, std::string & error_message)
{
  return sendControlCommand(
    interface.bus_id, interface.interface_id, interface.subsystem_id, value, expect_reply, error_message);
}

SendCommandResult SygnalInterfaceSocketcan::sendRelayCommand(
  uint8_t bus_id, uint8_t subsystem_id, bool relay_state, bool expect_reply, std::string & error_message)
{
  SygnalMcmDevice * mcm = find_mcm(bus_id, subsystem_id);
  if (nullptr == mcm) {
    error_message += "No MCM registered at bus address " + std::to_string(bus_id) + " subsystem " +
                     std::to_string(subsystem_id) + "\n";
    return {false, std::nullopt};
  }
  return mcm->send_relay_command(relay_state, expect_reply, error_message);
}

SendCommandResult SygnalInterfaceSocketcan::sendRelayCommand(
  RelayEndpoint relay, bool relay_state, bool expect_reply, std::string & error_message)
{
  return sendRelayCommand(relay.bus_id, relay.subsystem_id, relay_state, expect_reply, error_message);
}

SendHpoCommandResult SygnalInterfaceSocketcan::sendHpoControlEnable(
  uint8_t bus_id, uint8_t message_id, bool enable, bool expect_reply, std::string & error_message)
{
  SygnalHpoDevice * hpo = find_hpo(bus_id);
  if (nullptr == hpo) {
    error_message += "No HPO registered at bus address " + std::to_string(bus_id) + "\n";
    return {false, std::nullopt};
  }
  return hpo->send_control_enable(message_id, enable, expect_reply, error_message);
}

SendHpoCommandResult SygnalInterfaceSocketcan::sendHpoControlCommand(
  uint8_t bus_id, uint8_t message_id, double value, bool expect_reply, std::string & error_message)
{
  SygnalHpoDevice * hpo = find_hpo(bus_id);
  if (nullptr == hpo) {
    error_message += "No HPO registered at bus address " + std::to_string(bus_id) + "\n";
    return {false, std::nullopt};
  }
  return hpo->send_control_command(message_id, value, expect_reply, error_message);
}

std::optional<std::array<bool, HPO_NUM_INTERFACES>> SygnalInterfaceSocketcan::get_hpo_interface_states(
  uint8_t bus_address) const
{
  const SygnalHpoDevice * hpo = find_hpo(bus_address);
  if (nullptr == hpo) {
    return std::nullopt;
  }
  return hpo->get_interface_states();
}

SygnalMcmDevice * SygnalInterfaceSocketcan::find_mcm(uint8_t bus_address, uint8_t subsystem_id) const
{
  if (BUS_ADDRESS_MASK < bus_address || MAX_SYGNAL_SUBSYSTEMS <= subsystem_id) {
    return nullptr;
  }
  return dynamic_cast<SygnalMcmDevice *>(id_to_device_[address_of(bus_address, subsystem_id)]);
}

SygnalHpoDevice * SygnalInterfaceSocketcan::find_hpo(uint8_t bus_address) const
{
  if (BUS_ADDRESS_MASK < bus_address) {
    return nullptr;
  }
  return dynamic_cast<SygnalHpoDevice *>(id_to_device_[bus_address]);
}

}  // namespace polymath::sygnal
