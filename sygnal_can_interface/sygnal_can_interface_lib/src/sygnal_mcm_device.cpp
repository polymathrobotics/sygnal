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

#include "sygnal_can_interface_lib/sygnal_mcm_device.hpp"

#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include "mcm_control/mcm_control.hpp"
#include "mcm_heartbeat/mcm_heartbeat.hpp"
#include "mcm_relay/mcm_relay.hpp"

namespace polymath::sygnal
{

SygnalMcmDevice::SygnalMcmDevice(
  std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter,
  uint8_t bus_address,
  uint8_t subsystem_id,
  std::chrono::steady_clock::duration command_timeout)
: SygnalDevice(std::move(socketcan_adapter), bus_address, command_timeout)
, control_enable_responses_(command_timeout_)
, control_command_responses_(command_timeout_)
, relay_responses_(command_timeout_)
, subsystem_id_(subsystem_id)
{
  // Set initial states for the MCM interfaces.
  interface_states_.fill(SygnalSystemState::FAIL_HARD);

  initializeCANHandlerCallbacks();
}

void SygnalMcmDevice::initializeCANHandlerCallbacks()
{
  // Handle MCM Heartbeat messages.
  can_handler_.set_handler<mcm_heartbeat::Heartbeat>([this](const mcm_heartbeat::Heartbeat & msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // Update the system and interface states with the new heartbeat information.
    mcm_state_ = static_cast<SygnalSystemState>(msg.system_state);
    interface_states_ = {
      static_cast<SygnalSystemState>(msg.interface0_state),
      static_cast<SygnalSystemState>(msg.interface1_state),
      static_cast<SygnalSystemState>(msg.interface2_state),
      static_cast<SygnalSystemState>(msg.interface3_state),
      static_cast<SygnalSystemState>(msg.interface4_state)};
  });

  // Handle MCM Control Enable Response messages.
  can_handler_.set_handler<mcm_control::ControlEnableResponse>([this](const mcm_control::ControlEnableResponse & msg) {
    // Fulfill the pending promise for this interface ID with the enable response.
    const uint8_t id = static_cast<uint8_t>(msg.interface_id);
    control_enable_responses_.fulfill(
      id, {SygnalControlCommandResponseType::ENABLE, id, get_bus_address(), subsystem_id_, msg.enable});
  });

  // Handle MCM Control Command Response messages.
  can_handler_.set_handler<mcm_control::ControlCommandResponse>(
    [this](const mcm_control::ControlCommandResponse & msg) {
      // Fulfill the pending promise for this interface ID with the command value.
      const uint8_t id = static_cast<uint8_t>(msg.interface_id);
      control_command_responses_.fulfill(
        id, {SygnalControlCommandResponseType::CONTROL, id, get_bus_address(), subsystem_id_, msg.value});
    });

  // Handle MCM Relay Command Response messages.
  can_handler_.set_handler<mcm_relay::RelayCommandResponse>([this](const mcm_relay::RelayCommandResponse & msg) {
    // Fulfill the pending promise for the relay command.
    relay_responses_.fulfill(
      0, {SygnalControlCommandResponseType::RELAY, subsystem_id_, get_bus_address(), subsystem_id_, msg.enable});
  });
}

uint8_t SygnalMcmDevice::get_subsystem_id() const
{
  return subsystem_id_;
}

std::array<SygnalSystemState, 5> SygnalMcmDevice::get_interface_states() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return interface_states_;
}

SygnalSystemState SygnalMcmDevice::get_mcm_state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return mcm_state_;
}

SendCommandResult SygnalMcmDevice::send_control_state_command(
  uint8_t interface_id, SygnalControlState control_state, bool expect_reply, std::string & error_message)
{
  // Fail if the interface ID is out of range.
  if (interface_id >= MAX_SYGNAL_INTERFACES) {
    error_message += "Interface ID exceeds maximum allowed interfaces. \n";
    return {false, std::nullopt};
  }

  // Construct the control enable message.
  mcm_control::ControlEnable msg;
  msg.bus_address = get_bus_address();
  msg.sub_system_id = subsystem_id_;
  msg.interface_id = interface_id;
  msg.enable = static_cast<uint8_t>(control_state);
  const socketcan::CanFrame frame = to_frame_with_crc(msg);

  // Push a future for the expected reply if needed.
  std::optional<std::future<SygnalControlCommandResponse>> future;
  if (expect_reply) {
    future = control_enable_responses_.push(interface_id);
  }

  // Send the control enable message.
  const std::optional<std::string> send_error = socketcan_adapter_->send(frame);

  // Withdraw the future if sending the message failed.
  if (send_error.has_value()) {
    if (future) {
      control_enable_responses_.withdraw_newest(interface_id);
    }
    error_message += "Failed to send control state command: " + send_error.value() + "\n";
    return {false, std::nullopt};
  }

  // Return success if the message was sent successfully.
  return {true, std::move(future)};
}

SendCommandResult SygnalMcmDevice::send_control_command(
  uint8_t interface_id, double value, bool expect_reply, std::string & error_message)
{
  // Fail if the interface ID is out of range.
  if (interface_id >= MAX_SYGNAL_INTERFACES) {
    error_message += "Interface ID exceeds maximum allowed interfaces. \n";
    return {false, std::nullopt};
  }

  // Construct the control command message.
  mcm_control::ControlCommand msg;
  msg.bus_address = get_bus_address();
  msg.sub_system_id = subsystem_id_;
  msg.interface_id = interface_id;
  msg.count8 = 0;
  msg.value = value;
  const socketcan::CanFrame frame = to_frame_with_crc(msg);

  // Push a future for the expected reply if needed.
  std::optional<std::future<SygnalControlCommandResponse>> future;
  if (expect_reply) {
    future = control_command_responses_.push(interface_id);
  }

  // Send the control command message.
  const std::optional<std::string> send_error = socketcan_adapter_->send(frame);

  // Withdraw the future if sending the message failed.
  if (send_error.has_value()) {
    if (future) {
      control_command_responses_.withdraw_newest(interface_id);
    }
    error_message += "Failed to send control command: " + send_error.value() + "\n";
    return {false, std::nullopt};
  }

  // Return success if the message was sent successfully.
  return {true, std::move(future)};
}

SendCommandResult SygnalMcmDevice::send_relay_command(bool relay_state, bool expect_reply, std::string & error_message)
{
  // Construct the relay command message.
  mcm_relay::RelayCommand msg;
  msg.bus_address = get_bus_address();
  msg.subsystem_id = subsystem_id_;
  msg.enable = relay_state ? 1 : 0;
  const socketcan::CanFrame frame = to_frame_with_crc(msg);

  // Push a future for the expected reply if needed.
  std::optional<std::future<SygnalControlCommandResponse>> future;
  if (expect_reply) {
    future = relay_responses_.push(0);
  }

  // Send the relay command message.
  const std::optional<std::string> send_error = socketcan_adapter_->send(frame);

  // Withdraw the future if sending the message failed.
  if (send_error.has_value()) {
    if (future) {
      relay_responses_.withdraw_newest(0);
    }
    error_message += "Failed to send relay command: " + send_error.value() + "\n";
    return {false, std::nullopt};
  }

  // Return success if the message was sent successfully.
  return {true, std::move(future)};
}

}  // namespace polymath::sygnal
