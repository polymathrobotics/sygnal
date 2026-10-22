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

#include "sygnal_can_interface_lib/sygnal_hpo_device.hpp"

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include "hpo_control/hpo_control.hpp"
#include "hpo_heartbeat/hpo_heartbeat.hpp"

namespace polymath::sygnal
{

SygnalHpoDevice::SygnalHpoDevice(
  std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter,
  const SygnalControlDevice & device,
  std::chrono::steady_clock::duration command_timeout)
: SygnalDevice(std::move(socketcan_adapter), device, command_timeout)
{
  // Set initial states for the HPO interfaces.
  interface_states_.fill(Stamped<SygnalSystemState>{SygnalSystemState::FAIL_HARD, {}});

  initializeCANHandlerCallbacks();
}

void SygnalHpoDevice::initializeCANHandlerCallbacks()
{
  // Handle HPO Heartbeat messages.
  can_handler_.set_handler<hpo_heartbeat::Heartbeat>([this](const hpo_heartbeat::Heartbeat & msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // Update the system and interface states with the new heartbeat information.
    system_state_ = {static_cast<SygnalSystemState>(msg.system_state), msg.bus_time};
    interface_states_ = {
      {{static_cast<SygnalSystemState>(msg.interface0_state), msg.bus_time},
       {static_cast<SygnalSystemState>(msg.interface1_state), msg.bus_time},
       {static_cast<SygnalSystemState>(msg.interface2_state), msg.bus_time},
       {static_cast<SygnalSystemState>(msg.interface3_state), msg.bus_time}}};

    // Update the interface values with the pin state from the heartbeat: 0 HiZ, 1 Pull_Down, 2 Pull_Up.
    values_[0] = {msg.interface0_value, msg.bus_time};
    values_[1] = {msg.interface1_value, msg.bus_time};
    values_[2] = {msg.interface2_value, msg.bus_time};
    values_[3] = {msg.interface3_value, msg.bus_time};
  });

  // Handle HPO Control Enable Response messages.
  can_handler_.set_handler<hpo_control::ControlEnableResponse>([this](const hpo_control::ControlEnableResponse & msg) {
    // Parse the message ID and enable state from the message.
    const uint8_t id = static_cast<uint8_t>(msg.message_id);
    const SygnalSystemState state = static_cast<SygnalSystemState>(msg.enable);

    std::lock_guard<std::mutex> lock(state_mutex_);

    // Update the interface state if the ID is valid.
    if (HPO_NUM_INTERFACES > id) {
      interface_states_[id] = {state, msg.bus_time};
    }

    // Fulfill the pending promise for this message ID with the enable state.
    control_enable_responses_.fulfill(id, {state, msg.bus_time});
  });

  // Handle HPO Control Command Response messages.
  can_handler_.set_handler<hpo_control::ControlCommandResponse>(
    [this](const hpo_control::ControlCommandResponse & msg) {
      // Parse the message ID from the message.
      const uint8_t id = static_cast<uint8_t>(msg.message_id);

      std::lock_guard<std::mutex> lock(state_mutex_);

      // Update the value for this message ID.
      values_[id] = {msg.value, msg.bus_time};

      // Fulfill the pending promise for this message ID with the command value.
      control_command_responses_.fulfill(id, values_[id]);
    });
}

Stamped<SygnalSystemState> SygnalHpoDevice::get_interface_control_state(const SygnalEndpoint & endpoint) const
{
  // Check if the endpoint type is HPO_INTERFACE; otherwise, delegate to the base class for error handling.
  if (SygnalControlEndpointType::HPO_INTERFACE != endpoint.type) {
    return SygnalDevice::get_interface_control_state(endpoint);
  }

  // Throw if the message ID has no interface state.
  if (HPO_NUM_INTERFACES <= endpoint.endpoint_id) {
    throw std::invalid_argument(
      "Sygnal Endpoint " + std::string(endpoint.name) + " has message ID " + std::to_string(endpoint.endpoint_id) +
      "; HPO interface states are only reported for message IDs 0-3.");
  }

  // Return the interface control state for the specified message ID.
  std::lock_guard<std::mutex> lock(state_mutex_);
  return interface_states_[endpoint.endpoint_id];
}

Stamped<double> SygnalHpoDevice::get_control_command_value(const SygnalEndpoint & endpoint) const
{
  // If the endpoint is not an HPO_INTERFACE, delegate to the base class for error handling.
  if (SygnalControlEndpointType::HPO_INTERFACE != endpoint.type) {
    return SygnalDevice::get_control_command_value(endpoint);
  }

  // Return the control command value for the specified message ID.
  std::lock_guard<std::mutex> lock(state_mutex_);
  return values_[endpoint.endpoint_id];
}

ControlEnableCommandResult SygnalHpoDevice::send_control_enable_command(
  const SygnalEndpoint & endpoint, bool enable, bool expect_reply)
{
  // If the endpoint is not an HPO_INTERFACE, delegate to the base class for error handling.
  if (SygnalControlEndpointType::HPO_INTERFACE != endpoint.type) {
    return SygnalDevice::send_control_enable_command(endpoint, enable, expect_reply);
  }

  // Construct the control enable message.
  hpo_control::ControlEnable msg;
  msg.bus_address = device().bus_id;
  msg.message_id = endpoint.endpoint_id;
  msg.enable = enable ? 1 : 0;
  const socketcan::CanFrame frame = to_frame_with_crc(msg);

  // Push a future for the expected reply if needed.
  std::optional<std::future<Stamped<SygnalSystemState>>> future;
  if (expect_reply) {
    future = control_enable_responses_.push(endpoint.endpoint_id);
  }

  // Send the control enable message.
  const std::optional<std::string> send_error = socketcan_adapter_->send(frame);

  // Withdraw the future if sending the message failed.
  if (send_error.has_value()) {
    if (future) {
      control_enable_responses_.withdraw_newest(endpoint.endpoint_id);
    }
    return {false, send_error.value(), std::nullopt};
  }

  // Return success if the message was sent successfully.
  return {true, "", std::move(future)};
}

ControlCommandResult SygnalHpoDevice::send_control_command(
  const SygnalEndpoint & endpoint, double value, bool expect_reply)
{
  // If the endpoint is not an HPO_INTERFACE, delegate to the base class for error handling.
  if (SygnalControlEndpointType::HPO_INTERFACE != endpoint.type) {
    return SygnalDevice::send_control_command(endpoint, value, expect_reply);
  }

  // Construct the control command message.
  hpo_control::ControlCommand msg;
  msg.bus_address = device().bus_id;
  msg.message_id = endpoint.endpoint_id;
  /// TODO: (Ryan) Increment Count8 per command once the HPO's count check is enabled.
  msg.count8 = 0;
  msg.value = value;
  const socketcan::CanFrame frame = to_frame_with_crc(msg);

  // Push a future for the expected reply if needed.
  std::optional<std::future<Stamped<double>>> future;
  if (expect_reply) {
    future = control_command_responses_.push(endpoint.endpoint_id);
  }

  // Send the control command message.
  const std::optional<std::string> send_error = socketcan_adapter_->send(frame);

  // Withdraw the future if sending the message failed.
  if (send_error.has_value()) {
    if (future) {
      control_command_responses_.withdraw_newest(endpoint.endpoint_id);
    }
    return {false, send_error.value(), std::nullopt};
  }

  // Return success if the message was sent successfully.
  return {true, "", std::move(future)};
}

}  // namespace polymath::sygnal
