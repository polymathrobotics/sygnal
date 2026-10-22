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

#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include "hpo_control/hpo_control.hpp"
#include "hpo_heartbeat/hpo_heartbeat.hpp"

namespace polymath::sygnal
{

SygnalHpoDevice::SygnalHpoDevice(
  std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter,
  uint8_t bus_address,
  std::chrono::steady_clock::duration command_timeout)
: SygnalDevice(std::move(socketcan_adapter), bus_address, command_timeout)
, control_enable_responses_(command_timeout_)
, control_command_responses_(command_timeout_)
{
  initializeCANHandlerCallbacks();
}

void SygnalHpoDevice::initializeCANHandlerCallbacks()
{
  // Handle HPO Heartbeat messages.
  can_handler_.set_handler<hpo_heartbeat::Heartbeat>([this](const hpo_heartbeat::Heartbeat & msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // Update the interface states with the new heartbeat information.
    interface_states_ = {
      0 != msg.interface0_state, 0 != msg.interface1_state, 0 != msg.interface2_state, 0 != msg.interface3_state};
  });

  // Handle HPO Control Enable Response messages.
  can_handler_.set_handler<hpo_control::ControlEnableResponse>([this](const hpo_control::ControlEnableResponse & msg) {
    // Fulfill the pending promise for this message ID with the enable response.
    const uint8_t id = static_cast<uint8_t>(msg.message_id);
    control_enable_responses_.fulfill(id, {get_bus_address(), id, 0.0, 0 != msg.enable, true});
  });

  // Handle HPO Control Command Response messages.
  can_handler_.set_handler<hpo_control::ControlCommandResponse>(
    [this](const hpo_control::ControlCommandResponse & msg) {
      // Fulfill the pending promise for this message ID with the command value.
      const uint8_t id = static_cast<uint8_t>(msg.message_id);
      control_command_responses_.fulfill(id, {get_bus_address(), id, msg.value, false, false});
    });
}

std::array<bool, HPO_NUM_INTERFACES> SygnalHpoDevice::get_interface_states() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return interface_states_;
}

SendHpoCommandResult SygnalHpoDevice::send_control_enable(
  uint8_t message_id, bool enable, bool expect_reply, std::string & error_message)
{
  // Construct the control enable message.
  hpo_control::ControlEnable msg;
  msg.bus_address = get_bus_address();
  msg.message_id = message_id;
  msg.enable = enable ? 1 : 0;
  const socketcan::CanFrame frame = to_frame_with_crc(msg);

  // Push a future for the expected reply if needed.
  std::optional<std::future<HpoControlResponse>> future;
  if (expect_reply) {
    future = control_enable_responses_.push(message_id);
  }

  // Send the control enable message.
  const std::optional<std::string> send_error = socketcan_adapter_->send(frame);

  // Withdraw the future if sending the message failed.
  if (send_error.has_value()) {
    if (future) {
      control_enable_responses_.withdraw_newest(message_id);
    }
    error_message += "Failed to send HPO control enable: " + send_error.value() + "\n";
    return {false, std::nullopt};
  }

  // Return success if the message was sent successfully.
  return {true, std::move(future)};
}

SendHpoCommandResult SygnalHpoDevice::send_control_command(
  uint8_t message_id, double value, bool expect_reply, std::string & error_message)
{
  // Construct the control command message.
  hpo_control::ControlCommand msg;
  msg.bus_address = get_bus_address();
  msg.message_id = message_id;
  /// TODO: (Ryan) Increment Count8 per command once the HPO's count check is enabled.
  msg.count8 = 0;
  msg.value = value;
  const socketcan::CanFrame frame = to_frame_with_crc(msg);

  // Push a future for the expected reply if needed.
  std::optional<std::future<HpoControlResponse>> future;
  if (expect_reply) {
    future = control_command_responses_.push(message_id);
  }

  // Send the control command message.
  const std::optional<std::string> send_error = socketcan_adapter_->send(frame);

  // Withdraw the future if sending the message failed.
  if (send_error.has_value()) {
    if (future) {
      control_command_responses_.withdraw_newest(message_id);
    }
    error_message += "Failed to send HPO control command: " + send_error.value() + "\n";
    return {false, std::nullopt};
  }

  // Return success if the message was sent successfully.
  return {true, std::move(future)};
}

}  // namespace polymath::sygnal
