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

#include <string>
#include <utility>
#include <vector>

namespace polymath::sygnal
{

SygnalInterfaceSocketcan::SygnalInterfaceSocketcan(std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter)
: socketcan_adapter_(std::move(socketcan_adapter))
, mcm_interface_0_(0)
, mcm_interface_1_(1)
, control_interface_()
{}

void SygnalInterfaceSocketcan::parse(const socketcan::CanFrame & frame)
{
  // Try parsing as MCM heartbeat (both interfaces will check subsystem_id)
  mcm_interface_0_.parseMcmHeartbeatFrame(frame);
  mcm_interface_1_.parseMcmHeartbeatFrame(frame);

  // Try parsing as command response
  auto response = control_interface_.parseCommandResponseFrame(frame);
  if (!response.has_value()) {
    return;
  }

  // Fulfill the appropriate promise based on response type
  std::lock_guard<std::mutex> lock(promises_mutex_);

  switch (response->response_type) {
    case SygnalControlCommandResponseType::ENABLE: {
      if (!enable_response_promises_.empty()) {
        auto promise = std::move(enable_response_promises_.front());
        enable_response_promises_.pop();
        promise.set_value(*response);
      }
      break;
    }
    case SygnalControlCommandResponseType::CONTROL: {
      if (!control_response_promises_.empty()) {
        auto promise = std::move(control_response_promises_.front());
        control_response_promises_.pop();
        promise.set_value(*response);
      }
      break;
    }
    case SygnalControlCommandResponseType::RELAY: {
      if (!relay_response_promises_.empty()) {
        auto promise = std::move(relay_response_promises_.front());
        relay_response_promises_.pop();
        promise.set_value(*response);
      }
      break;
    }
  }
}

std::array<SygnalSystemState, 5> SygnalInterfaceSocketcan::get_interface_states_0() const
{
  return mcm_interface_0_.get_interface_states();
}

std::array<SygnalSystemState, 5> SygnalInterfaceSocketcan::get_interface_states_1() const
{
  return mcm_interface_1_.get_interface_states();
}

SygnalSystemState SygnalInterfaceSocketcan::get_sygnal_mcm0_state() const
{
  return mcm_interface_0_.get_mcm_state();
}

SygnalSystemState SygnalInterfaceSocketcan::get_sygnal_mcm1_state() const
{
  return mcm_interface_1_.get_mcm_state();
}

SendCommandResult SygnalInterfaceSocketcan::sendControlStateCommand(
  uint8_t bus_id,
  uint8_t interface_id,
  SygnalControlState control_state,
  bool expect_reply,
  std::string & error_message)
{
  auto frame_opt =
    control_interface_.createControlStateCommandFrame(bus_id, interface_id, control_state, error_message);

  if (!frame_opt.has_value()) {
    return {false, std::nullopt};
  }

  std::optional<std::future<SygnalControlCommandResponse>> future_opt;
  if (expect_reply) {
    std::promise<SygnalControlCommandResponse> promise;
    future_opt = promise.get_future();
    std::lock_guard<std::mutex> lock(promises_mutex_);
    enable_response_promises_.push(std::move(promise));
  }

  auto err = socketcan_adapter_->send(*frame_opt);
  if (err.has_value()) {
    error_message += "Failed to send control state command: " + err.value() + "\n";
    return {false, std::nullopt};
  }

  return {true, std::move(future_opt)};
}

SendCommandResult SygnalInterfaceSocketcan::sendControlCommand(
  uint8_t bus_id, uint8_t interface_id, double value, bool expect_reply, std::string & error_message)
{
  auto frame_opt = control_interface_.createControlCommandFrame(bus_id, interface_id, value, error_message);

  if (!frame_opt.has_value()) {
    return {false, std::nullopt};
  }

  std::optional<std::future<SygnalControlCommandResponse>> future_opt;
  if (expect_reply) {
    std::promise<SygnalControlCommandResponse> promise;
    future_opt = promise.get_future();
    std::lock_guard<std::mutex> lock(promises_mutex_);
    control_response_promises_.push(std::move(promise));
  }

  auto err = socketcan_adapter_->send(*frame_opt);
  if (err.has_value()) {
    error_message += "Failed to send control command: " + err.value() + "\n";
    return {false, std::nullopt};
  }

  return {true, std::move(future_opt)};
}

SendCommandResult SygnalInterfaceSocketcan::sendRelayCommand(
  uint8_t bus_id, uint8_t subsystem_id, bool relay_state, bool expect_reply, std::string & error_message)
{
  auto frame_opt = control_interface_.createRelayCommandFrame(bus_id, subsystem_id, relay_state, error_message);

  if (!frame_opt.has_value()) {
    return {false, std::nullopt};
  }

  std::optional<std::future<SygnalControlCommandResponse>> future_opt;
  if (expect_reply) {
    std::promise<SygnalControlCommandResponse> promise;
    future_opt = promise.get_future();
    std::lock_guard<std::mutex> lock(promises_mutex_);
    relay_response_promises_.push(std::move(promise));
  }

  auto err = socketcan_adapter_->send(*frame_opt);
  if (err.has_value()) {
    error_message += "Failed to send relay command: " + err.value() + "\n";
    return {false, std::nullopt};
  }

  return {true, std::move(future_opt)};
}

}  // namespace polymath::sygnal
