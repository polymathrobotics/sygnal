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

namespace
{

bool is_interface(const SygnalEndpoint & endpoint)
{
  return SygnalControlEndpointType::HPO_IF == endpoint.type;
}

std::invalid_argument unknown_endpoint(const SygnalEndpoint & endpoint)
{
  return std::invalid_argument(
    "HPO has no endpoint of type " + std::to_string(static_cast<int>(endpoint.type)) + " with id " +
    std::to_string(endpoint.endpoint_id));
}

}  // namespace

SygnalHpoDevice::SygnalHpoDevice(std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter, uint8_t bus_id)
: SygnalDevice(std::move(socketcan_adapter))
, bus_id_(bus_id)
{
  // Set initial states for the HPO interfaces.
  interface_states_.fill(SygnalSystemState::FAIL_HARD);
  values_.fill(0.0);

  // Handle Heartbeat Message
  set_handler<hpo_heartbeat::Heartbeat>([this](const hpo_heartbeat::Heartbeat & msg) {
    // Update the system and interface states based on the heartbeat message.
    std::lock_guard<std::mutex> lock(mutex_);
    system_state_ = static_cast<SygnalSystemState>(msg.system_state);
    interface_states_ = {
      static_cast<SygnalSystemState>(msg.interface0_state),
      static_cast<SygnalSystemState>(msg.interface1_state),
      static_cast<SygnalSystemState>(msg.interface2_state),
      static_cast<SygnalSystemState>(msg.interface3_state)};
  });

  // Handle ControlEnableResponse Message
  set_handler<hpo_control::ControlEnableResponse>([this](const hpo_control::ControlEnableResponse & msg) {
    // Update the interface state based on the control enable response message.
    const uint8_t id = static_cast<uint8_t>(msg.message_id);
    const SygnalSystemState state = static_cast<SygnalSystemState>(msg.enable);
    if (HPO_NUM_INTERFACES > id) {
      std::lock_guard<std::mutex> lock(mutex_);
      interface_states_[id] = state;
    }

    // Fulfill the promise for the endpoint state response, if one was expected.
    fulfill_endpoint_state_response(SygnalControlEndpointType::HPO_IF, id, state);
  });

  // Handle ControlCommandResponse Message
  set_handler<hpo_control::ControlCommandResponse>([this](const hpo_control::ControlCommandResponse & msg) {
    // Update the value of the interface based on the control command response message.
    const uint8_t id = static_cast<uint8_t>(msg.message_id);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      values_[id] = msg.value;
    }

    // Fulfill the promise for the endpoint value response, if one was expected.
    fulfill_endpoint_value_response(SygnalControlEndpointType::HPO_IF, id, msg.value);
  });
}

SygnalSystemState SygnalHpoDevice::get_endpoint_state(const SygnalEndpoint & endpoint) const
{
  if (!is_interface(endpoint) || HPO_NUM_INTERFACES <= endpoint.endpoint_id) {
    throw unknown_endpoint(endpoint);
  }
  std::lock_guard<std::mutex> lock(mutex_);
  return interface_states_[endpoint.endpoint_id];
}

double SygnalHpoDevice::get_endpoint_value(const SygnalEndpoint & endpoint) const
{
  if (!is_interface(endpoint)) {
    throw unknown_endpoint(endpoint);
  }
  std::lock_guard<std::mutex> lock(mutex_);
  return values_[endpoint.endpoint_id];
}

EndpointStateCommandResult SygnalHpoDevice::send_endpoint_state_command(
  const SygnalEndpoint & endpoint, bool enable, bool expect_reply)
{
  // Fail early if the endpoint is not the correct type (HPO_IF).
  if (!is_interface(endpoint)) {
    return {false, "HPO commands take an HPO_IF endpoint.", std::nullopt};
  }

  // Build the message to send over the CAN bus.
  hpo_control::ControlEnable msg;
  msg.bus_address = bus_id_;
  msg.message_id = endpoint.endpoint_id;
  msg.enable = enable ? 1 : 0;
  msg.crc = 0;
  const socketcan::CanFrame frame = to_frame(msg);

  std::optional<std::future<SygnalSystemState>> future;
  if (expect_reply) {
    future = expect_endpoint_state_response(endpoint);
  }
  if (const auto send_error = socketcan_adapter_->send(frame)) {
    if (future) {
      cancel_endpoint_state_response(endpoint);
    }
    return {false, *send_error, std::nullopt};
  }
  return {true, "", std::move(future)};
}

EndpointValueCommandResult SygnalHpoDevice::send_endpoint_value_command(
  const SygnalEndpoint & endpoint, double value, bool expect_reply)
{
  if (!is_interface(endpoint)) {
    return {false, "HPO commands take an HPO_IF endpoint.", std::nullopt};
  }
  hpo_control::ControlCommand msg;
  msg.bus_address = bus_id_;
  msg.message_id = endpoint.endpoint_id;
  /// TODO: (Ryan) Increment Count8 per command once the HPO's count check is enabled.
  msg.count8 = 0;
  msg.value = value;
  msg.crc = 0;
  const socketcan::CanFrame frame = to_frame(msg);

  std::optional<std::future<double>> future;
  if (expect_reply) {
    future = expect_endpoint_value_response(endpoint);
  }
  if (const auto send_error = socketcan_adapter_->send(frame)) {
    if (future) {
      cancel_endpoint_value_response(endpoint);
    }
    return {false, *send_error, std::nullopt};
  }
  return {true, "", std::move(future)};
}

}  // namespace polymath::sygnal
