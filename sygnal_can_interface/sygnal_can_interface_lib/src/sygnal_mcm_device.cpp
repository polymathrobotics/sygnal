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

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include "mcm_control/mcm_control.hpp"
#include "mcm_fault/mcm_fault.hpp"
#include "mcm_heartbeat/mcm_heartbeat.hpp"
#include "mcm_relay/mcm_relay.hpp"

namespace polymath::sygnal
{

namespace
{

bool is_fail_state(SygnalSystemState state)
{
  return SygnalSystemState::FAIL_OPERATIONAL_1 == state || SygnalSystemState::FAIL_OPERATIONAL_2 == state ||
         SygnalSystemState::FAIL_HARD == state;
}

SygnalSystemState control_state(bool enabled)
{
  return enabled ? SygnalSystemState::MCM_CONTROL : SygnalSystemState::HUMAN_CONTROL;
}

McmFaultCause to_cause(double wire_value)
{
  return static_cast<McmFaultCause>(static_cast<uint8_t>(wire_value));
}

bool is_interface(const SygnalEndpoint & endpoint)
{
  return SygnalControlEndpointType::MCM_IF == endpoint.type && MCM_NUM_INTERFACES > endpoint.endpoint_id;
}

std::invalid_argument unknown_endpoint(const SygnalEndpoint & endpoint)
{
  return std::invalid_argument(
    "MCM has no endpoint of type " + std::to_string(static_cast<int>(endpoint.type)) + " with id " +
    std::to_string(endpoint.endpoint_id));
}

}  // namespace

SygnalMcmDevice::SygnalMcmDevice(
  std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter, uint8_t bus_id, uint8_t subsystem_id)
: SygnalDevice(std::move(socketcan_adapter))
, bus_id_(bus_id)
, subsystem_id_(subsystem_id)
{
  set_handler<mcm_heartbeat::Heartbeat>([this](const mcm_heartbeat::Heartbeat & msg) {
    const SygnalSystemState state = static_cast<SygnalSystemState>(msg.system_state);

    std::lock_guard<std::mutex> lock(mutex_);
    const bool cleared = is_fail_state(system_state_) &&
                         (SygnalSystemState::HUMAN_CONTROL == state || SygnalSystemState::MCM_CONTROL == state);
    system_state_ = state;
    interface_enabled_ = {
      0 != msg.interface0_state,
      0 != msg.interface1_state,
      0 != msg.interface2_state,
      0 != msg.interface3_state,
      0 != msg.interface4_state,
      0 != msg.interface5_state,
      0 != msg.interface6_state};
    if (cleared) {
      faults_ = McmFaultStatus{};
    }
  });

  set_handler<mcm_control::ControlEnableResponse>([this](const mcm_control::ControlEnableResponse & msg) {
    const uint8_t id = static_cast<uint8_t>(msg.interface_id);
    const bool enabled = 0 != msg.enable;
    if (MCM_NUM_INTERFACES > id) {
      std::lock_guard<std::mutex> lock(mutex_);
      interface_enabled_[id] = enabled;
    }
    fulfill_endpoint_state_response(SygnalControlEndpointType::MCM_IF, id, control_state(enabled));
  });

  set_handler<mcm_control::ControlCommandResponse>([this](const mcm_control::ControlCommandResponse & msg) {
    const uint8_t id = static_cast<uint8_t>(msg.interface_id);
    if (MCM_NUM_INTERFACES > id) {
      std::lock_guard<std::mutex> lock(mutex_);
      interface_values_[id] = msg.value;
    }
    fulfill_endpoint_value_response(SygnalControlEndpointType::MCM_IF, id, msg.value);
  });

  set_handler<mcm_relay::RelayCommandResponse>([this](const mcm_relay::RelayCommandResponse & msg) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      relay_value_ = msg.enable;
    }
    fulfill_endpoint_value_response(SygnalControlEndpointType::MCM_RELAY, 0, msg.enable);
  });

  set_handler<mcm_fault::FaultState>([this](const mcm_fault::FaultState & msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    system_state_ = static_cast<SygnalSystemState>(msg.fault_state);
    faults_.state_cause = to_cause(msg.fault_cause);
  });

  set_handler<mcm_fault::FaultIncrement>([this](const mcm_fault::FaultIncrement & msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    faults_.counts[to_cause(msg.fault_type)] = static_cast<uint16_t>(msg.fault_count);
  });

  set_handler<mcm_fault::FaultList>([this](const mcm_fault::FaultList & msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    faults_.counts[to_cause(msg.fault_type)] = static_cast<uint16_t>(msg.fault_count);
  });

  set_handler<mcm_fault::FaultRootCause>([this](const mcm_fault::FaultRootCause & msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    faults_.fail_op1_cause = to_cause(msg.fail_op1_cause);
    faults_.fail_op2_cause = to_cause(msg.fail_op2_cause);
    faults_.fail_hard_cause = to_cause(msg.fail_hard_cause);
  });
}

SygnalSystemState SygnalMcmDevice::get_endpoint_state(const SygnalEndpoint & endpoint) const
{
  if (!is_interface(endpoint)) {
    throw unknown_endpoint(endpoint);
  }
  std::lock_guard<std::mutex> lock(mutex_);
  return control_state(interface_enabled_[endpoint.endpoint_id]);
}

double SygnalMcmDevice::get_endpoint_value(const SygnalEndpoint & endpoint) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (SygnalControlEndpointType::MCM_RELAY == endpoint.type) {
    return relay_value_;
  }
  if (!is_interface(endpoint)) {
    throw unknown_endpoint(endpoint);
  }
  return interface_values_[endpoint.endpoint_id];
}

EndpointStateCommandResult SygnalMcmDevice::send_endpoint_state_command(
  const SygnalEndpoint & endpoint, bool enable, bool expect_reply)
{
  if (!is_interface(endpoint)) {
    return {false, "MCM enable commands take an MCM_IF endpoint with interface ID below 7.", std::nullopt};
  }
  mcm_control::ControlEnable msg;
  msg.bus_address = bus_id_;
  msg.sub_system_id = subsystem_id_;
  msg.interface_id = endpoint.endpoint_id;
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

EndpointValueCommandResult SygnalMcmDevice::send_endpoint_value_command(
  const SygnalEndpoint & endpoint, double value, bool expect_reply)
{
  socketcan::CanFrame frame;
  if (SygnalControlEndpointType::MCM_RELAY == endpoint.type) {
    mcm_relay::RelayCommand msg;
    msg.bus_address = bus_id_;
    msg.subsystem_id = subsystem_id_;
    msg.enable = 0.0 != value ? 1 : 0;
    msg.crc = 0;
    frame = to_frame(msg);
  } else if (is_interface(endpoint)) {
    mcm_control::ControlCommand msg;
    msg.bus_address = bus_id_;
    msg.sub_system_id = subsystem_id_;
    msg.interface_id = endpoint.endpoint_id;
    /// TODO: (Ryan) Increment Count8 per command once the MCM's count check is enabled.
    msg.count8 = 0;
    msg.value = value;
    msg.crc = 0;
    frame = to_frame(msg);
  } else {
    return {false, "MCM value commands take an MCM_IF endpoint with interface ID below 7, or MCM_RELAY.", std::nullopt};
  }
  // RelayCommandResponse carries no relay ID, so relay responses are fulfilled on endpoint ID 0.
  SygnalEndpoint response_endpoint = endpoint;
  if (SygnalControlEndpointType::MCM_RELAY == endpoint.type) {
    response_endpoint.endpoint_id = 0;
  }
  std::optional<std::future<double>> future;
  if (expect_reply) {
    future = expect_endpoint_value_response(response_endpoint);
  }
  if (const auto send_error = socketcan_adapter_->send(frame)) {
    if (future) {
      cancel_endpoint_value_response(response_endpoint);
    }
    return {false, *send_error, std::nullopt};
  }
  return {true, "", std::move(future)};
}

McmFaultStatus SygnalMcmDevice::get_faults() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  McmFaultStatus faults = faults_;
  faults.system_state = system_state_;
  return faults;
}

}  // namespace polymath::sygnal
