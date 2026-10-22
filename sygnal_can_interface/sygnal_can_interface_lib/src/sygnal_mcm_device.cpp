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

#include <algorithm>
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

McmFaultCause to_cause(double wire_value)
{
  return static_cast<McmFaultCause>(static_cast<uint8_t>(wire_value));
}

}  // namespace

SygnalMcmDevice::SygnalMcmDevice(
  std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter,
  const SygnalControlDevice & device,
  std::chrono::steady_clock::duration command_timeout)
: SygnalDevice(std::move(socketcan_adapter), device, command_timeout)
, relay_responses_(command_timeout_)
{
  initializeCANHandlerCallbacks();
}

void SygnalMcmDevice::initializeCANHandlerCallbacks()
{
  // Handle MCM Heartbeat messages.
  can_handler_.set_handler<mcm_heartbeat::Heartbeat>([this](const mcm_heartbeat::Heartbeat & msg) {
    // Grab the system state from the heartbeat message.
    const SygnalSystemState state = static_cast<SygnalSystemState>(msg.system_state);


    std::lock_guard<std::mutex> lock(state_mutex_);

    // Determine if faults were cleared since the last heartbeat before updating the state.
    const bool was_failed = SygnalSystemState::FAIL_OPERATIONAL_1 == system_state_.value ||
                            SygnalSystemState::FAIL_OPERATIONAL_2 == system_state_.value ||
                            SygnalSystemState::HUMAN_OVERRIDE == system_state_.value ||
                            SygnalSystemState::FAIL_HARD == system_state_.value;
    const bool cleared =
      was_failed && (SygnalSystemState::HUMAN_CONTROL == state || SygnalSystemState::MCM_CONTROL == state);


    // Per the Sygnal docs, fault message only send when there is an active fault.
    // So if the MCM is in a clear state, reset the cached faults because we don't expect
    // any more fault messages until a new fault occurs.
    if (cleared) {
      faults_ = {McmFaultStatus{}, msg.bus_time};
    }

    // Update the system state with the new heartbeat information.
    system_state_ = {state, msg.bus_time};
    const std::array<bool, MCM_NUM_INTERFACES> enabled{
      0 != msg.interface0_state,
      0 != msg.interface1_state,
      0 != msg.interface2_state,
      0 != msg.interface3_state,
      0 != msg.interface4_state,
      0 != msg.interface5_state,
      0 != msg.interface6_state};
    for (size_t i = 0; i < MCM_NUM_INTERFACES; ++i) {
      interface_enabled_[i] = {enabled[i], msg.bus_time};
    }

    /// TODO: (Ryan) Handle msg.overall_interface_state (1 MCM Control, 0 Human Control): expose it through a getter,
    /// or drop it if system_state always agrees with it.
  });

  // Handle MCM Control Enable Response messages.
  can_handler_.set_handler<mcm_control::ControlEnableResponse>([this](const mcm_control::ControlEnableResponse & msg) {
    // Parse the interface ID and enable state from the message.
    const uint8_t id = static_cast<uint8_t>(msg.interface_id);
    const bool enabled = 0 != msg.enable;

    std::lock_guard<std::mutex> lock(state_mutex_);

    // Update the interface enabled state if the ID is valid.
    if (MCM_NUM_INTERFACES > id) {
      interface_enabled_[id] = {enabled, msg.bus_time};
    }

    // Fulfill the pending promise for this interface ID with the enable state.
    control_enable_responses_.fulfill(id, {enabled ? SygnalSystemState::MCM_CONTROL : SygnalSystemState::HUMAN_CONTROL, msg.bus_time});
  });

  // Handle MCM Control Command Response messages.
  can_handler_.set_handler<mcm_control::ControlCommandResponse>([this](const mcm_control::ControlCommandResponse & msg) {
    // Parse the interface ID from the message.
    const uint8_t id = static_cast<uint8_t>(msg.interface_id);

    std::lock_guard<std::mutex> lock(state_mutex_);

    // Update the interface value if the ID is valid.
    if (MCM_NUM_INTERFACES > id) {
      interface_values_[id] = {msg.value, msg.bus_time};
    }

    // Fulfill the pending promise for this interface ID with the command value.
    control_command_responses_.fulfill(id, {msg.value, msg.bus_time});
  });

  // Handle MCM Relay Command Response messages.
  can_handler_.set_handler<mcm_relay::RelayCommandResponse>([this](const mcm_relay::RelayCommandResponse & msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // Update the relay value with the message data.
    relay_value_ = {msg.enable, msg.bus_time};
    // Fulfill the pending promise for the relay command.
    relay_responses_.fulfill(0, relay_value_);
  });

  // Handle MCM Fault State messages.
  can_handler_.set_handler<mcm_fault::FaultState>([this](const mcm_fault::FaultState & msg) {
    // Update the system state and fault information with the message data.
    std::lock_guard<std::mutex> lock(state_mutex_);
    system_state_ = {static_cast<SygnalSystemState>(msg.fault_state), msg.bus_time};
    faults_.value.state_cause = to_cause(msg.fault_cause);
    faults_.timestamp = msg.bus_time;
  });

  // Handle MCM Fault Increment messages.
  can_handler_.set_handler<mcm_fault::FaultIncrement>([this](const mcm_fault::FaultIncrement & msg) {
    // Update the fault count for the specified fault type.
    std::lock_guard<std::mutex> lock(state_mutex_);
    faults_.value.counts[to_cause(msg.fault_type)] = static_cast<uint16_t>(msg.fault_count);
    faults_.timestamp = msg.bus_time;
  });

  // Handle MCM Fault List messages.
  can_handler_.set_handler<mcm_fault::FaultList>([this](const mcm_fault::FaultList & msg) {
    // Update the fault count for the specified fault type from the fault list message.
    std::lock_guard<std::mutex> lock(state_mutex_);
    faults_.value.counts[to_cause(msg.fault_type)] = static_cast<uint16_t>(msg.fault_count);
    faults_.timestamp = msg.bus_time;
  });

  // Handle MCM Fault Root Cause messages.
  can_handler_.set_handler<mcm_fault::FaultRootCause>([this](const mcm_fault::FaultRootCause & msg) {
    // Update the root cause information for the specified fault types.
    std::lock_guard<std::mutex> lock(state_mutex_);
    faults_.value.fail_op1_cause = to_cause(msg.fail_op1_cause);
    faults_.value.fail_op2_cause = to_cause(msg.fail_op2_cause);
    faults_.value.fail_hard_cause = to_cause(msg.fail_hard_cause);
    faults_.timestamp = msg.bus_time;
  });
}

Stamped<SygnalSystemState> SygnalMcmDevice::get_interface_control_state(const SygnalEndpoint & endpoint) const
{
  // Check if the endpoint type is MCM_INTERFACE; otherwise, delegate to the base class for error handling.
  if (SygnalControlEndpointType::MCM_INTERFACE != endpoint.type) {
    return SygnalDevice::get_interface_control_state(endpoint);
  }

  // throw if the interface ID is out of range
  if (MCM_NUM_INTERFACES <= endpoint.endpoint_id) {
    throw std::invalid_argument(
      "Sygnal Endpoint " + std::string(endpoint.name) + " has interface ID " + std::to_string(endpoint.endpoint_id) +
      "; MCM interface IDs are 0-6.");
  }

  // Return the interface control state for the specified interface ID.
  std::lock_guard<std::mutex> lock(state_mutex_);
  const Stamped<bool> & enabled = interface_enabled_[endpoint.endpoint_id];
  return {enabled.value ? SygnalSystemState::MCM_CONTROL : SygnalSystemState::HUMAN_CONTROL, enabled.timestamp};
}

Stamped<double> SygnalMcmDevice::get_control_command_value(const SygnalEndpoint & endpoint) const
{
  if (SygnalControlEndpointType::MCM_INTERFACE == endpoint.type) {
    // Throw if the interface ID is out of range.
    if (MCM_NUM_INTERFACES <= endpoint.endpoint_id) {
      throw std::invalid_argument(
        "Sygnal Endpoint " + std::string(endpoint.name) + " has interface ID " + std::to_string(endpoint.endpoint_id) +
        "; MCM interface IDs are 0-6.");
    }

    // Return the control command value for the specified interface ID.
    std::lock_guard<std::mutex> lock(state_mutex_);
    return interface_values_[endpoint.endpoint_id];
  }

  // Just return the single relay value for MCM_RELAY endpoints.
  if (SygnalControlEndpointType::MCM_RELAY == endpoint.type) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return relay_value_;
  }

  // If neither MCM_RELAY nor MCM_INTERFACE, delegate to the base class for error handling.
  return SygnalDevice::get_control_command_value(endpoint);
}

ControlEnableCommandResult SygnalMcmDevice::send_control_enable_command(
  const SygnalEndpoint & endpoint, bool enable, bool expect_reply)
{
  // If the endpoint is not an MCM_INTERFACE, delegate to the base class for error handling.
  if (SygnalControlEndpointType::MCM_INTERFACE != endpoint.type) {
    return SygnalDevice::send_control_enable_command(endpoint, enable, expect_reply);
  }

  // Throw if the interface ID is out of range.
  if (MCM_NUM_INTERFACES <= endpoint.endpoint_id) {
    throw std::invalid_argument(
      "Sygnal Endpoint " + std::string(endpoint.name) + " has interface ID " + std::to_string(endpoint.endpoint_id) +
      "; MCM interface IDs are 0-6.");
  }

  // Construct the control enable message.
  mcm_control::ControlEnable msg;
  msg.bus_address = device().bus_id;
  msg.sub_system_id = device().subsystem_id;
  msg.interface_id = endpoint.endpoint_id;
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

ControlCommandResult SygnalMcmDevice::send_control_command(
  const SygnalEndpoint & endpoint, double value, bool expect_reply)
{
  // Delegate to the appropriate command handler based on the endpoint type (MCM_INTERFACE or MCM_RELAY)
  switch (endpoint.type) {
    case SygnalControlEndpointType::MCM_INTERFACE:
      return send_interface_command(endpoint, value, expect_reply);
    case SygnalControlEndpointType::MCM_RELAY:
      return send_relay_command(value, expect_reply);
    default:
      return SygnalDevice::send_control_command(endpoint, value, expect_reply);
  }
}

ControlCommandResult SygnalMcmDevice::send_interface_command(
  const SygnalEndpoint & endpoint, double value, bool expect_reply)
{
  // If type is not MCM_INTERFACE, delegate to the base class for error handling.
  if (endpoint.type != SygnalControlEndpointType::MCM_INTERFACE) {
    return SygnalDevice::send_control_command(endpoint, value, expect_reply);
  }

  // Throw if the interface ID is out of range.
  if (MCM_NUM_INTERFACES <= endpoint.endpoint_id) {
    throw std::invalid_argument(
      "Sygnal Endpoint " + std::string(endpoint.name) + " has interface ID " + std::to_string(endpoint.endpoint_id) +
      "; MCM interface IDs are 0-6.");
  }

  // Construct the control command message.
  mcm_control::ControlCommand msg;
  msg.bus_address = device().bus_id;
  msg.sub_system_id = device().subsystem_id;
  msg.interface_id = endpoint.endpoint_id;
  /// TODO: (Ryan) Increment Count8 per command once the MCM's count check is enabled.
  msg.count8 = 0;
  msg.value = value;
  const socketcan::CanFrame frame = to_frame_with_crc(msg);

  // Push a future for the expected reply if needed.
  std::optional<std::future<Stamped<double>>> future;
  if (expect_reply) {
    future = control_command_responses_.push(endpoint.endpoint_id);
  }

  // Send the control command message
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

ControlCommandResult SygnalMcmDevice::send_relay_command(double value, bool expect_reply)
{
  // Construct the relay command message.
  mcm_relay::RelayCommand msg;
  msg.bus_address = device().bus_id;
  msg.subsystem_id = device().subsystem_id;
  msg.enable = 0.0 != value ? 1 : 0;
  const socketcan::CanFrame frame = to_frame_with_crc(msg);

  // Push a future for the expected reply if needed.
  // RelayCommandResponse carries no ID, so every relay response fulfills key 0.
  std::optional<std::future<Stamped<double>>> future;
  if (expect_reply) {
    future = relay_responses_.push(0);
  }

  // send() returns an error message when the send fails, and std::nullopt when it succeeds.
  const std::optional<std::string> send_error = socketcan_adapter_->send(frame);
  if (send_error.has_value()) {
    if (future) {
      relay_responses_.withdraw_newest(0);
    }
    return {false, send_error.value(), std::nullopt};
  }
  return {true, "", std::move(future)};
}

Stamped<McmFaultStatus> SygnalMcmDevice::get_device_faults() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  Stamped<McmFaultStatus> faults = faults_;
  faults.value.system_state = system_state_.value;
  faults.timestamp = std::max(faults_.timestamp, system_state_.timestamp);
  return faults;
}

}  // namespace polymath::sygnal
