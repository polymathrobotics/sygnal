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

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace polymath::sygnal
{

SygnalInterfaceSocketcan::SygnalInterfaceSocketcan(
  std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter,
  const std::vector<McmId> & mcm_ids,
  const std::vector<HpoId> & hpo_ids)
: socketcan_adapter_(socketcan_adapter)
, control_interface_()
{
  // Sygnal overloads CAN IDs across MCM/HPO/IO devices: a single frame ID (e.g. 0x161 ControlCommandResponse)
  // carries different byte layouts depending on which device type produced it. The only field with stable
  // semantics across layouts is the 7-bit BusAddress. Per-board parsers filter by bus_address to claim only
  // their own frames; the MCM control-response parser (SygnalControlInterface) does not filter, so it would
  // misinterpret an HPO response if it ever saw one. We sidestep this by (a) running HPO parsers first in
  // parse() so HPO frames never reach the MCM parser, and (b) requiring disjoint bus addresses so the
  // ordering can correctly route by elimination. The second invariant is enforced here.
  for (const auto & hpo : hpo_ids) {
    for (const auto & mcm : mcm_ids) {
      if (hpo.bus_id == mcm.bus_id) {
        throw std::invalid_argument(
          "Bus address " + std::to_string(hpo.bus_id) +
          " is assigned to both an MCM and an HPO; bus addresses must be disjoint across device types "
          "until Sygnal CAN-ID overlap is resolved structurally.");
      }
    }
  }

  mcms_.reserve(mcm_ids.size());
  for (const auto & id : mcm_ids) {
    mcms_.emplace_back(id.bus_id, id.subsystem_id);
  }

  hpos_.reserve(hpo_ids.size());
  for (const auto & id : hpo_ids) {
    hpos_.emplace_back(id.bus_id);
  }
}

bool SygnalInterfaceSocketcan::parse(const socketcan::CanFrame & frame)
{
  // DO NOT REORDER without re-reading the rationale in the constructor comment:
  // HPO parsers MUST run before SygnalControlInterface::parseCommandResponseFrame because the latter does
  // not filter by bus_address, and HPO control responses share CAN IDs (0x61 / 0x161) with MCM responses.
  // Heartbeats and HPO error frames already filter by bus_address inside each per-board parser, so the
  // order between MCM heartbeats and HPO heartbeats is arbitrary, but we keep the HPO block contiguous
  // for clarity.

  // --- HPO first ---
  for (auto & hpo : hpos_) {
    if (hpo.parseHeartbeatFrame(frame)) {
      return true;
    }
  }

  for (auto & hpo : hpos_) {
    auto hpo_response = hpo.parseControlResponse(frame);
    if (!hpo_response.has_value()) {
      continue;
    }
    std::lock_guard<std::mutex> lock(promises_mutex_);
    auto & queue = hpo_response->is_enable_response ? hpo_enable_response_promises_ : hpo_command_response_promises_;
    if (!queue.empty()) {
      auto promise = std::move(queue.front());
      queue.pop();
      promise.set_value(*hpo_response);
    }
    return true;
  }

  for (auto & hpo : hpos_) {
    // Error frames are parsed (and CRC-checked) but not yet surfaced to callers; see design doc open
    // question #2. Claim the frame so the MCM error path (if/when added) doesn't double-handle it.
    if (hpo.parseErrorFrame(frame).has_value()) {
      return true;
    }
  }

  // --- MCM ---
  for (auto & mcm : mcms_) {
    if (mcm.parseMcmHeartbeatFrame(frame)) {
      return true;
    }
  }

  // Try parsing as a (legacy unified) MCM command response. Safe to run last: every HPO-addressed frame
  // has already been claimed above, so anything reaching this line is either an MCM frame or unrelated.
  auto response = control_interface_.parseCommandResponseFrame(frame);
  if (!response.has_value()) {
    return false;
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

  return true;
}

std::optional<std::array<SygnalSystemState, 5>> SygnalInterfaceSocketcan::get_interface_state(
  const uint8_t bus_address, const uint8_t subsystem_id) const
{
  auto it = std::find_if(mcms_.begin(), mcms_.end(), [bus_address, subsystem_id](const SygnalMcmInterface & m) {
    return m.get_bus_address() == bus_address && m.get_subsystem_id() == subsystem_id;
  });
  if (it != mcms_.end()) {
    return it->get_interface_states();
  }
  return std::nullopt;
}

std::optional<SygnalSystemState> SygnalInterfaceSocketcan::get_sygnal_mcm_state(
  const uint8_t bus_address, const uint8_t subsystem_id) const
{
  auto it = std::find_if(mcms_.begin(), mcms_.end(), [bus_address, subsystem_id](const SygnalMcmInterface & m) {
    return m.get_bus_address() == bus_address && m.get_subsystem_id() == subsystem_id;
  });
  if (it != mcms_.end()) {
    return it->get_mcm_state();
  }
  return std::nullopt;
}

SendCommandResult SygnalInterfaceSocketcan::sendControlStateCommand(
  uint8_t bus_id,
  uint8_t interface_id,
  uint8_t subsystem_id,
  SygnalControlState control_state,
  bool expect_reply,
  std::string & error_message)
{
  auto frame_opt =
    control_interface_.createControlStateCommandFrame(bus_id, interface_id, subsystem_id, control_state, error_message);

  if (!frame_opt.has_value()) {
    return {false, std::nullopt};
  }

  std::optional<std::future<SygnalControlCommandResponse>> future_opt;
  if (expect_reply) {
    std::promise<SygnalControlCommandResponse> promise;
    future_opt = promise.get_future();
    std::lock_guard<std::mutex> lock(promises_mutex_);
    if (enable_response_promises_.size() >= MAX_PROMISE_QUEUE_LENGTH) {
      // Really old promises are dropped silently for now. We should warn in the future
      enable_response_promises_.pop();
    }
    enable_response_promises_.push(std::move(promise));
  }

  auto err = socketcan_adapter_->send(*frame_opt);
  if (err.has_value()) {
    error_message += "Failed to send control state command: " + err.value() + "\n";
    return {false, std::nullopt};
  }

  return {true, std::move(future_opt)};
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
  // TODO(ryan): update function def to include min max values and check the value before sending.
  auto frame_opt =
    control_interface_.createControlCommandFrame(bus_id, interface_id, subsystem_id, value, error_message);

  if (!frame_opt.has_value()) {
    return {false, std::nullopt};
  }

  std::optional<std::future<SygnalControlCommandResponse>> future_opt;
  if (expect_reply) {
    std::promise<SygnalControlCommandResponse> promise;
    future_opt = promise.get_future();
    std::lock_guard<std::mutex> lock(promises_mutex_);
    if (control_response_promises_.size() >= MAX_PROMISE_QUEUE_LENGTH) {
      // Really old promises are dropped silently for now. We should warn in the future
      control_response_promises_.pop();
    }
    control_response_promises_.push(std::move(promise));
  }

  auto err = socketcan_adapter_->send(*frame_opt);
  if (err.has_value()) {
    error_message += "Failed to send control command: " + err.value() + "\n";
    return {false, std::nullopt};
  }

  return {true, std::move(future_opt)};
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
  auto frame_opt = control_interface_.createRelayCommandFrame(bus_id, subsystem_id, relay_state, error_message);

  if (!frame_opt.has_value()) {
    return {false, std::nullopt};
  }

  std::optional<std::future<SygnalControlCommandResponse>> future_opt;
  if (expect_reply) {
    std::promise<SygnalControlCommandResponse> promise;
    future_opt = promise.get_future();
    std::lock_guard<std::mutex> lock(promises_mutex_);
    if (relay_response_promises_.size() >= MAX_PROMISE_QUEUE_LENGTH) {
      // Really old promises are dropped silently for now. We should warn in the future
      relay_response_promises_.pop();
    }
    relay_response_promises_.push(std::move(promise));
  }

  auto err = socketcan_adapter_->send(*frame_opt);
  if (err.has_value()) {
    error_message += "Failed to send relay command: " + err.value() + "\n";
    return {false, std::nullopt};
  }

  return {true, std::move(future_opt)};
}

SendCommandResult SygnalInterfaceSocketcan::sendRelayCommand(
  RelayEndpoint relay, bool relay_state, bool expect_reply, std::string & error_message)
{
  return sendRelayCommand(relay.bus_id, relay.subsystem_id, relay_state, expect_reply, error_message);
}

SendHpoCommandResult SygnalInterfaceSocketcan::sendHpoControlEnable(
  uint8_t bus_id, uint8_t message_id, bool enable, bool expect_reply, std::string & error_message)
{
  auto it = std::find_if(
    hpos_.begin(), hpos_.end(), [bus_id](const SygnalHpoInterface & h) { return h.get_bus_address() == bus_id; });
  if (it == hpos_.end()) {
    error_message += "No HPO registered at bus address " + std::to_string(bus_id) + "\n";
    return {false, std::nullopt};
  }

  auto frame_opt = it->createControlEnableFrame(message_id, enable, error_message);
  if (!frame_opt.has_value()) {
    return {false, std::nullopt};
  }

  std::optional<std::future<HpoControlResponse>> future_opt;
  if (expect_reply) {
    std::promise<HpoControlResponse> promise;
    future_opt = promise.get_future();
    std::lock_guard<std::mutex> lock(promises_mutex_);
    if (hpo_enable_response_promises_.size() >= MAX_PROMISE_QUEUE_LENGTH) {
      hpo_enable_response_promises_.pop();
    }
    hpo_enable_response_promises_.push(std::move(promise));
  }

  auto err = socketcan_adapter_->send(*frame_opt);
  if (err.has_value()) {
    error_message += "Failed to send HPO control enable: " + err.value() + "\n";
    // The promise was already pushed before the send attempt; surface the future regardless so the caller
    // can choose to wait on it (or discard) instead of leaving a dangling future the harness can't observe.
    return {false, std::move(future_opt)};
  }

  return {true, std::move(future_opt)};
}

SendHpoCommandResult SygnalInterfaceSocketcan::sendHpoControlCommand(
  uint8_t bus_id, uint8_t message_id, double value, bool expect_reply, std::string & error_message)
{
  auto it = std::find_if(
    hpos_.begin(), hpos_.end(), [bus_id](const SygnalHpoInterface & h) { return h.get_bus_address() == bus_id; });
  if (it == hpos_.end()) {
    error_message += "No HPO registered at bus address " + std::to_string(bus_id) + "\n";
    return {false, std::nullopt};
  }

  auto frame_opt = it->createControlCommandFrame(message_id, value, error_message);
  if (!frame_opt.has_value()) {
    return {false, std::nullopt};
  }

  std::optional<std::future<HpoControlResponse>> future_opt;
  if (expect_reply) {
    std::promise<HpoControlResponse> promise;
    future_opt = promise.get_future();
    std::lock_guard<std::mutex> lock(promises_mutex_);
    if (hpo_command_response_promises_.size() >= MAX_PROMISE_QUEUE_LENGTH) {
      hpo_command_response_promises_.pop();
    }
    hpo_command_response_promises_.push(std::move(promise));
  }

  auto err = socketcan_adapter_->send(*frame_opt);
  if (err.has_value()) {
    error_message += "Failed to send HPO control command: " + err.value() + "\n";
    return {false, std::move(future_opt)};
  }

  return {true, std::move(future_opt)};
}

std::optional<std::array<bool, HPO_NUM_INTERFACES>> SygnalInterfaceSocketcan::get_hpo_interface_states(
  uint8_t bus_address) const
{
  auto it = std::find_if(hpos_.begin(), hpos_.end(), [bus_address](const SygnalHpoInterface & h) {
    return h.get_bus_address() == bus_address;
  });
  if (it == hpos_.end()) {
    return std::nullopt;
  }
  return it->get_interface_states();
}

}  // namespace polymath::sygnal
