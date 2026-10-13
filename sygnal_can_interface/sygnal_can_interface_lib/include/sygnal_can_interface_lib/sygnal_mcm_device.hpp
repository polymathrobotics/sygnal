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

#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "sygnal_can_interface_lib/sygnal_device.hpp"

namespace polymath::sygnal
{

/// @brief Virtual interfaces an MCM subsystem reports in its heartbeat.
constexpr uint8_t MCM_NUM_INTERFACES = 7;

/// @brief MCM fault cause, as reported in FaultState, FaultIncrement, FaultList and FaultRootCause.
enum class McmFaultCause : uint8_t
{
  NONE = 0x00,
  EMERGENCY_STOP = 0x01,
  EXTERNAL_FAULT = 0x02,
  CAN_FAIL_OPERATIONAL_1 = 0x03,
  CAN_FAIL_OPERATIONAL_2 = 0x04,
  CAN_FAIL_HARD = 0x05,
  INVALID_SUBSYSTEM_ID = 0x10,
  INVALID_CONFIGURATION_CRC = 0x11,
  PHYSICAL_INTERFACE_TRANSITION = 0x1B,
  INVALID_INPUT_SIGNAL = 0x20,
  INVALID_OUTPUT_SIGNAL = 0x21,
  HEARTBEAT_TIMEOUT = 0x30,
  HEARTBEAT_COUNT = 0x31,
  CONTROL_TIMEOUT = 0x32,
  CONTROL_COUNT = 0x33,
  CONTROL_OUT_OF_BOUNDS = 0x34,
  CLOSED_LOOP_TIMEOUT = 0x35,
  CLOSED_LOOP_COUNT = 0x36,
  CLOSED_LOOP_OUT_OF_BOUNDS = 0x37,
  INVALID_CONFIGURATION_OPTION = 0x38,
  SW_INDEX_OUT_OF_BOUNDS = 0x40,
  SW_INVALID_STATE = 0x41,
  SW_COPROCESSOR_OVERFLOW = 0x42,
};

/// @brief Fault status of one MCM subsystem since its faults were last cleared.
struct McmFaultStatus
{
  SygnalSystemState system_state{SygnalSystemState::FAIL_HARD};
  /// Cause of the current system_state; NONE when not faulted.
  McmFaultCause state_cause{McmFaultCause::NONE};
  /// Cause that tripped each fault level; NONE if that level was never entered.
  McmFaultCause fail_op1_cause{McmFaultCause::NONE};
  McmFaultCause fail_op2_cause{McmFaultCause::NONE};
  McmFaultCause fail_hard_cause{McmFaultCause::NONE};
  /// Occurrences of each cause.
  std::map<McmFaultCause, uint16_t> counts;
};

/// @brief One MCM subsystem: MCM_IF endpoints (interface ID 0-6) and one MCM_RELAY endpoint.
///        Fault status resets when a heartbeat moves the subsystem from a fail state to human or MCM control.
class SygnalMcmDevice : public SygnalDevice
{
public:
  /// @param socketcan_adapter Adapter that commands go out on.
  SygnalMcmDevice(std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter, uint8_t bus_id, uint8_t subsystem_id);

  SygnalSystemState get_endpoint_state(const SygnalEndpoint & endpoint) const override;
  double get_endpoint_value(const SygnalEndpoint & endpoint) const override;
  EndpointStateCommandResult send_endpoint_state_command(
    const SygnalEndpoint & endpoint, bool enable, bool expect_reply) override;
  EndpointValueCommandResult send_endpoint_value_command(
    const SygnalEndpoint & endpoint, double value, bool expect_reply) override;

  McmFaultStatus get_faults() const;

private:
  uint8_t bus_id_;
  uint8_t subsystem_id_;

  std::array<bool, MCM_NUM_INTERFACES> interface_enabled_{};
  std::array<double, MCM_NUM_INTERFACES> interface_values_{};
  double relay_value_{0.0};
  McmFaultStatus faults_;
};

}  // namespace polymath::sygnal
