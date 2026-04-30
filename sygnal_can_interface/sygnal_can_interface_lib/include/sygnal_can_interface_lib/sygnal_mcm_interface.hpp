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

#ifndef SYGNAL_CAN_INTERFACE_LIB__SYGNAL_MCM_INTERFACE_HPP_
#define SYGNAL_CAN_INTERFACE_LIB__SYGNAL_MCM_INTERFACE_HPP_

#include <array>
#include <map>
#include <string>

#include "socketcan_adapter/can_frame.hpp"
#include "socketcan_adapter/socketcan_adapter.hpp"

namespace polymath::sygnal
{

constexpr uint8_t DEFAULT_MCM_BUS_ADDRESS = 1;
constexpr uint8_t SECONDARY_MCM_BUS_ADDRESS = 2;

/// @brief Enum representing the overall state of the system
enum class SygnalSystemState : uint8_t
{
  FAIL_HARD = 254,
  HUMAN_OVERRIDE = 253,
  FAIL_OPERATIONAL_2 = 242,
  FAIL_OPERATIONAL_1 = 241,
  MCM_CONTROL = 1,
  HUMAN_CONTROL = 0
};

/// @brief Raw fault-cause byte from FaultState/FaultIncrement/FaultList/FaultRootCause messages.
/// Specific cause codes are defined in Sygnal's "Faults & Fault Behavior" documentation;
/// the value is stored unmodified so we don't drift from upstream when new causes are added.
using SygnalFaultCause = uint8_t;
static const SygnalFaultCause NO_FAULT_CAUSE = 0x00;

/// @brief Root cause breakdown reported by the FaultRootCause (0x221) message.
struct SygnalFaultRootCause
{
  SygnalFaultCause fail_op1 = NO_FAULT_CAUSE;
  SygnalFaultCause fail_op2 = NO_FAULT_CAUSE;
  SygnalFaultCause fail_hard = NO_FAULT_CAUSE;
};

/// @brief Get the string of the Sygnal System State
/// @param state input SygnalSystemState
/// @return string of the state value
inline std::string sygnalSystemStateToString(SygnalSystemState state)
{
  switch (state) {
    case SygnalSystemState::FAIL_HARD:
      return "FAIL_HARD";
    case SygnalSystemState::HUMAN_OVERRIDE:
      return "HUMAN_OVERRIDE";
    case SygnalSystemState::FAIL_OPERATIONAL_2:
      return "FAIL_OPERATIONAL_2";
    case SygnalSystemState::FAIL_OPERATIONAL_1:
      return "FAIL_OPERATIONAL_1";
    case SygnalSystemState::MCM_CONTROL:
      return "MCM_CONTROL";
    case SygnalSystemState::HUMAN_CONTROL:
      return "HUMAN_CONTROL";
    default:
      return "UNKNOWN_STATE";
  }
}

class SygnalMcmInterface
{
public:
  SygnalMcmInterface();
  explicit SygnalMcmInterface(const uint8_t bus_address, const uint8_t subsystem_id);
  ~SygnalMcmInterface() = default;

  std::array<SygnalSystemState, 5> get_interface_states() const
  {
    return sygnal_interface_states_;
  }

  SygnalSystemState get_mcm_state() const
  {
    return sygnal_mcm_state_;
  }

  std::chrono::system_clock::time_point get_last_heartbeat_timestamp() const
  {
    return last_heartbeat_timestamp_;
  }

  /// @brief Most recent fault cause reported in a FaultState (0x20) message.
  SygnalFaultCause get_last_fault_cause() const
  {
    return last_fault_cause_;
  }

  /// @brief Most recent root cause breakdown reported in a FaultRootCause (0x221) message.
  SygnalFaultRootCause get_last_root_cause() const
  {
    return last_root_cause_;
  }

  /// @brief Per-cause fault counts as last reported in FaultIncrement (0x21) /
  /// FaultList (0x220) messages. Both messages carry the current count for the cause,
  /// so this map reflects the latest snapshot rather than an accumulated total.
  std::map<SygnalFaultCause, uint16_t> get_fault_counts() const
  {
    return fault_counts_;
  }

  std::chrono::system_clock::time_point get_last_fault_state_timestamp() const
  {
    return last_fault_state_timestamp_;
  }

  std::chrono::system_clock::time_point get_last_root_cause_timestamp() const
  {
    return last_root_cause_timestamp_;
  }

  std::chrono::system_clock::time_point get_last_fault_count_timestamp() const
  {
    return last_fault_count_timestamp_;
  }

  bool parseMcmHeartbeatFrame(const socketcan::CanFrame & frame);
  bool parseFaultStateFrame(const socketcan::CanFrame & frame);
  bool parseFaultIncrementFrame(const socketcan::CanFrame & frame);
  bool parseFaultListFrame(const socketcan::CanFrame & frame);
  bool parseFaultRootCauseFrame(const socketcan::CanFrame & frame);

private:
  uint8_t subsystem_id_;
  uint8_t bus_address_;
  SygnalSystemState sygnal_mcm_state_;
  std::array<SygnalSystemState, 5> sygnal_interface_states_;
  std::chrono::system_clock::time_point last_heartbeat_timestamp_;

  SygnalFaultCause last_fault_cause_ = NO_FAULT_CAUSE;
  SygnalFaultRootCause last_root_cause_ = {};
  std::map<SygnalFaultCause, uint16_t> fault_counts_;
  std::chrono::system_clock::time_point last_fault_state_timestamp_;
  std::chrono::system_clock::time_point last_root_cause_timestamp_;
  std::chrono::system_clock::time_point last_fault_count_timestamp_;
};

}  // namespace polymath::sygnal

#endif  // SYGNAL_CAN_INTERFACE_LIB__SYGNAL_MCM_INTERFACE_HPP_
