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
#include <string>

#include "socketcan_adapter/can_frame.hpp"
#include "socketcan_adapter/socketcan_adapter.hpp"

namespace polymath::sygnal
{

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
  ~SygnalMcmInterface() = default;

  std::array<SygnalSystemState, 5> get_interface_states() const
  {
    return sygnal_interface_states_;
  }

  SygnalSystemState get_sygnal_mcm0_state() const
  {
    return sygnal_mcm_0_state_;
  }

  SygnalSystemState get_sygnal_mcm1_state() const
  {
    return sygnal_mcm_1_state_;
  }

  bool parseMcmHeartbeatFrame(const socketcan::CanFrame & frame);

private:
  SygnalSystemState sygnal_mcm_0_state_;
  SygnalSystemState sygnal_mcm_1_state_;
  std::array<SygnalSystemState, 5> sygnal_interface_states_;
};

}  // namespace polymath::sygnal

#endif  // SYGNAL_CAN_INTERFACE_LIB__SYGNAL_MCM_INTERFACE_HPP_
