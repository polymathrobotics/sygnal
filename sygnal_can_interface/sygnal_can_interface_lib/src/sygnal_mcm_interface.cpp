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

#include "sygnal_can_interface_lib/sygnal_mcm_interface.hpp"

#include "sygnal_can_interface_lib/crc8.hpp"
#include "sygnal_dbc/mcm_heartbeat.h"

namespace polymath::sygnal
{

SygnalMcmInterface::SygnalMcmInterface()
: sygnal_mcm_0_state_(SygnalSystemState::FAIL_HARD)
, sygnal_mcm_1_state_(SygnalSystemState::FAIL_HARD)
{
  sygnal_interface_states_.fill(SygnalSystemState::FAIL_HARD);
}

bool SygnalMcmInterface::parseMcmHeartbeatFrame(const socketcan::CanFrame & frame)
{
  if (frame.get_id() != MCM_HEARTBEAT_HEARTBEAT_FRAME_ID) {
    return false;
  }

  auto frame_copy = frame.get_frame();

  if (!check_crc8(reinterpret_cast<uint8_t *>(frame_copy.data))) {
    return false;
  }

  mcm_heartbeat_heartbeat_t unpacked_heartbeat_t;
  if (mcm_heartbeat_heartbeat_init(&unpacked_heartbeat_t) != 0) {
    return false;
  }

  if (mcm_heartbeat_heartbeat_unpack(&unpacked_heartbeat_t, frame_copy.data, frame_copy.len) != 0) {
    return false;
  }

  // Set the sygnal states based on subsystem_id
  if (unpacked_heartbeat_t.subsystem_id == 0) {
    sygnal_mcm_0_state_ = SygnalSystemState(unpacked_heartbeat_t.system_state);
  } else if (unpacked_heartbeat_t.subsystem_id == 1) {
    sygnal_mcm_1_state_ = SygnalSystemState(unpacked_heartbeat_t.system_state);
  }

  // Get interface states
  sygnal_interface_states_[0] = SygnalSystemState(unpacked_heartbeat_t.interface0_state);
  sygnal_interface_states_[1] = SygnalSystemState(unpacked_heartbeat_t.interface1_state);
  sygnal_interface_states_[2] = SygnalSystemState(unpacked_heartbeat_t.interface2_state);
  sygnal_interface_states_[3] = SygnalSystemState(unpacked_heartbeat_t.interface3_state);
  sygnal_interface_states_[4] = SygnalSystemState(unpacked_heartbeat_t.interface4_state);

  return true;
}

}  // namespace polymath::sygnal
