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

#include <linux/can.h>

#include <array>
#include <cstdint>

#include "hpo_control/hpo_control.hpp"
#include "hpo_heartbeat/hpo_heartbeat.hpp"
#include "mcm_control/mcm_control.hpp"
#include "mcm_fault/mcm_fault.hpp"
#include "mcm_heartbeat/mcm_heartbeat.hpp"
#include "mcm_relay/mcm_relay.hpp"
#include "sygnal_can_interface_lib/crc8.hpp"
#include "sygnal_can_interface_lib/sygnal_types.hpp"

namespace sygnal_test
{

/// @brief Pack msg into a frame with a valid CRC.
template <typename T>
can_frame to_frame(const T & msg)
{
  can_frame frame = msg;
  frame.data[CAN_MAX_DLC - 1] = polymath::sygnal::generate_crc8(frame.data);
  return frame;
}

inline can_frame mcm_heartbeat(
  uint8_t bus_id,
  uint8_t subsystem_id,
  polymath::sygnal::SygnalSystemState state,
  const std::array<bool, 7> & interfaces_enabled = {})
{
  mcm_heartbeat::Heartbeat msg;
  msg.bus_address = bus_id;
  msg.subsystem_id = subsystem_id;
  msg.system_state = static_cast<uint8_t>(state);
  msg.interface0_state = interfaces_enabled[0];
  msg.interface1_state = interfaces_enabled[1];
  msg.interface2_state = interfaces_enabled[2];
  msg.interface3_state = interfaces_enabled[3];
  msg.interface4_state = interfaces_enabled[4];
  msg.interface5_state = interfaces_enabled[5];
  msg.interface6_state = interfaces_enabled[6];
  msg.overall_interface_state = 0;
  msg.count16 = 0;
  msg.crc = 0;
  return to_frame(msg);
}

inline can_frame mcm_enable_response(uint8_t bus_id, uint8_t subsystem_id, uint8_t interface_id, bool enable)
{
  mcm_control::ControlEnableResponse msg;
  msg.bus_address = bus_id;
  msg.sub_system_id = subsystem_id;
  msg.interface_id = interface_id;
  msg.enable = enable;
  msg.crc = 0;
  return to_frame(msg);
}

inline can_frame mcm_command_response(uint8_t bus_id, uint8_t subsystem_id, uint8_t interface_id, double value)
{
  mcm_control::ControlCommandResponse msg;
  msg.bus_address = bus_id;
  msg.sub_system_id = subsystem_id;
  msg.interface_id = interface_id;
  msg.count8 = 0;
  msg.value = value;
  msg.crc = 0;
  return to_frame(msg);
}

inline can_frame mcm_relay_response(uint8_t bus_id, uint8_t subsystem_id, bool enable)
{
  mcm_relay::RelayCommandResponse msg;
  msg.bus_address = bus_id;
  msg.subsystem_id = subsystem_id;
  msg.enable = enable;
  msg.crc = 0;
  return to_frame(msg);
}

inline can_frame mcm_fault_state(
  uint8_t bus_id, uint8_t subsystem_id, polymath::sygnal::SygnalSystemState state, uint8_t cause)
{
  mcm_fault::FaultState msg;
  msg.bus_address = bus_id;
  msg.subsystem_id = subsystem_id;
  msg.fault_state = static_cast<uint8_t>(state);
  msg.fault_cause = cause;
  msg.crc = 0;
  return to_frame(msg);
}

inline can_frame mcm_fault_increment(uint8_t bus_id, uint8_t subsystem_id, uint8_t cause, uint16_t count)
{
  mcm_fault::FaultIncrement msg;
  msg.bus_address = bus_id;
  msg.subsystem_id = subsystem_id;
  msg.fault_type = cause;
  msg.fault_count = count;
  msg.crc = 0;
  return to_frame(msg);
}

inline can_frame mcm_fault_list(uint8_t bus_id, uint8_t subsystem_id, uint8_t cause, uint16_t count)
{
  mcm_fault::FaultList msg;
  msg.bus_address = bus_id;
  msg.subsystem_id = subsystem_id;
  msg.fault_type = cause;
  msg.fault_count = count;
  msg.crc = 0;
  return to_frame(msg);
}

inline can_frame mcm_fault_root_cause(
  uint8_t bus_id, uint8_t subsystem_id, uint8_t fail_op1, uint8_t fail_op2, uint8_t fail_hard)
{
  mcm_fault::FaultRootCause msg;
  msg.bus_address = bus_id;
  msg.subsystem_id = subsystem_id;
  msg.fail_op1_cause = fail_op1;
  msg.fail_op2_cause = fail_op2;
  msg.fail_hard_cause = fail_hard;
  msg.crc = 0;
  return to_frame(msg);
}

inline can_frame hpo_heartbeat(
  uint8_t bus_id,
  polymath::sygnal::SygnalSystemState state,
  const std::array<bool, 4> & interfaces_enabled = {},
  const std::array<uint8_t, 4> & interface_values = {})
{
  hpo_heartbeat::Heartbeat msg;
  msg.bus_address = bus_id;
  msg.subsystem_id = 0;
  msg.system_state = static_cast<uint8_t>(state);
  msg.interface0_state = interfaces_enabled[0];
  msg.interface1_state = interfaces_enabled[1];
  msg.interface2_state = interfaces_enabled[2];
  msg.interface3_state = interfaces_enabled[3];
  msg.interface0_value = interface_values[0];
  msg.interface1_value = interface_values[1];
  msg.interface2_value = interface_values[2];
  msg.interface3_value = interface_values[3];
  msg.count16 = 0;
  msg.crc = 0;
  return to_frame(msg);
}

inline can_frame hpo_enable_response(uint8_t bus_id, uint8_t message_id, bool enable)
{
  hpo_control::ControlEnableResponse msg;
  msg.bus_address = bus_id;
  msg.message_id = message_id;
  msg.enable = enable;
  msg.crc = 0;
  return to_frame(msg);
}

inline can_frame hpo_command_response(uint8_t bus_id, uint8_t message_id, double value)
{
  hpo_control::ControlCommandResponse msg;
  msg.bus_address = bus_id;
  msg.message_id = message_id;
  msg.count8 = 0;
  msg.value = value;
  msg.crc = 0;
  return to_frame(msg);
}

}  // namespace sygnal_test
