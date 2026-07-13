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

#include <chrono>

#include "sygnal_can_interface_lib/crc8.hpp"
#include "sygnal_dbc/mcm_heartbeat.h"
#include "sygnal_dbc/mcm_identify.h"

namespace polymath::sygnal
{

SygnalMcmInterface::SygnalMcmInterface()
: subsystem_id_(0)
, bus_address_(0)
, sygnal_mcm_state_(SygnalSystemState::FAIL_HARD)
{
  sygnal_interface_states_.fill(SygnalSystemState::FAIL_HARD);
}

SygnalMcmInterface::SygnalMcmInterface(const uint8_t bus_address, const uint8_t subsystem_id)
: subsystem_id_(subsystem_id)
, bus_address_(bus_address)
, sygnal_mcm_state_(SygnalSystemState::FAIL_HARD)
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

  // Only parse if this frame is for our subsystem
  if (unpacked_heartbeat_t.subsystem_id != subsystem_id_) {
    return false;
  }

  if (unpacked_heartbeat_t.bus_address != bus_address_) {
    return false;
  }

  last_heartbeat_timestamp_ = std::chrono::system_clock::now();

  // Set the MCM state
  sygnal_mcm_state_ = SygnalSystemState(unpacked_heartbeat_t.system_state);

  // Get interface states
  sygnal_interface_states_[0] = SygnalSystemState(unpacked_heartbeat_t.interface0_state);
  sygnal_interface_states_[1] = SygnalSystemState(unpacked_heartbeat_t.interface1_state);
  sygnal_interface_states_[2] = SygnalSystemState(unpacked_heartbeat_t.interface2_state);
  sygnal_interface_states_[3] = SygnalSystemState(unpacked_heartbeat_t.interface3_state);
  sygnal_interface_states_[4] = SygnalSystemState(unpacked_heartbeat_t.interface4_state);

  return true;
}

bool SygnalMcmInterface::parseIdentifyResponseFrame(const socketcan::CanFrame & frame)
{
  auto frame_copy = frame.get_frame();

  switch (frame.get_id()) {
    case MCM_IDENTIFY_IDENTIFY_RESPONSE_MAIN_FRAME_ID: {
      mcm_identify_identify_response_main_t unpacked;
      if (mcm_identify_identify_response_main_init(&unpacked) != 0) {
        return false;
      }
      if (mcm_identify_identify_response_main_unpack(&unpacked, frame_copy.data, frame_copy.len) != 0) {
        return false;
      }
      if (unpacked.bus_address != bus_address_ || unpacked.subsystem_id != subsystem_id_) {
        return false;
      }
      mcm_identity_.module_serial_number = unpacked.module_serial_number;
      mcm_identity_.product_id = unpacked.product_id;
      mcm_identity_.module_boot_state = unpacked.module_boot_state;
      mcm_identity_.has_main = true;
      return true;
    }

    case MCM_IDENTIFY_IDENTIFY_RESPONSE_APP_VERSION_FRAME_ID: {
      mcm_identify_identify_response_app_version_t unpacked;
      if (mcm_identify_identify_response_app_version_init(&unpacked) != 0) {
        return false;
      }
      if (mcm_identify_identify_response_app_version_unpack(&unpacked, frame_copy.data, frame_copy.len) != 0) {
        return false;
      }
      if (unpacked.bus_address != bus_address_ || unpacked.subsystem_id != subsystem_id_) {
        return false;
      }
      mcm_identity_.app_version = {
        unpacked.software_version_major, unpacked.software_version_minor, unpacked.software_version_patch};
      mcm_identity_.has_app_version = true;
      return true;
    }

    case MCM_IDENTIFY_IDENTIFY_RESPONSE_BL_VERSION_FRAME_ID: {
      mcm_identify_identify_response_bl_version_t unpacked;
      if (mcm_identify_identify_response_bl_version_init(&unpacked) != 0) {
        return false;
      }
      if (mcm_identify_identify_response_bl_version_unpack(&unpacked, frame_copy.data, frame_copy.len) != 0) {
        return false;
      }
      if (unpacked.bus_address != bus_address_ || unpacked.subsystem_id != subsystem_id_) {
        return false;
      }
      mcm_identity_.bl_version = {
        unpacked.software_version_major, unpacked.software_version_minor, unpacked.software_version_patch};
      mcm_identity_.has_bl_version = true;
      return true;
    }

    default:
      return false;
  }
}

}  // namespace polymath::sygnal
