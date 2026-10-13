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
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "sygnal_can_interface_lib/sygnal_device.hpp"

namespace polymath::sygnal
{

/// @brief Interfaces an HPO reports in its heartbeat.
constexpr uint8_t HPO_NUM_INTERFACES = 4;

/// @brief One HPO board: HPO_IF endpoints, where endpoint_id is the HPO MessageID.
///        Endpoint state is known for message IDs 0-3, the interfaces in the heartbeat.
///        An HPO has no subsystems: its subsystem_id is ignored and it receives frames with either bit 7 value.
class SygnalHpoDevice : public SygnalDevice
{
public:
  /// @param socketcan_adapter Adapter that commands go out on.
  SygnalHpoDevice(std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter, uint8_t bus_id);

  SygnalSystemState get_endpoint_state(const SygnalEndpoint & endpoint) const override;
  double get_endpoint_value(const SygnalEndpoint & endpoint) const override;
  EndpointStateCommandResult send_endpoint_state_command(
    const SygnalEndpoint & endpoint, bool enable, bool expect_reply) override;
  EndpointValueCommandResult send_endpoint_value_command(
    const SygnalEndpoint & endpoint, double value, bool expect_reply) override;

private:
  /// @brief The bus ID this HPO device is on.
  uint8_t bus_id_;

  /// @brief Mutex protecting access to the interface states and values.

  /// @brief The SygnalSystemState of each HPO interface as reported in the heartbeat.
  ///        FAIL_HARD until a heartbeat is received.
  std::array<SygnalSystemState, HPO_NUM_INTERFACES> interface_states_;

  /// @brief The last known value of each HPO interface as reported in the control command response.
  ///        Zero until a response is received.
  std::array<double, UINT8_MAX + 1> values_{};  ///< Indexed by MessageID.
};

}  // namespace polymath::sygnal
