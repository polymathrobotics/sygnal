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

#include "sygnal_can_interface_lib/sygnal_device.hpp"

#include <memory>
#include <mutex>
#include <utility>

#include "sygnal_can_interface_lib/crc8.hpp"

namespace polymath::sygnal
{

SygnalDevice::SygnalDevice(std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter)
: socketcan_adapter_(std::move(socketcan_adapter))
{}

bool SygnalDevice::handle(
  const can_frame & frame,
  std::chrono::steady_clock::time_point receive_time,
  std::chrono::system_clock::time_point bus_time)
{
  can_frame copy = frame;
  if (CAN_MAX_DLC != copy.len || !check_crc8(copy.data)) {
    return false;
  }
  return handler_.handle(frame, receive_time, bus_time);
}

SygnalSystemState SygnalDevice::get_system_state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return system_state_;
}

std::future<SygnalSystemState> SygnalDevice::expect_endpoint_state_response(const SygnalEndpoint & endpoint)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_responses_.push({endpoint.type, endpoint.endpoint_id});
}

std::future<double> SygnalDevice::expect_endpoint_value_response(const SygnalEndpoint & endpoint)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return value_responses_.push({endpoint.type, endpoint.endpoint_id});
}

void SygnalDevice::cancel_endpoint_state_response(const SygnalEndpoint & endpoint)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_responses_.withdraw_newest({endpoint.type, endpoint.endpoint_id});
}

void SygnalDevice::cancel_endpoint_value_response(const SygnalEndpoint & endpoint)
{
  std::lock_guard<std::mutex> lock(mutex_);
  value_responses_.withdraw_newest({endpoint.type, endpoint.endpoint_id});
}

void SygnalDevice::fulfill_endpoint_state_response(
  SygnalControlEndpointType type, uint8_t endpoint_id, SygnalSystemState state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_responses_.fulfill({type, endpoint_id}, state);
}

void SygnalDevice::fulfill_endpoint_value_response(SygnalControlEndpointType type, uint8_t endpoint_id, double value)
{
  std::lock_guard<std::mutex> lock(mutex_);
  value_responses_.fulfill({type, endpoint_id}, value);
}

}  // namespace polymath::sygnal
