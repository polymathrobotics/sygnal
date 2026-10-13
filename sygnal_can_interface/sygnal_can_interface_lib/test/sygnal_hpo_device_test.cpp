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

#include "sygnal_can_interface_lib/sygnal_hpo_device.hpp"

#include <chrono>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>

#if __has_include(<catch2/catch_all.hpp>)
  #include <catch2/catch_all.hpp>
#elif __has_include(<catch2/catch.hpp>)
  #include <catch2/catch.hpp>
#else
  #error "Catch2 headers not found. Please install Catch2 (v2 or v3)."
#endif

#include "sygnal_test_frames.hpp"  // NOLINT(build/include_subdir)

using polymath::sygnal::SygnalControlDeviceType;
using polymath::sygnal::SygnalControlEndpointType;
using polymath::sygnal::SygnalEndpoint;
using polymath::sygnal::SygnalHpoDevice;
using polymath::sygnal::SygnalSystemState;

namespace
{

/// @brief Unopened adapter; these tests never send.
std::shared_ptr<polymath::socketcan::SocketcanAdapter> adapter()
{
  return std::make_shared<polymath::socketcan::SocketcanAdapter>("vcan0");
}

constexpr uint8_t BUS = 3;

SygnalEndpoint hpo_if(uint8_t id)
{
  return {SygnalControlEndpointType::HPO_IF, {SygnalControlDeviceType::HPO, BUS, 0}, id, 0, 1, false, "test"};
}

template <typename T>
bool is_ready(const std::future<T> & future)
{
  return std::future_status::ready == future.wait_for(std::chrono::seconds(0));
}

}  // namespace

TEST_CASE("HPO heartbeat sets system and interface state", "[hpo_device]")
{
  SygnalHpoDevice device(adapter(), BUS);
  CHECK(SygnalSystemState::FAIL_HARD == device.get_system_state());
  CHECK(SygnalSystemState::FAIL_HARD == device.get_endpoint_state(hpo_if(0)));

  REQUIRE(device.handle(sygnal_test::hpo_heartbeat(BUS, SygnalSystemState::HUMAN_CONTROL, {false, false, true})));

  CHECK(SygnalSystemState::HUMAN_CONTROL == device.get_system_state());
  CHECK(SygnalSystemState::HUMAN_CONTROL == device.get_endpoint_state(hpo_if(0)));
  CHECK(SygnalSystemState::MCM_CONTROL == device.get_endpoint_state(hpo_if(2)));
}

TEST_CASE("HPO enable response fulfills only its message's future", "[hpo_device]")
{
  SygnalHpoDevice device(adapter(), BUS);
  auto message0 = device.expect_endpoint_state_response(hpo_if(0));
  auto message1 = device.expect_endpoint_state_response(hpo_if(1));

  REQUIRE(device.handle(sygnal_test::hpo_enable_response(BUS, 1, true)));

  CHECK_FALSE(is_ready(message0));
  REQUIRE(is_ready(message1));
  CHECK(SygnalSystemState::MCM_CONTROL == message1.get());
  CHECK(SygnalSystemState::MCM_CONTROL == device.get_endpoint_state(hpo_if(1)));
}

TEST_CASE("HPO command response fulfills its future and caches the value", "[hpo_device]")
{
  SygnalHpoDevice device(adapter(), BUS);
  auto future = device.expect_endpoint_value_response(hpo_if(10));

  REQUIRE(device.handle(sygnal_test::hpo_command_response(BUS, 10, 1.5)));

  REQUIRE(is_ready(future));
  CHECK(1.5 == future.get());
  CHECK(1.5 == device.get_endpoint_value(hpo_if(10)));
}

TEST_CASE("HPO device rejects endpoints and values it does not have", "[hpo_device]")
{
  SygnalHpoDevice device(adapter(), BUS);
  const SygnalEndpoint mcm_if{
    SygnalControlEndpointType::MCM_IF, {SygnalControlDeviceType::HPO, BUS, 0}, 0, 0, 1, false, "test"};

  const auto state_result = device.send_endpoint_state_command(mcm_if, true, false);
  CHECK_FALSE(state_result.success);
  CHECK_FALSE(state_result.error_message.empty());
  const auto value_result = device.send_endpoint_value_command(mcm_if, 1.0, false);
  CHECK_FALSE(value_result.success);
  CHECK_FALSE(value_result.error_message.empty());

  CHECK_THROWS_AS(device.get_endpoint_state(hpo_if(4)), std::invalid_argument);
  CHECK_THROWS_AS(device.get_endpoint_value(mcm_if), std::invalid_argument);
}
