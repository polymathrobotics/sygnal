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

using polymath::sygnal::SygnalControlDevice;
using polymath::sygnal::SygnalControlDeviceType;
using polymath::sygnal::SygnalControlEndpointType;
using polymath::sygnal::SygnalDevice;
using polymath::sygnal::SygnalEndpoint;
using polymath::sygnal::SygnalErrorType;
using polymath::sygnal::SygnalHpoDevice;
using polymath::sygnal::SygnalSystemState;

namespace
{

/// @brief Unopened adapter; these tests never send.
std::shared_ptr<polymath::socketcan::SocketcanAdapter> adapter()
{
  return std::make_shared<polymath::socketcan::SocketcanAdapter>("vcan0");
}

/// @brief Exposes the response queues, so replies can be awaited without sending.
class TestHpoDevice : public SygnalHpoDevice
{
public:
  using SygnalHpoDevice::SygnalHpoDevice;
  using SygnalDevice::control_command_responses_;
  using SygnalDevice::control_enable_responses_;
};

constexpr uint8_t BUS = 3;
constexpr SygnalControlDevice HPO{SygnalControlDeviceType::HPO, BUS, 0, "hpo"};
constexpr std::chrono::system_clock::time_point RESPONSE_TIME{std::chrono::milliseconds(200)};
constexpr std::chrono::seconds COMMAND_TIMEOUT{1};

SygnalEndpoint hpo_if(uint8_t id)
{
  return {SygnalControlEndpointType::HPO_INTERFACE, HPO, id, 0, 1, false, "test"};
}

template <typename T>
bool is_ready(const std::future<T> & future)
{
  return std::future_status::ready == future.wait_for(std::chrono::seconds(0));
}

}  // namespace

TEST_CASE("HPO heartbeat sets system and interface state", "[hpo_device]")
{
  TestHpoDevice device(adapter(), HPO, COMMAND_TIMEOUT);
  CHECK(SygnalSystemState::FAIL_HARD == device.get_device_system_state().value);
  CHECK(SygnalSystemState::FAIL_HARD == device.get_interface_control_state(hpo_if(0)).value);

  REQUIRE(device.handle(sygnal_test::hpo_heartbeat(BUS, SygnalSystemState::HUMAN_CONTROL, {false, false, true})));

  CHECK(SygnalSystemState::HUMAN_CONTROL == device.get_device_system_state().value);
  CHECK(SygnalSystemState::HUMAN_CONTROL == device.get_interface_control_state(hpo_if(0)).value);
  CHECK(SygnalSystemState::MCM_CONTROL == device.get_interface_control_state(hpo_if(2)).value);
}

TEST_CASE("HPO heartbeat sets interface values from the pin state", "[hpo_device]")
{
  TestHpoDevice device(adapter(), HPO, COMMAND_TIMEOUT);

  REQUIRE(device.handle(
    sygnal_test::hpo_heartbeat(BUS, SygnalSystemState::MCM_CONTROL, {}, {0, 1, 2, 1}), {}, RESPONSE_TIME));

  CHECK(0.0 == device.get_control_command_value(hpo_if(0)).value);
  CHECK(1.0 == device.get_control_command_value(hpo_if(1)).value);
  CHECK(2.0 == device.get_control_command_value(hpo_if(2)).value);
  CHECK(1.0 == device.get_control_command_value(hpo_if(3)).value);
  CHECK(RESPONSE_TIME == device.get_control_command_value(hpo_if(3)).timestamp);
}

TEST_CASE("HPO enable response fulfills only its message's future", "[hpo_device]")
{
  TestHpoDevice device(adapter(), HPO, COMMAND_TIMEOUT);
  auto message0 = device.control_enable_responses_.push(0);
  auto message1 = device.control_enable_responses_.push(1);

  REQUIRE(device.handle(sygnal_test::hpo_enable_response(BUS, 1, true), {}, RESPONSE_TIME));

  CHECK_FALSE(is_ready(message0));
  REQUIRE(is_ready(message1));
  const auto response = message1.get();
  CHECK(SygnalSystemState::MCM_CONTROL == response.value);
  CHECK(RESPONSE_TIME == response.timestamp);
  CHECK(SygnalSystemState::MCM_CONTROL == device.get_interface_control_state(hpo_if(1)).value);
}

TEST_CASE("HPO command response fulfills its future and caches the value", "[hpo_device]")
{
  TestHpoDevice device(adapter(), HPO, COMMAND_TIMEOUT);
  auto future = device.control_command_responses_.push(10);

  REQUIRE(device.handle(sygnal_test::hpo_command_response(BUS, 10, 1.5), {}, RESPONSE_TIME));

  REQUIRE(is_ready(future));
  const auto response = future.get();
  CHECK(1.5 == response.value);
  CHECK(RESPONSE_TIME == response.timestamp);
  CHECK(1.5 == device.get_control_command_value(hpo_if(10)).value);
}

TEST_CASE("HPO getters carry the bus time of the frame that set them", "[hpo_device]")
{
  TestHpoDevice device(adapter(), HPO, COMMAND_TIMEOUT);
  const std::chrono::system_clock::time_point heartbeat_time{std::chrono::milliseconds(100)};
  const std::chrono::system_clock::time_point response_time{std::chrono::milliseconds(200)};

  CHECK(std::chrono::system_clock::time_point{} == device.get_device_system_state().timestamp);

  REQUIRE(device.handle(
    sygnal_test::hpo_heartbeat(BUS, SygnalSystemState::HUMAN_CONTROL, {false, true}), {}, heartbeat_time));
  REQUIRE(device.handle(sygnal_test::hpo_command_response(BUS, 1, 0.5), {}, response_time));

  CHECK(heartbeat_time == device.get_device_system_state().timestamp);
  CHECK(heartbeat_time == device.get_interface_control_state(hpo_if(1)).timestamp);
  CHECK(response_time == device.get_control_command_value(hpo_if(1)).timestamp);
}

TEST_CASE("HPO device rejects endpoints and values it does not have", "[hpo_device]")
{
  TestHpoDevice device(adapter(), HPO, COMMAND_TIMEOUT);
  const SygnalEndpoint mcm_if{SygnalControlEndpointType::MCM_INTERFACE, HPO, 0, 0, 1, false, "test"};

  CHECK_THROWS_AS(device.send_control_enable_command(mcm_if, true, false), std::invalid_argument);
  CHECK_THROWS_AS(device.send_control_command(mcm_if, 1.0, false), std::invalid_argument);
  CHECK_THROWS_AS(device.get_interface_control_state(hpo_if(4)), std::invalid_argument);
  CHECK_THROWS_AS(device.get_control_command_value(mcm_if), std::invalid_argument);
  CHECK_THROWS_AS(device.get_device_faults(), std::invalid_argument);
}

TEST_CASE("HPO get_new_device_errors returns its errors and clears them", "[hpo_device]")
{
  TestHpoDevice device(adapter(), HPO, COMMAND_TIMEOUT);

  REQUIRE(device.handle(
    sygnal_test::error_status<hpo_error::ErrorStatus>(BUS, 0, SygnalErrorType::INTERFACE_ID, 0x160), {}, RESPONSE_TIME));

  const auto errors = device.get_new_device_errors();
  REQUIRE(1 == errors.size());  // NOLINT(readability/check)
  CHECK(SygnalErrorType::INTERFACE_ID == errors[0].value.type);
  CHECK(0x160 == errors[0].value.can_id);  // NOLINT(readability/check)
  CHECK(RESPONSE_TIME == errors[0].timestamp);
  CHECK(device.get_new_device_errors().empty());
}
