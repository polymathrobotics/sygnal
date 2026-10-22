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

#include "sygnal_can_interface_lib/sygnal_cb_device.hpp"

#include <chrono>
#include <future>
#include <memory>
#include <stdexcept>

#if __has_include(<catch2/catch_all.hpp>)
  #include <catch2/catch_all.hpp>
#elif __has_include(<catch2/catch.hpp>)
  #include <catch2/catch.hpp>
#else
  #error "Catch2 headers not found. Please install Catch2 (v2 or v3)."
#endif

#include "cb_error/cb_error.hpp"
#include "sygnal_test_frames.hpp"  // NOLINT(build/include_subdir)

using polymath::sygnal::SygnalCbDevice;
using polymath::sygnal::SygnalControlDevice;
using polymath::sygnal::SygnalControlDeviceType;
using polymath::sygnal::SygnalControlEndpointType;
using polymath::sygnal::SygnalDevice;
using polymath::sygnal::SygnalEndpoint;
using polymath::sygnal::SygnalErrorType;
using polymath::sygnal::SygnalSystemState;

namespace
{

/// @brief Unopened adapter; these tests never send.
std::shared_ptr<polymath::socketcan::SocketcanAdapter> adapter()
{
  return std::make_shared<polymath::socketcan::SocketcanAdapter>("vcan0");
}

/// @brief Exposes the response queues, so replies can be awaited without sending.
class TestCbDevice : public SygnalCbDevice
{
public:
  using SygnalCbDevice::SygnalCbDevice;
  using SygnalDevice::control_command_responses_;
  using SygnalDevice::control_enable_responses_;
};

constexpr uint8_t BUS = 4;
constexpr SygnalControlDevice CB{SygnalControlDeviceType::CB, BUS, 0, "cb"};
constexpr std::chrono::system_clock::time_point RESPONSE_TIME{std::chrono::milliseconds(200)};
constexpr std::chrono::seconds COMMAND_TIMEOUT{1};

SygnalEndpoint cb_if(uint8_t id)
{
  return {SygnalControlEndpointType::CB_INTERFACE, CB, id, 0, 1, true, "test"};
}

template <typename T>
bool is_ready(const std::future<T> & future)
{
  return std::future_status::ready == future.wait_for(std::chrono::seconds(0));
}

}  // namespace

TEST_CASE("CB heartbeat sets the system state", "[cb_device]")
{
  TestCbDevice device(adapter(), CB, COMMAND_TIMEOUT);
  CHECK(SygnalSystemState::FAIL_HARD == device.get_device_system_state().value);

  REQUIRE(device.handle(sygnal_test::cb_heartbeat(BUS, SygnalSystemState::MCM_CONTROL), {}, RESPONSE_TIME));

  CHECK(SygnalSystemState::MCM_CONTROL == device.get_device_system_state().value);
  CHECK(RESPONSE_TIME == device.get_device_system_state().timestamp);
}

TEST_CASE("CB enable response fulfills only its message's future", "[cb_device]")
{
  TestCbDevice device(adapter(), CB, COMMAND_TIMEOUT);
  auto message1 = device.control_enable_responses_.push(1);
  auto message200 = device.control_enable_responses_.push(200);

  REQUIRE(device.handle(sygnal_test::cb_enable_response(BUS, 200, true), {}, RESPONSE_TIME));

  CHECK_FALSE(is_ready(message1));
  REQUIRE(is_ready(message200));
  const auto response = message200.get();
  CHECK(SygnalSystemState::MCM_CONTROL == response.value);
  CHECK(RESPONSE_TIME == response.timestamp);
}

TEST_CASE("CB command response fulfills its future and caches the value", "[cb_device]")
{
  TestCbDevice device(adapter(), CB, COMMAND_TIMEOUT);
  auto future = device.control_command_responses_.push(9);

  REQUIRE(device.handle(sygnal_test::cb_command_response(BUS, 9, -0.5), {}, RESPONSE_TIME));

  REQUIRE(is_ready(future));
  const auto response = future.get();
  CHECK(-0.5 == response.value);
  CHECK(RESPONSE_TIME == response.timestamp);
  CHECK(-0.5 == device.get_control_command_value(cb_if(9)).value);
  CHECK(0.0 == device.get_control_command_value(cb_if(10)).value);
}

TEST_CASE("CB get_new_device_errors returns its errors and clears them", "[cb_device]")
{
  TestCbDevice device(adapter(), CB, COMMAND_TIMEOUT);

  REQUIRE(device.handle(sygnal_test::error_status<cb_error::ErrorStatus>(BUS, 0, SygnalErrorType::COUNT, 0x160)));

  const auto errors = device.get_new_device_errors();
  REQUIRE(1 == errors.size());  // NOLINT(readability/check)
  CHECK(SygnalErrorType::COUNT == errors[0].value.type);
  CHECK(device.get_new_device_errors().empty());
}

TEST_CASE("CB device rejects endpoints and values it does not have", "[cb_device]")
{
  TestCbDevice device(adapter(), CB, COMMAND_TIMEOUT);
  const SygnalEndpoint hpo_if{SygnalControlEndpointType::HPO_INTERFACE, CB, 0, 0, 1, false, "test"};

  CHECK_THROWS_AS(device.send_control_enable_command(hpo_if, true, false), std::invalid_argument);
  CHECK_THROWS_AS(device.send_control_command(hpo_if, 1.0, false), std::invalid_argument);
  CHECK_THROWS_AS(device.get_interface_control_state(hpo_if), std::invalid_argument);
  CHECK_THROWS_AS(device.get_interface_control_state(cb_if(0)), std::invalid_argument);
  CHECK_THROWS_AS(device.get_control_command_value(hpo_if), std::invalid_argument);
  CHECK_THROWS_AS(device.get_device_faults(), std::invalid_argument);
}
