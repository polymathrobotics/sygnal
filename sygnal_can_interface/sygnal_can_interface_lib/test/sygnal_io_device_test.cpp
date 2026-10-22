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

#include "sygnal_can_interface_lib/sygnal_io_device.hpp"

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

#include "io_error/io_error.hpp"
#include "sygnal_test_frames.hpp"  // NOLINT(build/include_subdir)

using polymath::sygnal::SygnalControlDevice;
using polymath::sygnal::SygnalControlDeviceType;
using polymath::sygnal::SygnalControlEndpointType;
using polymath::sygnal::SygnalDevice;
using polymath::sygnal::SygnalEndpoint;
using polymath::sygnal::SygnalErrorType;
using polymath::sygnal::SygnalIoDevice;
using polymath::sygnal::SygnalSystemState;

namespace
{

/// @brief Unopened adapter; these tests never send.
std::shared_ptr<polymath::socketcan::SocketcanAdapter> adapter()
{
  return std::make_shared<polymath::socketcan::SocketcanAdapter>("vcan0");
}

/// @brief Exposes the response queues, so replies can be awaited without sending.
class TestIoDevice : public SygnalIoDevice
{
public:
  using SygnalIoDevice::SygnalIoDevice;
  using SygnalDevice::control_command_responses_;
  using SygnalDevice::control_enable_responses_;
};

constexpr uint8_t BUS = 6;
constexpr SygnalControlDevice IO{SygnalControlDeviceType::IO, BUS, 0, "io"};
constexpr std::chrono::system_clock::time_point RESPONSE_TIME{std::chrono::milliseconds(200)};
constexpr std::chrono::seconds COMMAND_TIMEOUT{1};

SygnalEndpoint io_if(uint8_t id)
{
  return {SygnalControlEndpointType::IO_INTERFACE, IO, id, 0, 1, false, "test"};
}

template <typename T>
bool is_ready(const std::future<T> & future)
{
  return std::future_status::ready == future.wait_for(std::chrono::seconds(0));
}

}  // namespace

TEST_CASE("IO heartbeat sets system and interface state", "[io_device]")
{
  TestIoDevice device(adapter(), IO, COMMAND_TIMEOUT);
  CHECK(SygnalSystemState::FAIL_HARD == device.get_device_system_state().value);
  CHECK(SygnalSystemState::FAIL_HARD == device.get_interface_control_state(io_if(0)).value);

  REQUIRE(device.handle(
    sygnal_test::io_heartbeat(BUS, SygnalSystemState::MCM_CONTROL, {false, false, false, false, false, false, true}),
    {},
    RESPONSE_TIME));

  CHECK(SygnalSystemState::MCM_CONTROL == device.get_device_system_state().value);
  CHECK(SygnalSystemState::HUMAN_CONTROL == device.get_interface_control_state(io_if(0)).value);
  CHECK(SygnalSystemState::MCM_CONTROL == device.get_interface_control_state(io_if(6)).value);
  CHECK(RESPONSE_TIME == device.get_interface_control_state(io_if(6)).timestamp);
}

TEST_CASE("IO enable response fulfills only its interface's future", "[io_device]")
{
  TestIoDevice device(adapter(), IO, COMMAND_TIMEOUT);
  auto interface0 = device.control_enable_responses_.push(0);
  auto interface3 = device.control_enable_responses_.push(3);

  REQUIRE(device.handle(sygnal_test::io_enable_response(BUS, 3, true), {}, RESPONSE_TIME));

  CHECK_FALSE(is_ready(interface0));
  REQUIRE(is_ready(interface3));
  const auto response = interface3.get();
  CHECK(SygnalSystemState::MCM_CONTROL == response.value);
  CHECK(RESPONSE_TIME == response.timestamp);
  CHECK(SygnalSystemState::MCM_CONTROL == device.get_interface_control_state(io_if(3)).value);
}

TEST_CASE("IO command response fulfills its future and caches the value", "[io_device]")
{
  TestIoDevice device(adapter(), IO, COMMAND_TIMEOUT);
  auto future = device.control_command_responses_.push(5);

  REQUIRE(device.handle(sygnal_test::io_command_response(BUS, 5, 0.75), {}, RESPONSE_TIME));

  REQUIRE(is_ready(future));
  const auto response = future.get();
  CHECK(0.75 == response.value);
  CHECK(RESPONSE_TIME == response.timestamp);
  CHECK(0.75 == device.get_control_command_value(io_if(5)).value);
  CHECK(0.0 == device.get_control_command_value(io_if(0)).value);
}

TEST_CASE("IO get_new_device_errors returns its errors and clears them", "[io_device]")
{
  TestIoDevice device(adapter(), IO, COMMAND_TIMEOUT);

  REQUIRE(device.handle(sygnal_test::error_status<io_error::ErrorStatus>(BUS, 0, SygnalErrorType::STATE, 0x60)));

  const auto errors = device.get_new_device_errors();
  REQUIRE(1 == errors.size());  // NOLINT(readability/check)
  CHECK(SygnalErrorType::STATE == errors[0].value.type);
  CHECK(device.get_new_device_errors().empty());
}

TEST_CASE("IO device rejects endpoints and values it does not have", "[io_device]")
{
  TestIoDevice device(adapter(), IO, COMMAND_TIMEOUT);
  const SygnalEndpoint mcm_if{SygnalControlEndpointType::MCM_INTERFACE, IO, 0, 0, 1, false, "test"};

  CHECK_THROWS_AS(device.send_control_enable_command(mcm_if, true, false), std::invalid_argument);
  CHECK_THROWS_AS(device.send_control_enable_command(io_if(7), true, false), std::invalid_argument);
  CHECK_THROWS_AS(device.send_control_command(mcm_if, 1.0, false), std::invalid_argument);
  CHECK_THROWS_AS(device.send_control_command(io_if(7), 1.0, false), std::invalid_argument);
  CHECK_THROWS_AS(device.get_interface_control_state(io_if(7)), std::invalid_argument);
  CHECK_THROWS_AS(device.get_control_command_value(io_if(7)), std::invalid_argument);
  CHECK_THROWS_AS(device.get_device_faults(), std::invalid_argument);
}
