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

#include "sygnal_can_interface_lib/sygnal_mcm_device.hpp"

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
using polymath::sygnal::SygnalMcmDevice;
using polymath::sygnal::SygnalSystemState;

namespace
{

/// @brief Unopened adapter; these tests never send.
std::shared_ptr<polymath::socketcan::SocketcanAdapter> adapter()
{
  return std::make_shared<polymath::socketcan::SocketcanAdapter>("vcan0");
}

/// @brief Exposes the response queues, so replies can be awaited without sending.
class TestMcmDevice : public SygnalMcmDevice
{
public:
  using SygnalDevice::control_command_responses_;
  using SygnalDevice::control_enable_responses_;
  using SygnalMcmDevice::relay_responses_;
  using SygnalMcmDevice::SygnalMcmDevice;
};

constexpr uint8_t BUS = 1;
constexpr uint8_t SUBSYSTEM = 1;
constexpr SygnalControlDevice MCM{SygnalControlDeviceType::MCM, BUS, SUBSYSTEM, "mcm"};
constexpr std::chrono::system_clock::time_point RESPONSE_TIME{std::chrono::milliseconds(200)};
constexpr std::chrono::seconds COMMAND_TIMEOUT{1};

SygnalEndpoint endpoint(SygnalControlEndpointType type, uint8_t id)
{
  return {type, MCM, id, -1, 1, false, "test"};
}

SygnalEndpoint mcm_if(uint8_t id)
{
  return endpoint(SygnalControlEndpointType::MCM_INTERFACE, id);
}

SygnalEndpoint relay()
{
  return endpoint(SygnalControlEndpointType::MCM_RELAY, 0);
}

template <typename T>
bool is_ready(const std::future<T> & future)
{
  return std::future_status::ready == future.wait_for(std::chrono::seconds(0));
}

}  // namespace

TEST_CASE("MCM device reports FAIL_HARD until its first heartbeat", "[mcm_device]")
{
  TestMcmDevice device(adapter(), MCM, COMMAND_TIMEOUT);
  CHECK(SygnalSystemState::FAIL_HARD == device.get_device_system_state().value);
}

TEST_CASE("MCM heartbeat sets system and interface state", "[mcm_device]")
{
  TestMcmDevice device(adapter(), MCM, COMMAND_TIMEOUT);
  REQUIRE(device.handle(sygnal_test::mcm_heartbeat(BUS, SUBSYSTEM, SygnalSystemState::MCM_CONTROL, {false, true})));

  CHECK(SygnalSystemState::MCM_CONTROL == device.get_device_system_state().value);
  CHECK(SygnalSystemState::HUMAN_CONTROL == device.get_interface_control_state(mcm_if(0)).value);
  CHECK(SygnalSystemState::MCM_CONTROL == device.get_interface_control_state(mcm_if(1)).value);
}

TEST_CASE("MCM device rejects a frame with a bad CRC", "[mcm_device]")
{
  TestMcmDevice device(adapter(), MCM, COMMAND_TIMEOUT);
  can_frame frame = sygnal_test::mcm_heartbeat(BUS, SUBSYSTEM, SygnalSystemState::MCM_CONTROL);
  frame.data[CAN_MAX_DLC - 1] ^= 0xFF;

  CHECK_FALSE(device.handle(frame));
  CHECK(SygnalSystemState::FAIL_HARD == device.get_device_system_state().value);
}

TEST_CASE("MCM enable response fulfills only its interface's future", "[mcm_device]")
{
  TestMcmDevice device(adapter(), MCM, COMMAND_TIMEOUT);
  auto interface0 = device.control_enable_responses_.push(0);
  auto interface2 = device.control_enable_responses_.push(2);

  REQUIRE(device.handle(sygnal_test::mcm_enable_response(BUS, SUBSYSTEM, 2, true), {}, RESPONSE_TIME));

  CHECK_FALSE(is_ready(interface0));
  REQUIRE(is_ready(interface2));
  const auto response = interface2.get();
  CHECK(SygnalSystemState::MCM_CONTROL == response.value);
  CHECK(RESPONSE_TIME == response.timestamp);
  CHECK(SygnalSystemState::MCM_CONTROL == device.get_interface_control_state(mcm_if(2)).value);
}

TEST_CASE("MCM withdrawn response promise does not take the next response", "[mcm_device]")
{
  TestMcmDevice device(adapter(), MCM, COMMAND_TIMEOUT);
  auto withdrawn = device.control_enable_responses_.push(1);
  device.control_enable_responses_.withdraw_newest(1);
  auto next = device.control_enable_responses_.push(1);

  REQUIRE(device.handle(sygnal_test::mcm_enable_response(BUS, SUBSYSTEM, 1, true)));

  CHECK_THROWS_AS(withdrawn.get(), std::future_error);
  REQUIRE(is_ready(next));
  CHECK(SygnalSystemState::MCM_CONTROL == next.get().value);
}

TEST_CASE("MCM command response fulfills its future and caches the value", "[mcm_device]")
{
  TestMcmDevice device(adapter(), MCM, COMMAND_TIMEOUT);
  auto future = device.control_command_responses_.push(3);

  REQUIRE(device.handle(sygnal_test::mcm_command_response(BUS, SUBSYSTEM, 3, 0.25), {}, RESPONSE_TIME));

  REQUIRE(is_ready(future));
  const auto response = future.get();
  CHECK(0.25 == response.value);
  CHECK(RESPONSE_TIME == response.timestamp);
  CHECK(0.25 == device.get_control_command_value(mcm_if(3)).value);
  CHECK(0.0 == device.get_control_command_value(mcm_if(0)).value);
}

TEST_CASE("MCM relay response fulfills the relay future", "[mcm_device]")
{
  TestMcmDevice device(adapter(), MCM, COMMAND_TIMEOUT);
  auto interface0 = device.control_command_responses_.push(0);
  auto future = device.relay_responses_.push(0);

  REQUIRE(device.handle(sygnal_test::mcm_relay_response(BUS, SUBSYSTEM, true), {}, RESPONSE_TIME));

  CHECK_FALSE(is_ready(interface0));
  REQUIRE(is_ready(future));
  const auto response = future.get();
  CHECK(1.0 == response.value);
  CHECK(RESPONSE_TIME == response.timestamp);
  CHECK(1.0 == device.get_control_command_value(relay()).value);
}

TEST_CASE("MCM getters carry the bus time of the frame that set them", "[mcm_device]")
{
  TestMcmDevice device(adapter(), MCM, COMMAND_TIMEOUT);
  const std::chrono::system_clock::time_point heartbeat_time{std::chrono::milliseconds(100)};
  const std::chrono::system_clock::time_point enable_time{std::chrono::milliseconds(200)};

  CHECK(std::chrono::system_clock::time_point{} == device.get_device_system_state().timestamp);

  REQUIRE(device.handle(
    sygnal_test::mcm_heartbeat(BUS, SUBSYSTEM, SygnalSystemState::MCM_CONTROL, {true, true}), {}, heartbeat_time));
  REQUIRE(device.handle(sygnal_test::mcm_enable_response(BUS, SUBSYSTEM, 1, false), {}, enable_time));

  CHECK(heartbeat_time == device.get_device_system_state().timestamp);
  CHECK(heartbeat_time == device.get_interface_control_state(mcm_if(0)).timestamp);
  CHECK(enable_time == device.get_interface_control_state(mcm_if(1)).timestamp);
}

TEST_CASE("MCM device rejects endpoints and values it does not have", "[mcm_device]")
{
  TestMcmDevice device(adapter(), MCM, COMMAND_TIMEOUT);
  const auto hpo_if = endpoint(SygnalControlEndpointType::HPO_INTERFACE, 0);

  CHECK_THROWS_AS(device.send_control_enable_command(relay(), true, false), std::invalid_argument);
  CHECK_THROWS_AS(device.send_control_enable_command(mcm_if(7), true, false), std::invalid_argument);
  CHECK_THROWS_AS(device.send_control_enable_command(hpo_if, true, false), std::invalid_argument);
  CHECK_THROWS_AS(device.send_control_command(mcm_if(7), 1.0, false), std::invalid_argument);
  CHECK_THROWS_AS(device.send_control_command(hpo_if, 1.0, false), std::invalid_argument);

  CHECK_THROWS_AS(device.get_interface_control_state(relay()), std::invalid_argument);
  CHECK_THROWS_AS(device.get_interface_control_state(mcm_if(7)), std::invalid_argument);
  CHECK_THROWS_AS(device.get_control_command_value(mcm_if(7)), std::invalid_argument);
  CHECK_THROWS_AS(device.get_control_command_value(hpo_if), std::invalid_argument);
}
