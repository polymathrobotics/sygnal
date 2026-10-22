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
#include <string>

#if __has_include(<catch2/catch_all.hpp>)
  #include <catch2/catch_all.hpp>
#elif __has_include(<catch2/catch.hpp>)
  #include <catch2/catch.hpp>
#else
  #error "Catch2 headers not found. Please install Catch2 (v2 or v3)."
#endif

#include "sygnal_test_frames.hpp"  // NOLINT(build/include_subdir)

using polymath::sygnal::SygnalControlState;
using polymath::sygnal::SygnalDevice;
using polymath::sygnal::SygnalMcmDevice;
using polymath::sygnal::SygnalSystemState;

namespace
{

/// @brief Unopened adapter; every send fails.
std::shared_ptr<polymath::socketcan::SocketcanAdapter> adapter()
{
  return std::make_shared<polymath::socketcan::SocketcanAdapter>("vcan0");
}

/// @brief Exposes the response queues, so replies can be awaited without sending.
class TestMcmDevice : public SygnalMcmDevice
{
public:
  using SygnalMcmDevice::control_command_responses_;
  using SygnalMcmDevice::control_enable_responses_;
  using SygnalMcmDevice::relay_responses_;
  using SygnalMcmDevice::SygnalMcmDevice;
};

constexpr uint8_t BUS = 1;
constexpr uint8_t SUBSYSTEM = 1;
constexpr std::chrono::seconds COMMAND_TIMEOUT{1};

template <typename T>
bool is_ready(const std::future<T> & future)
{
  return std::future_status::ready == future.wait_for(std::chrono::seconds(0));
}

}  // namespace

TEST_CASE("MCM device reports FAIL_HARD until its first heartbeat", "[mcm_device]")
{
  TestMcmDevice device(adapter(), BUS, SUBSYSTEM, COMMAND_TIMEOUT);
  CHECK(SygnalSystemState::FAIL_HARD == device.get_mcm_state());
  CHECK(SygnalSystemState::FAIL_HARD == device.get_interface_states()[0]);
}

TEST_CASE("MCM heartbeat sets system and interface state", "[mcm_device]")
{
  TestMcmDevice device(adapter(), BUS, SUBSYSTEM, COMMAND_TIMEOUT);
  REQUIRE(device.handle(sygnal_test::mcm_heartbeat(BUS, SUBSYSTEM, SygnalSystemState::MCM_CONTROL, {false, true})));

  CHECK(SygnalSystemState::MCM_CONTROL == device.get_mcm_state());
  CHECK(SygnalSystemState::HUMAN_CONTROL == device.get_interface_states()[0]);
  CHECK(SygnalSystemState::MCM_CONTROL == device.get_interface_states()[1]);
}

TEST_CASE("MCM device rejects a frame with a bad CRC or length", "[mcm_device]")
{
  TestMcmDevice device(adapter(), BUS, SUBSYSTEM, COMMAND_TIMEOUT);
  can_frame frame = sygnal_test::mcm_heartbeat(BUS, SUBSYSTEM, SygnalSystemState::MCM_CONTROL);

  SECTION("bad CRC")
  {
    frame.data[CAN_MAX_DLC - 1] ^= 0xFF;
  }

  SECTION("short frame")
  {
    frame.len = CAN_MAX_DLC - 1;
  }

  CHECK_FALSE(device.handle(frame));
  CHECK(SygnalSystemState::FAIL_HARD == device.get_mcm_state());
}

TEST_CASE("MCM enable response fulfills only its interface's future", "[mcm_device]")
{
  TestMcmDevice device(adapter(), BUS, SUBSYSTEM, COMMAND_TIMEOUT);
  auto interface0 = device.control_enable_responses_.push(0);
  auto interface2 = device.control_enable_responses_.push(2);

  REQUIRE(device.handle(sygnal_test::mcm_enable_response(BUS, SUBSYSTEM, 2, true)));

  CHECK_FALSE(is_ready(interface0));
  REQUIRE(is_ready(interface2));
  const auto response = interface2.get();
  CHECK(2 == response.interface_id);  // NOLINT(readability/check)
  CHECK(1.0 == response.value);
}

TEST_CASE("MCM command response fulfills its interface's future", "[mcm_device]")
{
  TestMcmDevice device(adapter(), BUS, SUBSYSTEM, COMMAND_TIMEOUT);
  auto future = device.control_command_responses_.push(3);

  REQUIRE(device.handle(sygnal_test::mcm_command_response(BUS, SUBSYSTEM, 3, 0.25)));

  REQUIRE(is_ready(future));
  CHECK(0.25 == future.get().value);
}

TEST_CASE("MCM relay response fulfills the relay future only", "[mcm_device]")
{
  TestMcmDevice device(adapter(), BUS, SUBSYSTEM, COMMAND_TIMEOUT);
  auto interface0 = device.control_command_responses_.push(0);
  auto relay = device.relay_responses_.push(0);

  REQUIRE(device.handle(sygnal_test::mcm_relay_response(BUS, SUBSYSTEM, true)));

  CHECK_FALSE(is_ready(interface0));
  REQUIRE(is_ready(relay));
  CHECK(1.0 == relay.get().value);
}

TEST_CASE("MCM withdrawn response promise does not take the next response", "[mcm_device]")
{
  TestMcmDevice device(adapter(), BUS, SUBSYSTEM, COMMAND_TIMEOUT);
  auto withdrawn = device.control_enable_responses_.push(1);
  device.control_enable_responses_.withdraw_newest(1);
  auto next = device.control_enable_responses_.push(1);

  REQUIRE(device.handle(sygnal_test::mcm_enable_response(BUS, SUBSYSTEM, 1, true)));

  CHECK_THROWS_AS(withdrawn.get(), std::future_error);
  REQUIRE(is_ready(next));
}

TEST_CASE("MCM failed send leaves no pending reply", "[mcm_device]")
{
  TestMcmDevice device(adapter(), BUS, SUBSYSTEM, COMMAND_TIMEOUT);
  std::string error_message;

  const auto result = device.send_control_command(4, 0.5, true, error_message);

  CHECK_FALSE(result.success);
  CHECK_FALSE(result.response_future.has_value());
  CHECK_FALSE(error_message.empty());

  // The reply to the failed send must reach the next caller, not the withdrawn promise.
  auto next = device.control_command_responses_.push(4);
  REQUIRE(device.handle(sygnal_test::mcm_command_response(BUS, SUBSYSTEM, 4, 0.5)));
  CHECK(is_ready(next));
}

TEST_CASE("MCM device rejects an interface ID out of range", "[mcm_device]")
{
  TestMcmDevice device(adapter(), BUS, SUBSYSTEM, COMMAND_TIMEOUT);
  std::string error_message;

  CHECK_FALSE(device.send_control_state_command(6, SygnalControlState::MCM_CONTROL, false, error_message).success);
  CHECK_FALSE(device.send_control_command(6, 1.0, false, error_message).success);
  CHECK_FALSE(error_message.empty());
}
