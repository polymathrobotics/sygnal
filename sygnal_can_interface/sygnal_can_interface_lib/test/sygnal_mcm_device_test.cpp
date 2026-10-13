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

using polymath::sygnal::McmFaultCause;
using polymath::sygnal::SygnalControlDeviceType;
using polymath::sygnal::SygnalControlEndpointType;
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

constexpr uint8_t BUS = 1;
constexpr uint8_t SUBSYSTEM = 1;

SygnalEndpoint endpoint(SygnalControlEndpointType type, uint8_t id)
{
  return {type, {SygnalControlDeviceType::MCM, BUS, SUBSYSTEM}, id, -1, 1, false, "test"};
}

SygnalEndpoint mcm_if(uint8_t id)
{
  return endpoint(SygnalControlEndpointType::MCM_IF, id);
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
  SygnalMcmDevice device(adapter(), BUS, SUBSYSTEM);
  CHECK(SygnalSystemState::FAIL_HARD == device.get_system_state());
}

TEST_CASE("MCM heartbeat sets system and interface state", "[mcm_device]")
{
  SygnalMcmDevice device(adapter(), BUS, SUBSYSTEM);
  REQUIRE(device.handle(sygnal_test::mcm_heartbeat(BUS, SUBSYSTEM, SygnalSystemState::MCM_CONTROL, {false, true})));

  CHECK(SygnalSystemState::MCM_CONTROL == device.get_system_state());
  CHECK(SygnalSystemState::HUMAN_CONTROL == device.get_endpoint_state(mcm_if(0)));
  CHECK(SygnalSystemState::MCM_CONTROL == device.get_endpoint_state(mcm_if(1)));
}

TEST_CASE("MCM device rejects a frame with a bad CRC", "[mcm_device]")
{
  SygnalMcmDevice device(adapter(), BUS, SUBSYSTEM);
  can_frame frame = sygnal_test::mcm_heartbeat(BUS, SUBSYSTEM, SygnalSystemState::MCM_CONTROL);
  frame.data[CAN_MAX_DLC - 1] ^= 0xFF;

  CHECK_FALSE(device.handle(frame));
  CHECK(SygnalSystemState::FAIL_HARD == device.get_system_state());
}

TEST_CASE("MCM enable response fulfills only its interface's future", "[mcm_device]")
{
  SygnalMcmDevice device(adapter(), BUS, SUBSYSTEM);
  auto interface0 = device.expect_endpoint_state_response(mcm_if(0));
  auto interface2 = device.expect_endpoint_state_response(mcm_if(2));

  REQUIRE(device.handle(sygnal_test::mcm_enable_response(BUS, SUBSYSTEM, 2, true)));

  CHECK_FALSE(is_ready(interface0));
  REQUIRE(is_ready(interface2));
  CHECK(SygnalSystemState::MCM_CONTROL == interface2.get());
  CHECK(SygnalSystemState::MCM_CONTROL == device.get_endpoint_state(mcm_if(2)));
}

TEST_CASE("MCM cancelled response promise does not take the next response", "[mcm_device]")
{
  SygnalMcmDevice device(adapter(), BUS, SUBSYSTEM);
  auto cancelled = device.expect_endpoint_state_response(mcm_if(1));
  device.cancel_endpoint_state_response(mcm_if(1));
  auto next = device.expect_endpoint_state_response(mcm_if(1));

  REQUIRE(device.handle(sygnal_test::mcm_enable_response(BUS, SUBSYSTEM, 1, true)));

  CHECK_THROWS_AS(cancelled.get(), std::future_error);
  REQUIRE(is_ready(next));
  CHECK(SygnalSystemState::MCM_CONTROL == next.get());
}

TEST_CASE("MCM command response fulfills its future and caches the value", "[mcm_device]")
{
  SygnalMcmDevice device(adapter(), BUS, SUBSYSTEM);
  auto future = device.expect_endpoint_value_response(mcm_if(3));

  REQUIRE(device.handle(sygnal_test::mcm_command_response(BUS, SUBSYSTEM, 3, 0.25)));

  REQUIRE(is_ready(future));
  CHECK(0.25 == future.get());
  CHECK(0.25 == device.get_endpoint_value(mcm_if(3)));
  CHECK(0.0 == device.get_endpoint_value(mcm_if(0)));
}

TEST_CASE("MCM relay response fulfills the relay future", "[mcm_device]")
{
  SygnalMcmDevice device(adapter(), BUS, SUBSYSTEM);
  auto future = device.expect_endpoint_value_response(relay());

  REQUIRE(device.handle(sygnal_test::mcm_relay_response(BUS, SUBSYSTEM, true)));

  REQUIRE(is_ready(future));
  CHECK(1.0 == future.get());
  CHECK(1.0 == device.get_endpoint_value(relay()));
}

TEST_CASE("MCM fault messages populate the fault status", "[mcm_device]")
{
  SygnalMcmDevice device(adapter(), BUS, SUBSYSTEM);
  REQUIRE(device.handle(sygnal_test::mcm_fault_state(BUS, SUBSYSTEM, SygnalSystemState::FAIL_HARD, 0x01)));
  REQUIRE(device.handle(sygnal_test::mcm_fault_increment(BUS, SUBSYSTEM, 0x32, 3)));
  REQUIRE(device.handle(sygnal_test::mcm_fault_list(BUS, SUBSYSTEM, 0x33, 4)));
  REQUIRE(device.handle(sygnal_test::mcm_fault_root_cause(BUS, SUBSYSTEM, 0x32, 0x00, 0x01)));

  const auto faults = device.get_faults();
  CHECK(SygnalSystemState::FAIL_HARD == faults.system_state);
  CHECK(McmFaultCause::EMERGENCY_STOP == faults.state_cause);
  CHECK(McmFaultCause::CONTROL_TIMEOUT == faults.fail_op1_cause);
  CHECK(McmFaultCause::NONE == faults.fail_op2_cause);
  CHECK(McmFaultCause::EMERGENCY_STOP == faults.fail_hard_cause);
  CHECK(3 == faults.counts.at(McmFaultCause::CONTROL_TIMEOUT));  // NOLINT(readability/check)
  CHECK(4 == faults.counts.at(McmFaultCause::CONTROL_COUNT));  // NOLINT(readability/check)
}

TEST_CASE("MCM fault counts take the latest absolute count", "[mcm_device]")
{
  SygnalMcmDevice device(adapter(), BUS, SUBSYSTEM);
  REQUIRE(device.handle(sygnal_test::mcm_fault_increment(BUS, SUBSYSTEM, 0x32, 5)));
  REQUIRE(device.handle(sygnal_test::mcm_fault_list(BUS, SUBSYSTEM, 0x32, 5)));

  CHECK(5 == device.get_faults().counts.at(McmFaultCause::CONTROL_TIMEOUT));  // NOLINT(readability/check)
}

TEST_CASE("MCM faults clear only when a heartbeat leaves a fail state", "[mcm_device]")
{
  SygnalMcmDevice device(adapter(), BUS, SUBSYSTEM);
  REQUIRE(device.handle(sygnal_test::mcm_heartbeat(BUS, SUBSYSTEM, SygnalSystemState::MCM_CONTROL)));
  REQUIRE(device.handle(sygnal_test::mcm_fault_increment(BUS, SUBSYSTEM, 0x33, 1)));

  SECTION("a healthy heartbeat keeps sub-threshold counts")
  {
    REQUIRE(device.handle(sygnal_test::mcm_heartbeat(BUS, SUBSYSTEM, SygnalSystemState::MCM_CONTROL)));
    CHECK(1 == device.get_faults().counts.at(McmFaultCause::CONTROL_COUNT));  // NOLINT(readability/check)
  }

  SECTION("a fail state followed by human control clears everything")
  {
    REQUIRE(device.handle(sygnal_test::mcm_fault_state(BUS, SUBSYSTEM, SygnalSystemState::FAIL_HARD, 0x01)));
    REQUIRE(device.handle(sygnal_test::mcm_heartbeat(BUS, SUBSYSTEM, SygnalSystemState::FAIL_HARD)));
    CHECK(McmFaultCause::EMERGENCY_STOP == device.get_faults().state_cause);

    REQUIRE(device.handle(sygnal_test::mcm_heartbeat(BUS, SUBSYSTEM, SygnalSystemState::HUMAN_CONTROL)));
    const auto faults = device.get_faults();
    CHECK(SygnalSystemState::HUMAN_CONTROL == faults.system_state);
    CHECK(McmFaultCause::NONE == faults.state_cause);
    CHECK(faults.counts.empty());
  }
}

TEST_CASE("MCM device rejects endpoints and values it does not have", "[mcm_device]")
{
  SygnalMcmDevice device(adapter(), BUS, SUBSYSTEM);
  const auto hpo_if = endpoint(SygnalControlEndpointType::HPO_IF, 0);

  for (const auto & result :
       {device.send_endpoint_state_command(relay(), true, false),
        device.send_endpoint_state_command(mcm_if(7), true, false),
        device.send_endpoint_state_command(hpo_if, true, false)})
  {
    CHECK_FALSE(result.success);
    CHECK_FALSE(result.error_message.empty());
  }
  const auto value_result = device.send_endpoint_value_command(hpo_if, 1.0, false);
  CHECK_FALSE(value_result.success);
  CHECK_FALSE(value_result.error_message.empty());

  CHECK_THROWS_AS(device.get_endpoint_state(relay()), std::invalid_argument);
  CHECK_THROWS_AS(device.get_endpoint_state(mcm_if(7)), std::invalid_argument);
  CHECK_THROWS_AS(device.get_endpoint_value(endpoint(SygnalControlEndpointType::HPO_IF, 0)), std::invalid_argument);
}
