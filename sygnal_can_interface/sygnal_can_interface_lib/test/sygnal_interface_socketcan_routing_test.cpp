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

// Always-on tests that exercise SygnalInterfaceSocketcan logic that does NOT require a real CAN interface:
// constructor invariants and the parse() routing rules between HPO and MCM. End-to-end vcan tests live in
// sygnal_interface_socketcan_test.cpp.

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#if __has_include(<catch2/catch_all.hpp>)
  #include <catch2/catch_all.hpp>
#elif __has_include(<catch2/catch.hpp>)
  #include <catch2/catch.hpp>
#else
  #error "Catch2 headers not found. Please install Catch2 (v2 or v3)."
#endif

#include "socketcan_adapter/can_frame.hpp"
#include "socketcan_adapter/socketcan_adapter.hpp"
#include "sygnal_can_interface_lib/sygnal_interface_socketcan.hpp"
#include "sygnal_test_frames.hpp"  // NOLINT(build/include_subdir)

using polymath::socketcan::CanFrame;
using polymath::sygnal::McmFaultCause;
using polymath::sygnal::SygnalControlDevice;
using polymath::sygnal::SygnalControlDeviceType;
using polymath::sygnal::SygnalControlEndpointType;
using polymath::sygnal::SygnalEndpoint;
using polymath::sygnal::SygnalErrorType;
using polymath::sygnal::SygnalInterfaceSocketcan;
using polymath::sygnal::SygnalSystemState;

namespace
{

const SygnalControlDevice MCM_SUBSYSTEM_0{SygnalControlDeviceType::MCM, 1, 0, "mcm_0"};
const SygnalControlDevice MCM_SUBSYSTEM_1{SygnalControlDeviceType::MCM, 1, 1, "mcm_1"};
const SygnalControlDevice HPO{SygnalControlDeviceType::HPO, 3, 0, "hpo"};
const SygnalControlDevice CB{SygnalControlDeviceType::CB, 5, 0, "cb"};
const SygnalControlDevice IO{SygnalControlDeviceType::IO, 6, 0, "io"};
const SygnalControlDevice UNCONFIGURED{SygnalControlDeviceType::MCM, 9, 0, "missing"};

std::shared_ptr<polymath::socketcan::SocketcanAdapter> adapter()
{
  return std::make_shared<polymath::socketcan::SocketcanAdapter>("vcan0");
}

SygnalInterfaceSocketcan make_interface()
{
  return SygnalInterfaceSocketcan(adapter(), {MCM_SUBSYSTEM_0, MCM_SUBSYSTEM_1, HPO, CB, IO});
}

}  // namespace

TEST_CASE("SygnalInterfaceSocketcan rejects invalid device lists", "[socketcan_routing]")
{
  using Devices = std::vector<SygnalControlDevice>;
  const auto construct = [](const Devices & devices) { SygnalInterfaceSocketcan(adapter(), devices); };

  CHECK_THROWS_AS(construct(Devices{MCM_SUBSYSTEM_0, MCM_SUBSYSTEM_0}), std::invalid_argument);
  CHECK_THROWS_AS(construct(Devices{{SygnalControlDeviceType::MCM, 128, 0}}), std::invalid_argument);
  CHECK_THROWS_AS(construct(Devices{{SygnalControlDeviceType::MCM, 1, 2}}), std::invalid_argument);
  CHECK_THROWS_AS(construct(Devices{HPO, {SygnalControlDeviceType::HPO, 3, 1}}), std::invalid_argument);
  CHECK_THROWS_AS(construct(Devices{{SygnalControlDeviceType::HPO, 1, 0}, MCM_SUBSYSTEM_1}), std::invalid_argument);
  CHECK_THROWS_AS(construct(Devices{MCM_SUBSYSTEM_1, {SygnalControlDeviceType::HPO, 1, 0}}), std::invalid_argument);
  CHECK_NOTHROW(construct(Devices{MCM_SUBSYSTEM_0, MCM_SUBSYSTEM_1, HPO}));
}

TEST_CASE("SygnalInterfaceSocketcan routes frames by bus address and subsystem", "[socketcan_routing]")
{
  SygnalInterfaceSocketcan sygnal = make_interface();

  REQUIRE(sygnal.parse(CanFrame(sygnal_test::mcm_heartbeat(1, 1, SygnalSystemState::MCM_CONTROL))));
  CHECK(SygnalSystemState::FAIL_HARD == sygnal.get_device_system_state(MCM_SUBSYSTEM_0).value);
  CHECK(SygnalSystemState::MCM_CONTROL == sygnal.get_device_system_state(MCM_SUBSYSTEM_1).value);
  CHECK(SygnalSystemState::FAIL_HARD == sygnal.get_device_system_state(HPO).value);

  REQUIRE(sygnal.parse(CanFrame(sygnal_test::hpo_heartbeat(3, SygnalSystemState::HUMAN_CONTROL))));
  CHECK(SygnalSystemState::HUMAN_CONTROL == sygnal.get_device_system_state(HPO).value);

  CHECK_FALSE(sygnal.parse(CanFrame(sygnal_test::mcm_heartbeat(9, 0, SygnalSystemState::MCM_CONTROL))));
}

TEST_CASE("SygnalInterfaceSocketcan routes endpoint reads to the endpoint's device", "[socketcan_routing]")
{
  SygnalInterfaceSocketcan sygnal = make_interface();
  const SygnalEndpoint hpo_if{SygnalControlEndpointType::HPO_INTERFACE, HPO, 2, 0, 1, false, "hpo"};
  const SygnalEndpoint mcm_if{SygnalControlEndpointType::MCM_INTERFACE, MCM_SUBSYSTEM_0, 2, 0, 1, false, "mcm"};

  REQUIRE(sygnal.parse(CanFrame(sygnal_test::hpo_enable_response(3, 2, true))));

  CHECK(SygnalSystemState::MCM_CONTROL == sygnal.get_interface_control_state(hpo_if).value);
  CHECK(SygnalSystemState::HUMAN_CONTROL == sygnal.get_interface_control_state(mcm_if).value);
}

TEST_CASE("SygnalInterfaceSocketcan routes HPO frames regardless of bit 7", "[socketcan_routing]")
{
  SygnalInterfaceSocketcan sygnal = make_interface();
  const SygnalEndpoint hpo_if{SygnalControlEndpointType::HPO_INTERFACE, HPO, 1, 0, 1, false, "hpo"};

  can_frame frame = sygnal_test::hpo_enable_response(3, 1, true);
  frame.data[0] |= 0x80;
  frame.data[CAN_MAX_DLC - 1] = polymath::sygnal::generate_crc8(frame.data);

  REQUIRE(sygnal.parse(CanFrame(frame)));
  CHECK(SygnalSystemState::MCM_CONTROL == sygnal.get_interface_control_state(hpo_if).value);
}

TEST_CASE("SygnalInterfaceSocketcan routes CB frames regardless of bit 7", "[socketcan_routing]")
{
  SygnalInterfaceSocketcan sygnal = make_interface();
  const SygnalEndpoint cb_if{SygnalControlEndpointType::CB_INTERFACE, CB, 6, 0, 1, true, "cb"};

  can_frame frame = sygnal_test::cb_command_response(5, 6, 1.0);
  frame.data[0] |= 0x80;
  frame.data[CAN_MAX_DLC - 1] = polymath::sygnal::generate_crc8(frame.data);

  REQUIRE(sygnal.parse(CanFrame(frame)));
  CHECK(1.0 == sygnal.get_control_command_value(cb_if).value);
}

TEST_CASE("SygnalInterfaceSocketcan routes IO frames regardless of bit 7", "[socketcan_routing]")
{
  SygnalInterfaceSocketcan sygnal = make_interface();
  const SygnalEndpoint io_if{SygnalControlEndpointType::IO_INTERFACE, IO, 2, 0, 1, false, "io"};

  can_frame frame = sygnal_test::io_enable_response(6, 2, true);
  frame.data[0] |= 0x80;
  frame.data[CAN_MAX_DLC - 1] = polymath::sygnal::generate_crc8(frame.data);

  REQUIRE(sygnal.parse(CanFrame(frame)));
  CHECK(SygnalSystemState::MCM_CONTROL == sygnal.get_interface_control_state(io_if).value);
}

TEST_CASE("SygnalInterfaceSocketcan passes the frame's bus time to the getters", "[socketcan_routing]")
{
  SygnalInterfaceSocketcan sygnal = make_interface();
  const std::chrono::system_clock::time_point bus_time{std::chrono::milliseconds(1234)};

  CanFrame frame(sygnal_test::mcm_heartbeat(1, 0, SygnalSystemState::HUMAN_CONTROL));
  frame.set_bus_timestamp(bus_time);
  REQUIRE(sygnal.parse(frame));

  CHECK(bus_time == sygnal.get_device_system_state(MCM_SUBSYSTEM_0).timestamp);
  CHECK(std::chrono::system_clock::time_point{} == sygnal.get_device_system_state(MCM_SUBSYSTEM_1).timestamp);
}

TEST_CASE("SygnalInterfaceSocketcan serves faults only for MCMs", "[socketcan_routing]")
{
  SygnalInterfaceSocketcan sygnal = make_interface();
  REQUIRE(sygnal.parse(CanFrame(sygnal_test::mcm_fault_increment(1, 1, 0x32, 2))));

  const auto counts = sygnal.get_device_faults(MCM_SUBSYSTEM_1).value.counts;
  CHECK(2 == counts.at(McmFaultCause::CONTROL_TIMEOUT));  // NOLINT(readability/check)
  CHECK(sygnal.get_device_faults(MCM_SUBSYSTEM_0).value.counts.empty());
  CHECK_THROWS_AS(sygnal.get_device_faults(HPO), std::invalid_argument);
  CHECK_THROWS_AS(sygnal.get_device_faults(UNCONFIGURED), std::invalid_argument);
}

TEST_CASE("SygnalInterfaceSocketcan rejects unconfigured devices", "[socketcan_routing]")
{
  SygnalInterfaceSocketcan sygnal = make_interface();
  const SygnalEndpoint endpoint{SygnalControlEndpointType::MCM_INTERFACE, UNCONFIGURED, 0, 0, 1, false, "missing"};

  CHECK_THROWS_AS(sygnal.get_device_system_state(UNCONFIGURED), std::invalid_argument);
  CHECK_THROWS_AS(sygnal.get_control_command_value(endpoint), std::invalid_argument);
  CHECK_THROWS_AS(sygnal.send_control_enable_command(endpoint, true, true), std::invalid_argument);
  CHECK_THROWS_AS(sygnal.send_control_command(endpoint, 1.0, true), std::invalid_argument);
}

TEST_CASE("SygnalInterfaceSocketcan rejects a descriptor whose type differs from the configured device", "[socketcan_routing]")
{
  SygnalInterfaceSocketcan sygnal = make_interface();
  const SygnalControlDevice mcm_at_hpo_address{SygnalControlDeviceType::MCM, 3, 0, "wrong_type"};

  CHECK_THROWS_AS(sygnal.get_device_system_state(mcm_at_hpo_address), std::invalid_argument);
}

TEST_CASE("SygnalInterfaceSocketcan returns new errors from the addressed device only", "[socketcan_routing]")
{
  SygnalInterfaceSocketcan sygnal = make_interface();
  REQUIRE(sygnal.parse(
    CanFrame(sygnal_test::error_status<mcm_error::ErrorStatus>(1, 1, SygnalErrorType::KEY, 0x7F))));

  CHECK(sygnal.get_new_device_errors(MCM_SUBSYSTEM_0).empty());
  CHECK(sygnal.get_new_device_errors(HPO).empty());
  const auto errors = sygnal.get_new_device_errors(MCM_SUBSYSTEM_1);
  REQUIRE(1 == errors.size());  // NOLINT(readability/check)
  CHECK(SygnalErrorType::KEY == errors[0].value.type);
  CHECK_THROWS_AS(sygnal.get_new_device_errors(UNCONFIGURED), std::invalid_argument);
}
