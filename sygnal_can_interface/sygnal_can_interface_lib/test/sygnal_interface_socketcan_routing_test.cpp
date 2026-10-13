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
using polymath::sygnal::SygnalInterfaceSocketcan;
using polymath::sygnal::SygnalSystemState;

namespace
{

const SygnalControlDevice MCM_SUBSYSTEM_0{SygnalControlDeviceType::MCM, 1, 0};
const SygnalControlDevice MCM_SUBSYSTEM_1{SygnalControlDeviceType::MCM, 1, 1};
const SygnalControlDevice HPO{SygnalControlDeviceType::HPO, 3, 0};
const SygnalControlDevice UNCONFIGURED{SygnalControlDeviceType::MCM, 9, 0};

std::shared_ptr<polymath::socketcan::SocketcanAdapter> adapter()
{
  return std::make_shared<polymath::socketcan::SocketcanAdapter>("vcan0");
}

SygnalInterfaceSocketcan make_interface()
{
  return SygnalInterfaceSocketcan(adapter(), {MCM_SUBSYSTEM_0, MCM_SUBSYSTEM_1, HPO});
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
  CHECK_THROWS_AS(construct(Devices{{SygnalControlDeviceType::CB, 5, 0}}), std::invalid_argument);
  CHECK_NOTHROW(construct(Devices{MCM_SUBSYSTEM_0, MCM_SUBSYSTEM_1, HPO}));
}

TEST_CASE("SygnalInterfaceSocketcan routes frames by bus address and subsystem", "[socketcan_routing]")
{
  SygnalInterfaceSocketcan sygnal = make_interface();

  REQUIRE(sygnal.parse(CanFrame(sygnal_test::mcm_heartbeat(1, 1, SygnalSystemState::MCM_CONTROL))));
  CHECK(SygnalSystemState::FAIL_HARD == sygnal.get_device_system_state(MCM_SUBSYSTEM_0));
  CHECK(SygnalSystemState::MCM_CONTROL == sygnal.get_device_system_state(MCM_SUBSYSTEM_1));
  CHECK(SygnalSystemState::FAIL_HARD == sygnal.get_device_system_state(HPO));

  REQUIRE(sygnal.parse(CanFrame(sygnal_test::hpo_heartbeat(3, SygnalSystemState::HUMAN_CONTROL))));
  CHECK(SygnalSystemState::HUMAN_CONTROL == sygnal.get_device_system_state(HPO));

  CHECK_FALSE(sygnal.parse(CanFrame(sygnal_test::mcm_heartbeat(9, 0, SygnalSystemState::MCM_CONTROL))));
}

TEST_CASE("SygnalInterfaceSocketcan routes endpoint reads to the endpoint's device", "[socketcan_routing]")
{
  SygnalInterfaceSocketcan sygnal = make_interface();
  const SygnalEndpoint hpo_if{SygnalControlEndpointType::HPO_IF, HPO, 2, 0, 1, false, "hpo"};
  const SygnalEndpoint mcm_if{SygnalControlEndpointType::MCM_IF, MCM_SUBSYSTEM_0, 2, 0, 1, false, "mcm"};

  REQUIRE(sygnal.parse(CanFrame(sygnal_test::hpo_enable_response(3, 2, true))));

  CHECK(SygnalSystemState::MCM_CONTROL == sygnal.get_endpoint_system_state(hpo_if));
  CHECK(SygnalSystemState::HUMAN_CONTROL == sygnal.get_endpoint_system_state(mcm_if));
}

TEST_CASE("SygnalInterfaceSocketcan routes HPO frames regardless of bit 7", "[socketcan_routing]")
{
  SygnalInterfaceSocketcan sygnal = make_interface();
  const SygnalEndpoint hpo_if{SygnalControlEndpointType::HPO_IF, HPO, 1, 0, 1, false, "hpo"};

  can_frame frame = sygnal_test::hpo_enable_response(3, 1, true);
  frame.data[0] |= 0x80;
  frame.data[CAN_MAX_DLC - 1] = polymath::sygnal::generate_crc8(frame.data);

  REQUIRE(sygnal.parse(CanFrame(frame)));
  CHECK(SygnalSystemState::MCM_CONTROL == sygnal.get_endpoint_system_state(hpo_if));
}

TEST_CASE("SygnalInterfaceSocketcan serves faults only for MCMs", "[socketcan_routing]")
{
  SygnalInterfaceSocketcan sygnal = make_interface();
  REQUIRE(sygnal.parse(CanFrame(sygnal_test::mcm_fault_increment(1, 1, 0x32, 2))));

  const auto counts = sygnal.get_device_faults(MCM_SUBSYSTEM_1).counts;
  CHECK(2 == counts.at(McmFaultCause::CONTROL_TIMEOUT));  // NOLINT(readability/check)
  CHECK(sygnal.get_device_faults(MCM_SUBSYSTEM_0).counts.empty());
  CHECK_THROWS_AS(sygnal.get_device_faults(HPO), std::invalid_argument);
  CHECK_THROWS_AS(sygnal.get_device_faults(UNCONFIGURED), std::invalid_argument);
}

TEST_CASE("SygnalInterfaceSocketcan rejects unconfigured devices", "[socketcan_routing]")
{
  SygnalInterfaceSocketcan sygnal = make_interface();
  const SygnalEndpoint endpoint{SygnalControlEndpointType::MCM_IF, UNCONFIGURED, 0, 0, 1, false, "missing"};

  CHECK_THROWS_AS(sygnal.get_device_system_state(UNCONFIGURED), std::invalid_argument);
  CHECK_THROWS_AS(sygnal.get_endpoint_value(endpoint), std::invalid_argument);
  CHECK_THROWS_AS(sygnal.send_endpoint_state_command(endpoint, true, true), std::invalid_argument);
  CHECK_THROWS_AS(sygnal.send_endpoint_value_command(endpoint, 1.0, true), std::invalid_argument);
}
