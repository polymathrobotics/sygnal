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
//
// Covers the C++ API that dbc_gen_cpp adds on top of the cantools C output. The old
// aggregate sygnal_dbc library only ever exposed the C headers, so none of this was
// reachable before the per-subsystem split.

#if __has_include(<catch2/catch_all.hpp>)
  #include <catch2/catch_all.hpp>
#elif __has_include(<catch2/catch.hpp>)
  #include <catch2/catch.hpp>
#else
  #error "Catch2 headers not found. Please install Catch2 (v2 or v3)."
#endif

#include <cstring>

#include "mcm_heartbeat/mcm_heartbeat.hpp"

// Same subsystem-agnostic message name in four different DBCs -- see the collision
// test case below.
#include "cb_control/cb_control.hpp"
#include "hpo_control/hpo_control.hpp"
#include "io_control/io_control.hpp"
#include "mcm_control/mcm_control.hpp"

// The .hpp compatibility shim. Resolving to the same type as the canonical include
// above is the assertion; including both in one translation unit is the test.
#include "sygnal_dbc/mcm_control.hpp"

namespace
{
// The frame asserted by test_mcm_heartbeat.cpp / test_mcm_heartbeat.py.
can_frame make_heartbeat_frame()
{
  const uint8_t payload[8] = {0x03, 0x00, 0x00, 0x00, 0x00, 0xCE, 0x10, 0x2D};
  can_frame frame{};
  frame.can_id = mcm_heartbeat::Heartbeat::Id;
  frame.can_dlc = mcm_heartbeat::Heartbeat::DataLength;
  std::memcpy(frame.data, payload, sizeof(payload));
  return frame;
}
}  // namespace

TEST_CASE("C++ API decodes the same heartbeat frame as the C API")
{
  STATIC_REQUIRE(mcm_heartbeat::Heartbeat::Id == 0x170);
  STATIC_REQUIRE(mcm_heartbeat::Heartbeat::DataLength == 8);
  STATIC_REQUIRE(mcm_heartbeat::Heartbeat::IsExtendedFrame == false);

  const mcm_heartbeat::Heartbeat msg{make_heartbeat_frame()};

  REQUIRE(msg.bus_address == 3);
  REQUIRE(msg.subsystem_id == 0);
  REQUIRE(msg.system_state == 0);
  REQUIRE(msg.overall_interface_state == 0);
  REQUIRE(msg.interface0_state == 0);
  REQUIRE(msg.interface6_state == 0);
  REQUIRE(msg.count16 == 0x10CE);
  REQUIRE(msg.crc == 0x2D);
}

TEST_CASE("C++ API rejects a frame whose id belongs to another message")
{
  can_frame frame = make_heartbeat_frame();
  frame.can_id = mcm_heartbeat::Heartbeat::Id + 1;

  REQUIRE_THROWS_AS(mcm_heartbeat::Heartbeat{frame}, std::runtime_error);
}

TEST_CASE("C++ API round-trips a heartbeat through can_frame")
{
  mcm_heartbeat::Heartbeat original{};
  original.bus_address = 3;
  original.subsystem_id = 1;
  original.system_state = static_cast<double>(mcm_heartbeat::Heartbeat::SystemState::MCM_CONTROL);
  original.count16 = 0x10CE;
  original.crc = 0x2D;

  const can_frame frame = static_cast<can_frame>(original);
  REQUIRE(frame.can_id == mcm_heartbeat::Heartbeat::Id);

  const mcm_heartbeat::Heartbeat decoded{frame};
  REQUIRE(decoded.bus_address == original.bus_address);
  REQUIRE(decoded.subsystem_id == original.subsystem_id);
  REQUIRE(decoded.system_state == original.system_state);
  REQUIRE(decoded.count16 == original.count16);
  REQUIRE(decoded.crc == original.crc);
}

TEST_CASE("Generated value tables become C++ enums")
{
  using SystemState = mcm_heartbeat::Heartbeat::SystemState;

  STATIC_REQUIRE(static_cast<uint8_t>(SystemState::HUMAN_CONTROL) == 0);
  STATIC_REQUIRE(static_cast<uint8_t>(SystemState::MCM_CONTROL) == 1);
  STATIC_REQUIRE(static_cast<uint8_t>(SystemState::FAIL_OPERATIONAL_1) == 241);
  STATIC_REQUIRE(static_cast<uint8_t>(SystemState::FAIL_OPERATIONAL_2) == 242);
  STATIC_REQUIRE(static_cast<uint8_t>(SystemState::HUMAN_OVERRIDE) == 253);
  STATIC_REQUIRE(static_cast<uint8_t>(SystemState::FAIL_HARD) == 254);

  const mcm_heartbeat::Heartbeat msg{make_heartbeat_frame()};
  REQUIRE(static_cast<SystemState>(msg.system_state) == SystemState::HUMAN_CONTROL);
}

// Regression test for the reason dbc_package() requires PREFIX. cb/, hpo/, io/ and mcm/
// each ship a Control.dbc declaring a ControlCommand at id 352. Without the prefix all
// four would generate a library named "control" defining control_* symbols, and this
// translation unit would fail to compile (redefinition) or fail to link (duplicate
// symbols). Encoding through each one forces the linker to resolve all four packs.
TEST_CASE("Identically-named messages in sibling subsystems stay distinct")
{
  STATIC_REQUIRE(mcm_control::ControlCommand::Id == 352);
  STATIC_REQUIRE(io_control::ControlCommand::Id == 352);
  STATIC_REQUIRE(cb_control::ControlCommand::Id == 352);
  STATIC_REQUIRE(hpo_control::ControlCommand::Id == 352);

  STATIC_REQUIRE_FALSE(std::is_same_v<mcm_control::ControlCommand, io_control::ControlCommand>);
  STATIC_REQUIRE_FALSE(std::is_same_v<mcm_control::ControlCommand, cb_control::ControlCommand>);
  STATIC_REQUIRE_FALSE(std::is_same_v<mcm_control::ControlCommand, hpo_control::ControlCommand>);

  const can_frame from_mcm = static_cast<can_frame>(mcm_control::ControlCommand{});
  const can_frame from_io = static_cast<can_frame>(io_control::ControlCommand{});
  const can_frame from_cb = static_cast<can_frame>(cb_control::ControlCommand{});
  const can_frame from_hpo = static_cast<can_frame>(hpo_control::ControlCommand{});

  REQUIRE(from_mcm.can_id == mcm_control::ControlCommand::Id);
  REQUIRE(from_io.can_id == io_control::ControlCommand::Id);
  REQUIRE(from_cb.can_id == cb_control::ControlCommand::Id);
  REQUIRE(from_hpo.can_id == hpo_control::ControlCommand::Id);
}
