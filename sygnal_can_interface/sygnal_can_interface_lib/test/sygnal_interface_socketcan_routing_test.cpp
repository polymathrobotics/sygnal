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

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
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
#include "sygnal_can_interface_lib/crc8.hpp"
#include "sygnal_can_interface_lib/sygnal_interface_socketcan.hpp"
#include "sygnal_dbc/hpo_control.h"
#include "sygnal_dbc/hpo_heartbeat.h"
#include "sygnal_dbc/mcm_control.h"
#include "sygnal_dbc/mcm_heartbeat.h"

namespace
{

constexpr uint8_t MCM_BUS = 1;
constexpr uint8_t HPO_BUS = 3;

polymath::socketcan::CanFrame makeFrame(uint32_t can_id, const uint8_t * buffer, uint8_t len)
{
  polymath::socketcan::CanFrame frame;
  frame.set_can_id(can_id);
  std::array<unsigned char, CAN_MAX_DLC> data;
  data.fill(0);
  for (size_t i = 0; i < len && i < CAN_MAX_DLC; ++i) {
    data[i] = buffer[i];
  }
  frame.set_data(data);
  frame.set_len(len);
  return frame;
}

polymath::socketcan::CanFrame buildHpoHeartbeat(uint8_t bus_address, bool overall_state)
{
  hpo_heartbeat_heartbeat_t msg;
  hpo_heartbeat_heartbeat_init(&msg);
  msg.bus_address = bus_address;
  msg.subsystem_id = 0;
  msg.system_state = 0;
  msg.interface0_state = 1;
  msg.interface1_state = 0;
  msg.interface2_state = 1;
  msg.interface3_state = 0;
  msg.interface4_state = 1;
  msg.overall_interface_state = overall_state ? 1 : 0;
  msg.count16 = 0;
  msg.crc = 0;

  uint8_t buffer[CAN_MAX_DLC];
  hpo_heartbeat_heartbeat_pack(buffer, &msg, sizeof(buffer));
  msg.crc = polymath::sygnal::generate_crc8(buffer);
  hpo_heartbeat_heartbeat_pack(buffer, &msg, sizeof(buffer));
  return makeFrame(HPO_HEARTBEAT_HEARTBEAT_FRAME_ID, buffer, HPO_HEARTBEAT_HEARTBEAT_LENGTH);
}

polymath::socketcan::CanFrame buildHpoControlEnableResponse(uint8_t bus_address, uint8_t message_id, bool enable)
{
  hpo_control_control_enable_response_t msg;
  hpo_control_control_enable_response_init(&msg);
  msg.bus_address = bus_address;
  msg.message_id = message_id;
  msg.enable = enable ? 1 : 0;
  msg.crc = 0;

  uint8_t buffer[CAN_MAX_DLC];
  hpo_control_control_enable_response_pack(buffer, &msg, sizeof(buffer));
  msg.crc = polymath::sygnal::generate_crc8(buffer);
  hpo_control_control_enable_response_pack(buffer, &msg, sizeof(buffer));
  return makeFrame(HPO_CONTROL_CONTROL_ENABLE_RESPONSE_FRAME_ID, buffer, HPO_CONTROL_CONTROL_ENABLE_RESPONSE_LENGTH);
}

polymath::socketcan::CanFrame buildMcmHeartbeat(uint8_t bus_address, uint8_t subsystem_id, uint8_t system_state)
{
  mcm_heartbeat_heartbeat_t msg;
  mcm_heartbeat_heartbeat_init(&msg);
  msg.bus_address = bus_address;
  msg.subsystem_id = subsystem_id;
  msg.system_state = system_state;
  msg.interface0_state = 1;
  msg.interface1_state = 1;
  msg.interface2_state = 1;
  msg.interface3_state = 1;
  msg.interface4_state = 1;
  msg.interface5_state = 0;
  msg.interface6_state = 0;
  msg.overall_interface_state = 0;
  msg.count16 = 0;
  msg.crc = 0;

  uint8_t buffer[CAN_MAX_DLC];
  mcm_heartbeat_heartbeat_pack(buffer, &msg, sizeof(buffer));
  msg.crc = polymath::sygnal::generate_crc8(buffer);
  mcm_heartbeat_heartbeat_pack(buffer, &msg, sizeof(buffer));
  return makeFrame(MCM_HEARTBEAT_HEARTBEAT_FRAME_ID, buffer, MCM_HEARTBEAT_HEARTBEAT_LENGTH);
}

// SocketcanAdapter constructor stores config only; no socket is opened until openSocket() is called.
// We never open a socket in this file, so these tests work without CAN being available.
std::shared_ptr<polymath::socketcan::SocketcanAdapter> makeUnopenedAdapter()
{
  return std::make_shared<polymath::socketcan::SocketcanAdapter>("vcan0");
}

}  // namespace

TEST_CASE("Constructor throws when an HPO and MCM share a bus address", "[sygnal_socketcan_routing]")
{
  auto adapter = makeUnopenedAdapter();
  std::vector<polymath::sygnal::McmId> mcms{{1, 0}, {2, 0}};
  std::vector<polymath::sygnal::HpoId> hpos{{2}};  // collides with second MCM

  REQUIRE_THROWS_AS(polymath::sygnal::SygnalInterfaceSocketcan(adapter, mcms, hpos), std::invalid_argument);
}

TEST_CASE("Constructor with empty hpo_ids preserves legacy behavior", "[sygnal_socketcan_routing]")
{
  auto adapter = makeUnopenedAdapter();
  std::vector<polymath::sygnal::McmId> mcms{{1, 0}, {1, 1}};

  // Two-argument form (legacy) must still compile and not throw.
  REQUIRE_NOTHROW(polymath::sygnal::SygnalInterfaceSocketcan(adapter, mcms));

  // Three-argument form with empty HPO list must also work and behave identically.
  REQUIRE_NOTHROW(polymath::sygnal::SygnalInterfaceSocketcan(adapter, mcms, {}));
}

TEST_CASE("parse() routes HPO heartbeat to HPO state and leaves MCM untouched", "[sygnal_socketcan_routing]")
{
  auto adapter = makeUnopenedAdapter();
  std::vector<polymath::sygnal::McmId> mcms{{MCM_BUS, 0}};
  std::vector<polymath::sygnal::HpoId> hpos{{HPO_BUS}};

  polymath::sygnal::SygnalInterfaceSocketcan sygnal(adapter, mcms, hpos);

  auto frame = buildHpoHeartbeat(HPO_BUS, /*overall_state=*/true);
  REQUIRE(sygnal.parse(frame));

  auto hpo_state = sygnal.get_hpo_overall_interface_state(HPO_BUS);
  REQUIRE(hpo_state.has_value());
  REQUIRE(hpo_state.value());

  auto hpo_interfaces = sygnal.get_hpo_interface_states(HPO_BUS);
  REQUIRE(hpo_interfaces.has_value());
  REQUIRE((*hpo_interfaces)[0]);
  REQUIRE_FALSE((*hpo_interfaces)[1]);
  REQUIRE((*hpo_interfaces)[2]);
  REQUIRE_FALSE((*hpo_interfaces)[3]);
  REQUIRE((*hpo_interfaces)[4]);

  // MCM state untouched: still default FAIL_HARD.
  auto mcm_state = sygnal.get_sygnal_mcm_state(MCM_BUS, 0);
  REQUIRE(mcm_state.has_value());
  REQUIRE(mcm_state.value() == polymath::sygnal::SygnalSystemState::FAIL_HARD);
}

TEST_CASE("parse() routes MCM heartbeat to MCM state and leaves HPO untouched", "[sygnal_socketcan_routing]")
{
  auto adapter = makeUnopenedAdapter();
  std::vector<polymath::sygnal::McmId> mcms{{MCM_BUS, 0}};
  std::vector<polymath::sygnal::HpoId> hpos{{HPO_BUS}};

  polymath::sygnal::SygnalInterfaceSocketcan sygnal(adapter, mcms, hpos);

  auto frame = buildMcmHeartbeat(MCM_BUS, 0, static_cast<uint8_t>(polymath::sygnal::SygnalSystemState::MCM_CONTROL));
  REQUIRE(sygnal.parse(frame));

  auto mcm_state = sygnal.get_sygnal_mcm_state(MCM_BUS, 0);
  REQUIRE(mcm_state.has_value());
  REQUIRE(mcm_state.value() == polymath::sygnal::SygnalSystemState::MCM_CONTROL);

  // HPO overall interface state stays at the default false.
  auto hpo_state = sygnal.get_hpo_overall_interface_state(HPO_BUS);
  REQUIRE(hpo_state.has_value());
  REQUIRE_FALSE(hpo_state.value());
}

TEST_CASE(
  "parse() routes HPO ControlEnableResponse without disturbing MCM promise queues", "[sygnal_socketcan_routing]")
{
  // This test guards against the central invariant of the tactical mitigation: an HPO control response
  // arrives on a CAN ID (0x61) that the MCM control-response parser also accepts. The HPO parser MUST claim
  // it first so the MCM-side parser never sees it. We verify by issuing an HPO ControlEnable (which arms the
  // HPO enable promise queue), then feeding back an HPO ControlEnableResponse via parse(), and asserting the
  // HPO future resolves — implying the MCM-side parser did not consume the frame.
  auto adapter = makeUnopenedAdapter();
  std::vector<polymath::sygnal::McmId> mcms{{MCM_BUS, 0}};
  std::vector<polymath::sygnal::HpoId> hpos{{HPO_BUS}};

  polymath::sygnal::SygnalInterfaceSocketcan sygnal(adapter, mcms, hpos);

  // Arm the HPO enable promise queue directly by sending an HPO ControlEnable. The actual socketcan_send
  // will fail (we never opened the socket), but the promise gets pushed before the send call, so we can
  // still observe the future. Wrap in a scope and discard the send result.
  std::string err;
  std::optional<std::future<polymath::sygnal::HpoControlResponse>> fut_opt;
  {
    auto result =
      sygnal.sendHpoControlEnable(HPO_BUS, /*message_id=*/0x42, /*enable=*/true, /*expect_reply=*/true, err);
    // The send itself fails because no socket is open; that's fine for this routing test — the promise was
    // already enqueued before the send attempt, so the future is still valid.
    fut_opt = std::move(result.response_future);
  }
  REQUIRE(fut_opt.has_value());

  // Now deliver a matching response via parse().
  auto frame = buildHpoControlEnableResponse(HPO_BUS, /*message_id=*/0x42, /*enable=*/true);
  REQUIRE(sygnal.parse(frame));

  // The HPO future must be ready immediately.
  REQUIRE(fut_opt->wait_for(std::chrono::seconds(0)) == std::future_status::ready);
  auto response = fut_opt->get();
  REQUIRE(response.is_enable_response);
  REQUIRE(response.bus_address == HPO_BUS);
  REQUIRE(response.message_id == 0x42);
  REQUIRE(response.enable);
}

TEST_CASE("sendHpoControlEnable to an unregistered bus address fails cleanly", "[sygnal_socketcan_routing]")
{
  auto adapter = makeUnopenedAdapter();
  std::vector<polymath::sygnal::McmId> mcms{{MCM_BUS, 0}};
  std::vector<polymath::sygnal::HpoId> hpos{{HPO_BUS}};

  polymath::sygnal::SygnalInterfaceSocketcan sygnal(adapter, mcms, hpos);

  std::string err;
  auto result = sygnal.sendHpoControlEnable(/*bus_id=*/99, 0x10, true, false, err);
  REQUIRE_FALSE(result.success);
  REQUIRE_FALSE(result.response_future.has_value());
  REQUIRE_FALSE(err.empty());
}
