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

// Define a Catch2 main to avoid linking any extra library

#if __has_include(<catch2/catch_all.hpp>)
  #include <catch2/catch_all.hpp>
#elif __has_include(<catch2/catch.hpp>)
  #include <catch2/catch.hpp>
#else
  #error "Catch2 headers not found. Please install Catch2 (v2 or v3)."
#endif

extern "C"
{
#include "sygnal_dbc/mcm_heartbeat.h"
}

TEST_CASE("MCM Heartbeat unpack from frame 0x170")
{
  const uint8_t payload[8] = {0x03, 0x00, 0x00, 0x00, 0x00, 0xCE, 0x10, 0x2D};

  struct mcm_heartbeat_heartbeat_t msg;

  // The cantools-generated API typically uses _unpack for raw bytes -> struct.
  const int ret = mcm_heartbeat_heartbeat_unpack(&msg, payload, sizeof(payload));
  REQUIRE(ret >= 0);

  // Basic field checks from the DBC and Python test
  REQUIRE(msg.bus_address == 3);
  REQUIRE(msg.subsystem_id == 0);

  REQUIRE(msg.system_state == 0);
  REQUIRE(msg.overall_interface_state == 0);
  REQUIRE(msg.interface0_state == 0);
  REQUIRE(msg.interface1_state == 0);
  REQUIRE(msg.interface2_state == 0);
  REQUIRE(msg.interface3_state == 0);
  REQUIRE(msg.interface4_state == 0);
  REQUIRE(msg.interface5_state == 0);
  REQUIRE(msg.interface6_state == 0);

  // Note: generator yields 0x10CE for this field
  REQUIRE(msg.count16 == 0x10CE);
  REQUIRE(msg.crc == 0x2D);
}
