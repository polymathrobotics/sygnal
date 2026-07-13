// Copyright (c) 2026-present Polymath Robotics, Inc. All rights reserved
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

#include <array>

#include "sygnal_can_interface_lib/sygnal_mcm_interface.hpp"

#if __has_include(<catch2/catch_all.hpp>)
  #include <catch2/catch_all.hpp>
#elif __has_include(<catch2/catch.hpp>)
  #include <catch2/catch.hpp>
#else
  #error "Catch2 headers not found. Please install Catch2 (v2 or v3)."
#endif

#include "socketcan_adapter/can_frame.hpp"
#include "sygnal_can_interface_lib/sygnal_command_interface.hpp"
#include "sygnal_dbc/mcm_identify.h"

namespace
{

polymath::socketcan::CanFrame toFrame(uint32_t can_id, const uint8_t (&buf)[8])
{
  polymath::socketcan::CanFrame frame;
  std::array<unsigned char, CAN_MAX_DLC> data;
  data.fill(0);
  for (size_t i = 0; i < 8; ++i) {
    data[i] = buf[i];
  }
  frame.set_can_id(can_id);
  frame.set_len(8);
  frame.set_data(data);
  return frame;
}

polymath::socketcan::CanFrame mainFrame(uint8_t bus, uint8_t sub, uint8_t boot, uint16_t product, uint32_t serial)
{
  mcm_identify_identify_response_main_t m;
  mcm_identify_identify_response_main_init(&m);
  m.bus_address = bus;
  m.subsystem_id = sub;
  m.module_boot_state = boot;
  m.product_id = product;
  m.module_serial_number = serial;
  uint8_t buf[8] = {0};
  mcm_identify_identify_response_main_pack(buf, &m, sizeof(buf));
  return toFrame(MCM_IDENTIFY_IDENTIFY_RESPONSE_MAIN_FRAME_ID, buf);
}

polymath::socketcan::CanFrame appVersionFrame(uint8_t bus, uint8_t sub, uint16_t major, uint16_t minor, uint16_t patch)
{
  mcm_identify_identify_response_app_version_t m;
  mcm_identify_identify_response_app_version_init(&m);
  m.bus_address = bus;
  m.subsystem_id = sub;
  m.software_version_major = major;
  m.software_version_minor = minor;
  m.software_version_patch = patch;
  uint8_t buf[8] = {0};
  mcm_identify_identify_response_app_version_pack(buf, &m, sizeof(buf));
  return toFrame(MCM_IDENTIFY_IDENTIFY_RESPONSE_APP_VERSION_FRAME_ID, buf);
}

polymath::socketcan::CanFrame blVersionFrame(uint8_t bus, uint8_t sub, uint16_t major, uint16_t minor, uint16_t patch)
{
  mcm_identify_identify_response_bl_version_t m;
  mcm_identify_identify_response_bl_version_init(&m);
  m.bus_address = bus;
  m.subsystem_id = sub;
  m.software_version_major = major;
  m.software_version_minor = minor;
  m.software_version_patch = patch;
  uint8_t buf[8] = {0};
  mcm_identify_identify_response_bl_version_pack(buf, &m, sizeof(buf));
  return toFrame(MCM_IDENTIFY_IDENTIFY_RESPONSE_BL_VERSION_FRAME_ID, buf);
}

}  // namespace

TEST_CASE("get_identity is nullopt before any identify response", "[sygnal_identify]")
{
  polymath::sygnal::SygnalMcmInterface interface(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 0);
  REQUIRE_FALSE(interface.get_identity().has_value());
}

TEST_CASE("parses IdentifyResponseMain for the matching endpoint", "[sygnal_identify]")
{
  polymath::sygnal::SygnalMcmInterface interface(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 0);

  auto frame = mainFrame(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 0, 2, 0x1234, 0xDEADBEEF);
  REQUIRE(interface.parseIdentifyResponseFrame(frame));

  auto identity = interface.get_identity();
  REQUIRE(identity.has_value());
  REQUIRE(identity->has_main);
  REQUIRE_FALSE(identity->has_app_version);
  REQUIRE_FALSE(identity->has_bl_version);
  REQUIRE(identity->module_boot_state == 2);
  REQUIRE(identity->product_id == 0x1234);
  REQUIRE(identity->module_serial_number == 0xDEADBEEF);
}

TEST_CASE("assembles full identity from all three response frames", "[sygnal_identify]")
{
  polymath::sygnal::SygnalMcmInterface interface(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 1);

  REQUIRE(interface.parseIdentifyResponseFrame(mainFrame(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 1, 0, 42, 100)));
  REQUIRE(interface.parseIdentifyResponseFrame(appVersionFrame(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 1, 3, 4, 5)));
  REQUIRE(interface.parseIdentifyResponseFrame(blVersionFrame(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 1, 6, 7, 8)));

  auto identity = interface.get_identity();
  REQUIRE(identity.has_value());
  REQUIRE(identity->has_main);
  REQUIRE(identity->has_app_version);
  REQUIRE(identity->has_bl_version);
  REQUIRE(identity->product_id == 42);
  REQUIRE(identity->app_version == polymath::sygnal::SygnalVersion{3, 4, 5});
  REQUIRE(identity->bl_version == polymath::sygnal::SygnalVersion{6, 7, 8});
}

TEST_CASE("ignores identify responses addressed to a different endpoint", "[sygnal_identify]")
{
  polymath::sygnal::SygnalMcmInterface interface(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 0);

  // Wrong subsystem
  REQUIRE_FALSE(interface.parseIdentifyResponseFrame(mainFrame(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 1, 0, 1, 1)));
  // Wrong bus address
  REQUIRE_FALSE(
    interface.parseIdentifyResponseFrame(mainFrame(polymath::sygnal::SECONDARY_MCM_BUS_ADDRESS, 0, 0, 1, 1)));

  REQUIRE_FALSE(interface.get_identity().has_value());
}

TEST_CASE("ignores non-identify frames", "[sygnal_identify]")
{
  polymath::sygnal::SygnalMcmInterface interface(polymath::sygnal::DEFAULT_MCM_BUS_ADDRESS, 0);
  uint8_t buf[8] = {0};
  REQUIRE_FALSE(interface.parseIdentifyResponseFrame(toFrame(0x123, buf)));
}

TEST_CASE("createIdentifyCommandFrame is a zero-length broadcast", "[sygnal_identify]")
{
  polymath::sygnal::SygnalControlInterface control;
  auto frame = control.createIdentifyCommandFrame();
  REQUIRE(frame.get_id() == MCM_IDENTIFY_IDENTIFY_COMMAND_FRAME_ID);
  REQUIRE(frame.get_len() == 0);
}
