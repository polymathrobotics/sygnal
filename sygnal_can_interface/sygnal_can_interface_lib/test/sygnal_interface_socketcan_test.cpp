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

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#if __has_include(<catch2/catch_all.hpp>)
  #include <catch2/catch_all.hpp>
#else
  #include <catch2/catch.hpp>
#endif

#include <algorithm>
#include <future>
#include <string>

#include "socketcan_adapter/socketcan_adapter.hpp"
#include "sygnal_can_interface_lib/crc8.hpp"
#include "sygnal_can_interface_lib/sygnal_interface_socketcan.hpp"
#include "sygnal_dbc/mcm_control.h"
#include "sygnal_dbc/mcm_heartbeat.h"
#include "sygnal_dbc/mcm_relay.h"

namespace
{
constexpr const char * VCAN_INTERFACE = "vcan0";

polymath::socketcan::CanFrame createEnableResponseFrame(uint8_t bus_id, uint8_t interface_id, uint8_t enable)
{
  uint8_t buffer[CAN_MAX_DLC] = {0};
  mcm_control_control_enable_response_t response;
  mcm_control_control_enable_response_init(&response);
  response.bus_address = bus_id;
  response.interface_id = interface_id;
  response.enable = enable;
  response.crc = 0;

  mcm_control_control_enable_response_pack(buffer, &response, sizeof(buffer));
  response.crc = polymath::sygnal::generate_crc8(buffer);
  mcm_control_control_enable_response_pack(buffer, &response, sizeof(buffer));

  std::array<unsigned char, CAN_MAX_DLC> data;
  std::copy(std::begin(buffer), std::end(buffer), data.begin());

  polymath::socketcan::CanFrame frame;
  frame.set_can_id(MCM_CONTROL_CONTROL_ENABLE_RESPONSE_FRAME_ID);
  frame.set_len(MCM_CONTROL_CONTROL_ENABLE_RESPONSE_LENGTH);
  frame.set_data(data);
  return frame;
}

polymath::socketcan::CanFrame createControlResponseFrame(uint8_t bus_id, uint8_t interface_id, float value)
{
  uint8_t buffer[CAN_MAX_DLC] = {0};
  mcm_control_control_command_response_t response;
  mcm_control_control_command_response_init(&response);
  response.bus_address = bus_id;
  response.interface_id = interface_id;
  response.count8 = 0;
  response.value = mcm_control_control_command_response_value_encode(value);
  response.crc = 0;

  mcm_control_control_command_response_pack(buffer, &response, sizeof(buffer));
  response.crc = polymath::sygnal::generate_crc8(buffer);
  mcm_control_control_command_response_pack(buffer, &response, sizeof(buffer));

  std::array<unsigned char, CAN_MAX_DLC> data;
  std::copy(std::begin(buffer), std::end(buffer), data.begin());

  polymath::socketcan::CanFrame frame;
  frame.set_can_id(MCM_CONTROL_CONTROL_COMMAND_RESPONSE_FRAME_ID);
  frame.set_len(MCM_CONTROL_CONTROL_COMMAND_RESPONSE_LENGTH);
  frame.set_data(data);
  return frame;
}

polymath::socketcan::CanFrame createRelayResponseFrame(uint8_t bus_id, uint8_t subsystem_id, uint8_t enable)
{
  uint8_t buffer[CAN_MAX_DLC] = {0};
  mcm_relay_relay_command_response_t response;
  mcm_relay_relay_command_response_init(&response);
  response.bus_address = bus_id;
  response.subsystem_id = subsystem_id;
  response.enable = enable;
  response.crc = 0;

  mcm_relay_relay_command_response_pack(buffer, &response, sizeof(buffer));
  response.crc = polymath::sygnal::generate_crc8(buffer);
  mcm_relay_relay_command_response_pack(buffer, &response, sizeof(buffer));

  std::array<unsigned char, CAN_MAX_DLC> data;
  std::copy(std::begin(buffer), std::end(buffer), data.begin());

  polymath::socketcan::CanFrame frame;
  frame.set_can_id(MCM_RELAY_RELAY_COMMAND_RESPONSE_FRAME_ID);
  frame.set_len(MCM_RELAY_RELAY_COMMAND_RESPONSE_LENGTH);
  frame.set_data(data);
  return frame;
}

polymath::socketcan::CanFrame createHeartbeatFrame(
  uint8_t bus_id, uint8_t subsystem_id, uint8_t system_state, std::array<uint8_t, 5> interface_states)
{
  uint8_t buffer[CAN_MAX_DLC] = {0};
  mcm_heartbeat_heartbeat_t heartbeat;
  mcm_heartbeat_heartbeat_init(&heartbeat);
  heartbeat.bus_address = bus_id;
  heartbeat.subsystem_id = subsystem_id;
  heartbeat.system_state = system_state;
  heartbeat.interface0_state = interface_states[0];
  heartbeat.interface1_state = interface_states[1];
  heartbeat.interface2_state = interface_states[2];
  heartbeat.interface3_state = interface_states[3];
  heartbeat.interface4_state = interface_states[4];
  heartbeat.interface5_state = 0;
  heartbeat.interface6_state = 0;
  heartbeat.overall_interface_state = 0;
  heartbeat.count16 = 0;
  heartbeat.crc = 0;

  mcm_heartbeat_heartbeat_pack(buffer, &heartbeat, sizeof(buffer));
  heartbeat.crc = polymath::sygnal::generate_crc8(buffer);
  mcm_heartbeat_heartbeat_pack(buffer, &heartbeat, sizeof(buffer));

  std::array<unsigned char, CAN_MAX_DLC> data;
  std::copy(std::begin(buffer), std::end(buffer), data.begin());

  polymath::socketcan::CanFrame frame;
  frame.set_can_id(MCM_HEARTBEAT_HEARTBEAT_FRAME_ID);
  frame.set_len(MCM_HEARTBEAT_HEARTBEAT_LENGTH);
  frame.set_data(data);
  return frame;
}

}  // namespace

TEST_CASE("SygnalInterfaceSocketcan vcan0 integration tests", "[vcan]")
{
  auto interface_adapter = std::make_shared<polymath::socketcan::SocketcanAdapter>(VCAN_INTERFACE);
  auto simulator_adapter = std::make_shared<polymath::socketcan::SocketcanAdapter>(VCAN_INTERFACE);

  REQUIRE(interface_adapter->openSocket());
  REQUIRE(simulator_adapter->openSocket());

  auto sygnal_interface = std::make_unique<polymath::sygnal::SygnalInterfaceSocketcan>(interface_adapter);

  std::promise<void> frame_received_promise;
  std::future<void> frame_received_future = frame_received_promise.get_future();

  REQUIRE(interface_adapter->setOnReceiveCallback(
    [&sygnal_interface, &frame_received_promise](std::unique_ptr<const polymath::socketcan::CanFrame> frame) {
      frame_received_promise.set_value();
      sygnal_interface->parse(*frame);
    }));

  REQUIRE(interface_adapter->startReceptionThread());

  SECTION("Send control state command and receive response")
  {
    constexpr uint8_t bus_id = 1;
    constexpr uint8_t interface_id = 2;
    constexpr uint8_t subsystem_id = 0;

    std::string error_message;
    auto result = sygnal_interface->sendControlStateCommand(
      bus_id, interface_id, subsystem_id, polymath::sygnal::SygnalControlState::MCM_CONTROL, true, error_message);

    REQUIRE(result.success);
    REQUIRE(result.response_future.has_value());

    // std::this_thread::sleep_for(std::chrono::milliseconds(10));

    auto response_frame = createEnableResponseFrame(bus_id, interface_id, 1);
    auto send_err = simulator_adapter->send(response_frame);
    REQUIRE_FALSE(send_err.has_value());

    auto status = result.response_future->wait_for(std::chrono::seconds(2));
    REQUIRE(status == std::future_status::ready);

    auto response = result.response_future->get();
    REQUIRE(response.response_type == polymath::sygnal::SygnalControlCommandResponseType::ENABLE);
    REQUIRE(response.bus_id == bus_id);
    REQUIRE(response.interface_id == interface_id);
    REQUIRE(response.value == 1.0);
  }

  SECTION("Send control command and receive response")
  {
    constexpr uint8_t bus_id = 1;
    constexpr uint8_t interface_id = 3;
    constexpr uint8_t subsystem_id = 0;
    constexpr double control_value = 0.75;

    std::string error_message;
    auto result =
      sygnal_interface->sendControlCommand(bus_id, interface_id, subsystem_id, control_value, true, error_message);

    REQUIRE(result.success);
    REQUIRE(result.response_future.has_value());

    // std::this_thread::sleep_for(std::chrono::milliseconds(10));

    auto response_frame = createControlResponseFrame(bus_id, interface_id, static_cast<float>(control_value));
    auto send_err = simulator_adapter->send(response_frame);
    REQUIRE_FALSE(send_err.has_value());

    auto status = result.response_future->wait_for(std::chrono::seconds(2));
    REQUIRE(status == std::future_status::ready);

    auto response = result.response_future->get();
    REQUIRE(response.response_type == polymath::sygnal::SygnalControlCommandResponseType::CONTROL);
    REQUIRE(response.bus_id == bus_id);
    REQUIRE(response.interface_id == interface_id);
    REQUIRE(response.value == Approx(control_value).margin(0.01));
  }

  SECTION("Send relay command and receive response")
  {
    constexpr uint8_t bus_id = 1;
    constexpr uint8_t subsystem_id = 0;
    constexpr bool relay_state = true;

    std::string error_message;
    auto result = sygnal_interface->sendRelayCommand(bus_id, subsystem_id, relay_state, true, error_message);

    REQUIRE(result.success);
    REQUIRE(result.response_future.has_value());

    // std::this_thread::sleep_for(std::chrono::milliseconds(10));

    auto response_frame = createRelayResponseFrame(bus_id, subsystem_id, 1);
    auto send_err = simulator_adapter->send(response_frame);
    REQUIRE_FALSE(send_err.has_value());

    auto status = result.response_future->wait_for(std::chrono::seconds(2));
    REQUIRE(status == std::future_status::ready);

    auto response = result.response_future->get();
    REQUIRE(response.response_type == polymath::sygnal::SygnalControlCommandResponseType::RELAY);
    REQUIRE(response.bus_id == bus_id);
    REQUIRE(response.interface_id == subsystem_id);
    REQUIRE(response.value == 1.0);
  }

  SECTION("Parse MCM heartbeat for subsystem 0")
  {
    constexpr uint8_t bus_id = 1;
    constexpr uint8_t subsystem_id = 0;
    constexpr uint8_t system_state = static_cast<uint8_t>(polymath::sygnal::SygnalSystemState::MCM_CONTROL);
    std::array<uint8_t, 5> interface_states = {1, 0, 1, 0, 1};

    auto heartbeat_frame = createHeartbeatFrame(bus_id, subsystem_id, system_state, interface_states);
    auto send_err = simulator_adapter->send(heartbeat_frame);
    REQUIRE_FALSE(send_err.has_value());

    // std::this_thread::sleep_for(std::chrono::milliseconds(50));

    frame_received_future.wait_for(std::chrono::seconds(2));

    auto mcm0_state = sygnal_interface->get_sygnal_mcm0_state();
    REQUIRE(mcm0_state == polymath::sygnal::SygnalSystemState::MCM_CONTROL);

    auto interface_states_result = sygnal_interface->get_interface_states_0();
    REQUIRE(interface_states_result[0] == polymath::sygnal::SygnalSystemState::MCM_CONTROL);
    REQUIRE(interface_states_result[1] == polymath::sygnal::SygnalSystemState::HUMAN_CONTROL);
    REQUIRE(interface_states_result[2] == polymath::sygnal::SygnalSystemState::MCM_CONTROL);
    REQUIRE(interface_states_result[3] == polymath::sygnal::SygnalSystemState::HUMAN_CONTROL);
    REQUIRE(interface_states_result[4] == polymath::sygnal::SygnalSystemState::MCM_CONTROL);
  }

  SECTION("Parse MCM heartbeat for subsystem 1")
  {
    constexpr uint8_t bus_id = 1;
    constexpr uint8_t subsystem_id = 1;
    constexpr uint8_t system_state = static_cast<uint8_t>(polymath::sygnal::SygnalSystemState::HUMAN_CONTROL);
    std::array<uint8_t, 5> interface_states = {0, 1, 0, 1, 0};

    auto heartbeat_frame = createHeartbeatFrame(bus_id, subsystem_id, system_state, interface_states);
    auto send_err = simulator_adapter->send(heartbeat_frame);
    REQUIRE_FALSE(send_err.has_value());

    frame_received_future.wait_for(std::chrono::seconds(2));

    auto mcm1_state = sygnal_interface->get_sygnal_mcm1_state();
    REQUIRE(mcm1_state == polymath::sygnal::SygnalSystemState::HUMAN_CONTROL);

    auto interface_states_result = sygnal_interface->get_interface_states_1();
    REQUIRE(interface_states_result[0] == polymath::sygnal::SygnalSystemState::HUMAN_CONTROL);
    REQUIRE(interface_states_result[1] == polymath::sygnal::SygnalSystemState::MCM_CONTROL);
    REQUIRE(interface_states_result[2] == polymath::sygnal::SygnalSystemState::HUMAN_CONTROL);
    REQUIRE(interface_states_result[3] == polymath::sygnal::SygnalSystemState::MCM_CONTROL);
    REQUIRE(interface_states_result[4] == polymath::sygnal::SygnalSystemState::HUMAN_CONTROL);
  }

  SECTION("Fire and forget command (no response expected)")
  {
    constexpr uint8_t bus_id = 1;
    constexpr uint8_t interface_id = 0;
    constexpr uint8_t subsystem_id = 0;

    std::string error_message;
    auto result = sygnal_interface->sendControlStateCommand(
      bus_id, interface_id, subsystem_id, polymath::sygnal::SygnalControlState::HUMAN_CONTROL, false, error_message);

    REQUIRE(result.success);
    REQUIRE_FALSE(result.response_future.has_value());
  }

  SECTION("Invalid interface ID returns failure")
  {
    constexpr uint8_t bus_id = 1;
    constexpr uint8_t invalid_interface_id = 10;
    constexpr uint8_t subsystem_id = 0;

    std::string error_message;
    auto result = sygnal_interface->sendControlStateCommand(
      bus_id,
      invalid_interface_id,
      subsystem_id,
      polymath::sygnal::SygnalControlState::MCM_CONTROL,
      true,
      error_message);

    REQUIRE_FALSE(result.success);
    REQUIRE_FALSE(error_message.empty());
  }

  SECTION("Invalid subsystem ID returns failure")
  {
    constexpr uint8_t bus_id = 1;
    constexpr uint8_t interface_id = 0;
    constexpr uint8_t invalid_subsystem_id = 5;

    std::string error_message;
    auto result = sygnal_interface->sendControlStateCommand(
      bus_id,
      interface_id,
      invalid_subsystem_id,
      polymath::sygnal::SygnalControlState::MCM_CONTROL,
      true,
      error_message);

    REQUIRE_FALSE(result.success);
    REQUIRE_FALSE(error_message.empty());
  }

  interface_adapter->joinReceptionThread();
  interface_adapter->closeSocket();
  simulator_adapter->closeSocket();
}
