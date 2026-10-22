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

#include "sygnal_can_interface_lib/sygnal_hpo_device.hpp"

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

using polymath::sygnal::SygnalDevice;
using polymath::sygnal::SygnalHpoDevice;
using polymath::sygnal::SygnalSystemState;

namespace
{

/// @brief Unopened adapter; every send fails.
std::shared_ptr<polymath::socketcan::SocketcanAdapter> adapter()
{
  return std::make_shared<polymath::socketcan::SocketcanAdapter>("vcan0");
}

/// @brief Exposes the response queues, so replies can be awaited without sending.
class TestHpoDevice : public SygnalHpoDevice
{
public:
  using SygnalHpoDevice::control_command_responses_;
  using SygnalHpoDevice::control_enable_responses_;
  using SygnalHpoDevice::SygnalHpoDevice;
};

constexpr uint8_t BUS = 3;
constexpr std::chrono::seconds COMMAND_TIMEOUT{1};

template <typename T>
bool is_ready(const std::future<T> & future)
{
  return std::future_status::ready == future.wait_for(std::chrono::seconds(0));
}

}  // namespace

TEST_CASE("HPO heartbeat sets interface states", "[hpo_device]")
{
  TestHpoDevice device(adapter(), BUS, COMMAND_TIMEOUT);
  CHECK_FALSE(device.get_interface_states()[2]);

  REQUIRE(device.handle(sygnal_test::hpo_heartbeat(BUS, SygnalSystemState::HUMAN_CONTROL, {false, false, true})));

  CHECK_FALSE(device.get_interface_states()[0]);
  CHECK(device.get_interface_states()[2]);
}

TEST_CASE("HPO device rejects a frame with a bad CRC", "[hpo_device]")
{
  TestHpoDevice device(adapter(), BUS, COMMAND_TIMEOUT);
  can_frame frame = sygnal_test::hpo_heartbeat(BUS, SygnalSystemState::HUMAN_CONTROL, {true});
  frame.data[CAN_MAX_DLC - 1] ^= 0xFF;

  CHECK_FALSE(device.handle(frame));
  CHECK_FALSE(device.get_interface_states()[0]);
}

TEST_CASE("HPO enable response fulfills only its message's future", "[hpo_device]")
{
  TestHpoDevice device(adapter(), BUS, COMMAND_TIMEOUT);
  auto message0 = device.control_enable_responses_.push(0);
  auto message1 = device.control_enable_responses_.push(1);

  REQUIRE(device.handle(sygnal_test::hpo_enable_response(BUS, 1, true)));

  CHECK_FALSE(is_ready(message0));
  REQUIRE(is_ready(message1));
  const auto response = message1.get();
  CHECK(response.is_enable_response);
  CHECK(response.enable);
  CHECK(1 == response.message_id);  // NOLINT(readability/check)
}

TEST_CASE("HPO command response fulfills its message's future", "[hpo_device]")
{
  TestHpoDevice device(adapter(), BUS, COMMAND_TIMEOUT);
  auto future = device.control_command_responses_.push(10);

  REQUIRE(device.handle(sygnal_test::hpo_command_response(BUS, 10, 1.5)));

  REQUIRE(is_ready(future));
  CHECK(1.5 == future.get().value);
}

TEST_CASE("HPO failed send leaves no pending reply", "[hpo_device]")
{
  TestHpoDevice device(adapter(), BUS, COMMAND_TIMEOUT);
  std::string error_message;

  const auto result = device.send_control_enable(0x42, true, true, error_message);

  CHECK_FALSE(result.success);
  CHECK_FALSE(result.response_future.has_value());
  CHECK_FALSE(error_message.empty());
}
