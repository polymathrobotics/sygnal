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

// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include <catch2/catch.hpp>

#include "mvec_lib/mvec_relay_socketcan.hpp"
#include "socketcan_adapter/socketcan_adapter.hpp"

TEST_CASE("MvecRelaySocketcan hardware integration test", "[hardware]")
{
  // Initialize socketcan adapter with a real CAN interface
  auto socketcan_adapter = std::make_shared<polymath::socketcan::SocketcanAdapter>("can0");

  // Open the socket
  bool open_result = socketcan_adapter->openSocket();
  REQUIRE(open_result);  // True means success

  // Create the MvecRelaySocketcan instance
  auto mvec_socketcan = std::make_unique<polymath::sygnal::MvecRelaySocketcan>(socketcan_adapter);

  // Set up callback to parse incoming messages
  bool callback_result = socketcan_adapter->setOnReceiveCallback(
    [&mvec_socketcan](std::unique_ptr<const polymath::socketcan::CanFrame> frame) { mvec_socketcan->parse(*frame); });
  REQUIRE(callback_result);  // True means success

  // Start receiving messages
  bool start_result = socketcan_adapter->startReceptionThread();
  REQUIRE(start_result);  // True means success

  SECTION("Test population query")
  {
    std::cout << "Testing population query..." << std::endl;

    // Send population query and get future
    auto population_future = mvec_socketcan->get_relay_population();

    // Wait for response (with timeout)
    auto status = population_future.wait_for(std::chrono::seconds(5));

    if (status == std::future_status::ready) {
      auto population_reply = population_future.get();
      std::cout << "Population query successful! Valid: " << population_reply.is_valid() << std::endl;

      // Check that we got a valid response
      REQUIRE(population_reply.is_valid());

      // Print some population information
      std::cout << "Population information received:" << std::endl;
      for (int i = 0; i < polymath::sygnal::MvecHardware::MAX_NUMBER_RELAYS; ++i) {
        if (population_reply.get_relay_population(i)) {
          std::cout << "  Relay " << i << " is populated" << std::endl;
        }
      }
      for (int i = 0; i < polymath::sygnal::MvecHardware::MAX_NUMBER_FUSES; ++i) {
        if (population_reply.get_fuse_population(i)) {
          std::cout << "  Fuse " << i << " is populated" << std::endl;
        }
      }
      std::cout << "High side output populated: " << population_reply.get_high_side_output_population() << std::endl;
    } else {
      std::cout << "Population query timed out - no MVEC device responding" << std::endl;
      // For hardware test, we'll mark this as a skip rather than failure
      WARN("No MVEC device found - population query timed out");
    }
  }

  SECTION("Test relay state query")
  {
    std::cout << "Testing relay state query..." << std::endl;

    // Send relay state query and get future
    auto relay_state_future = mvec_socketcan->get_relay_state();

    // Wait for response (with timeout)
    auto status = relay_state_future.wait_for(std::chrono::seconds(5));

    if (status == std::future_status::ready) {
      auto relay_query_reply = relay_state_future.get();
      std::cout << "Relay state query successful! Valid: " << relay_query_reply.is_valid() << std::endl;

      // Check that we got a valid response
      REQUIRE(relay_query_reply.is_valid());

      // Print relay states
      std::cout << "Relay states:" << std::endl;
      for (int i = 0; i < polymath::sygnal::MvecHardware::MAX_NUMBER_RELAYS; ++i) {
        std::cout << "  Relay " << i << ": " << relay_query_reply.get_relay_state(i) << std::endl;
      }

      std::cout << "High side output: " << relay_query_reply.get_high_side_output_state() << std::endl;

      // Print default relay states (power-on defaults)
      std::cout << "Default relay states (power-on):" << std::endl;
      for (int i = 0; i < polymath::sygnal::MvecHardware::MAX_NUMBER_RELAYS; ++i) {
        std::cout << "  Relay " << i << " default: " << relay_query_reply.get_relay_default(i) << std::endl;
      }

      std::cout << "High side output default: " << relay_query_reply.get_high_side_output_default() << std::endl;
    } else {
      std::cout << "Relay state query timed out - no MVEC device responding" << std::endl;
      // For hardware test, we'll mark this as a skip rather than failure
      WARN("No MVEC device found - relay state query timed out");
    }
  }

  SECTION("Test automatic status message updates")
  {
    std::cout << "Testing automatic status message updates..." << std::endl;

    std::atomic<bool> running{true};
    auto fuse_status_future = std::async(std::launch::async, [&]() {
      while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        const auto fuse_status = mvec_socketcan->get_last_fuse_status();
        if (fuse_status.has_value()) {
          return fuse_status;
        }
      }
    });

    REQUIRE(fuse_status_future.wait_for(std::chrono::seconds(2)) == std::future_status::ready);
    running = false;

    // Check if we received any status messages
    auto fuse_status = fuse_status_future.get();

    // Error status returns NO VALUE initially
    std::cout << "Received fuse status message! Valid: " << fuse_status->is_valid() << std::endl;
    if (fuse_status->is_valid()) {
      for (int i = 0; i < polymath::sygnal::MvecHardware::MAX_NUMBER_FUSES; ++i) {
        auto status = fuse_status->get_fuse_status(i);
        if (status != polymath::sygnal::MvecFuseStatus::NOT_USED) {
          std::cout << "  Fuse " << i << ": " << static_cast<int>(status) << std::endl;
        }
      }
    }

    running = true;
    auto relay_status_future = std::async(std::launch::async, [&]() {
      while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        const auto relay_status = mvec_socketcan->get_last_relay_status();
        if (relay_status.has_value()) {
          return relay_status;
        }
      }
    });

    REQUIRE(relay_status_future.wait_for(std::chrono::seconds(2)) == std::future_status::ready);
    running = false;

    // Check if we received any status messages
    auto relay_status = relay_status_future.get();

    std::cout << "Received relay status message! Valid: " << relay_status->is_valid() << std::endl;
    if (relay_status->is_valid()) {
      for (int i = 0; i < polymath::sygnal::MvecHardware::MAX_NUMBER_RELAYS; ++i) {
        auto status = relay_status->get_relay_status(i);
        if (status != polymath::sygnal::MvecRelayStatus::RELAY_LOCATION_NOT_USED) {
          std::cout << "  Relay " << i << ": " << static_cast<int>(status) << std::endl;
        }
      }
    }

    const auto & error_status = mvec_socketcan->get_last_error_status();

    if (error_status.has_value()) {
      std::cout << "Received error status message! Valid: " << error_status->is_valid() << std::endl;
      if (error_status->is_valid()) {
        std::cout << "  Error bits: 0x" << std::hex << error_status->get_error_bits() << std::dec << std::endl;
        std::cout << "  Grid address: 0x" << std::hex << static_cast<int>(error_status->get_grid_address()) << std::dec
                  << std::endl;
      }
    }

    // This test doesn't require assertions since status messages are broadcast periodically
    // and may not be available immediately
    std::cout << "Status message test completed (no assertions - depends on device broadcast)" << std::endl;
  }

  SECTION("Set Relays and Validate")
  {
    mvec_socketcan->set_relay_in_command(8, 1);
    mvec_socketcan->set_relay_in_command(9, 1);
    std::cout << "Setting relays 8 & 9 to true, leaving all other relays as is" << std::endl;
    auto relay_command_response_future = mvec_socketcan->send_relay_command();

    auto status = relay_command_response_future.wait_for(std::chrono::seconds(5));

    REQUIRE(status == std::future_status::ready);

    auto response = relay_command_response_future.get();

    // 1 is no error
    REQUIRE(response.get_success() == 1);
    std::cout << "Relay response message confirms success" << std::endl;

    auto mvec_query_future = mvec_socketcan->get_relay_state();
    status = mvec_query_future.wait_for(std::chrono::seconds(5));
    auto relay_state = mvec_query_future.get();

    REQUIRE(relay_state.get_relay_state(8) == 1);
    REQUIRE(relay_state.get_relay_state(9) == 1);
    std::cout << "Relay states queried and confirm command" << std::endl;
  }

  // Clean up
  socketcan_adapter->joinReceptionThread();
  socketcan_adapter->closeSocket();

  std::cout << "Hardware test completed successfully!" << std::endl;
}
