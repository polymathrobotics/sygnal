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

#include "mvec_lib/mvec_relay_socketcan.hpp"

#include <chrono>
#include <future>
#include <memory>
#include <thread>
#include <vector>

#if __has_include(<catch2/catch_all.hpp>)
  #include <catch2/catch_all.hpp>
#elif __has_include(<catch2/catch.hpp>)
  #include <catch2/catch.hpp>
#else
  #error "Catch2 headers not found. Please install Catch2 (v2 or v3)."
#endif

#include "mvec_lib/core/mvec_constants.hpp"
#include "socketcan_adapter/can_frame.hpp"
#include "socketcan_adapter/socketcan_adapter.hpp"

using namespace std::chrono_literals;

static polymath::socketcan::CanFrame createTestFrame(uint32_t can_id, const std::vector<uint8_t> & data)
{
  polymath::socketcan::CanFrame frame;
  frame.set_can_id(can_id);
  frame.set_id_as_extended();
  std::array<unsigned char, CAN_MAX_DLC> frame_data;
  frame_data.fill(0);
  for (size_t i = 0; i < data.size() && i < CAN_MAX_DLC; ++i) {
    frame_data[i] = data[i];
  }
  frame.set_data(frame_data);
  return frame;
}

// J1939 ID for specific (peer-to-peer) responses from MVEC device
static const J1939_ID SPECIFIC_RESPONSE_ID(
  6, 0, polymath::sygnal::MvecProtocol::QUERY_PDU, 0x00, polymath::sygnal::MvecProtocol::DEFAULT_SOURCE_ADDRESS);

static polymath::socketcan::CanFrame makeQueryReplyFrame()
{
  return createTestFrame(SPECIFIC_RESPONSE_ID.get_can_id(), {0x96, 0x00, 0xFF, 0x0F, 0xAA, 0x55, 0x00, 0x00});
}

static polymath::socketcan::CanFrame makeCommandReplyFrame()
{
  return createTestFrame(SPECIFIC_RESPONSE_ID.get_can_id(), {0x01, 0x88, 0x01, 0x00, 0xFF, 0x0F, 0x00, 0x00});
}

static polymath::socketcan::CanFrame makePopulationReplyFrame()
{
  return createTestFrame(SPECIFIC_RESPONSE_ID.get_can_id(), {0x94, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x0F, 0x00});
}

static std::shared_ptr<polymath::socketcan::SocketcanAdapter> makeDummyAdapter()
{
  // Socket is never opened — send() fails silently, which is fine for testing promise logic
  return std::make_shared<polymath::socketcan::SocketcanAdapter>("dummy_test_iface");
}

// ============================================================================
// Query tests (get_relay_state)
// ============================================================================

TEST_CASE("Query returns future that is fulfilled by parse", "[mvec_relay_socketcan][query]")
{
  auto mvec = polymath::sygnal::MvecRelaySocketcan(makeDummyAdapter());

  auto future = mvec.get_relay_state();
  REQUIRE(future.wait_for(0ms) == std::future_status::timeout);

  auto msg_type = mvec.parse(makeQueryReplyFrame());
  REQUIRE(msg_type == polymath::sygnal::MvecMessageType::RELAY_QUERY_RESPONSE);
  REQUIRE(future.wait_for(0ms) == std::future_status::ready);

  auto reply = future.get();
  REQUIRE(reply.is_valid());
}

TEST_CASE("Query abandon-and-resend: second call breaks first future", "[mvec_relay_socketcan][query]")
{
  auto mvec = polymath::sygnal::MvecRelaySocketcan(makeDummyAdapter());

  auto future1 = mvec.get_relay_state();
  auto future2 = mvec.get_relay_state();

  // future1's promise was destroyed — get() throws broken_promise
  REQUIRE_THROWS_AS(future1.get(), std::future_error);

  // future2 is still pending
  REQUIRE(future2.wait_for(0ms) == std::future_status::timeout);

  // Response fulfills the second (current) promise
  mvec.parse(makeQueryReplyFrame());
  REQUIRE(future2.wait_for(0ms) == std::future_status::ready);

  auto reply = future2.get();
  REQUIRE(reply.is_valid());
}

TEST_CASE("Query: mismatched response still valid (idempotent)", "[mvec_relay_socketcan][query]")
{
  auto mvec = polymath::sygnal::MvecRelaySocketcan(makeDummyAdapter());

  // Send two queries rapidly — second abandons first
  auto future1 = mvec.get_relay_state();
  auto future2 = mvec.get_relay_state();

  // First CAN response arrives (response to query 1, but fills promise 2)
  mvec.parse(makeQueryReplyFrame());
  REQUIRE(future2.wait_for(0ms) == std::future_status::ready);

  auto reply = future2.get();
  REQUIRE(reply.is_valid());

  // Second response arrives — no promise, silently dropped
  auto msg_type = mvec.parse(makeQueryReplyFrame());
  REQUIRE(msg_type == polymath::sygnal::MvecMessageType::RELAY_QUERY_RESPONSE);
}

TEST_CASE("Query: response with no pending promise is silently dropped", "[mvec_relay_socketcan][query]")
{
  auto mvec = polymath::sygnal::MvecRelaySocketcan(makeDummyAdapter());

  auto msg_type = mvec.parse(makeQueryReplyFrame());
  REQUIRE(msg_type == polymath::sygnal::MvecMessageType::RELAY_QUERY_RESPONSE);
}

TEST_CASE("Query: wait_for timeout when response never arrives", "[mvec_relay_socketcan][query]")
{
  auto mvec = polymath::sygnal::MvecRelaySocketcan(makeDummyAdapter());

  auto future = mvec.get_relay_state();
  REQUIRE(future.wait_for(10ms) == std::future_status::timeout);
}

// ============================================================================
// Command tests (send_relay_command)
// ============================================================================

TEST_CASE("Command returns future that is fulfilled by parse", "[mvec_relay_socketcan][command]")
{
  auto mvec = polymath::sygnal::MvecRelaySocketcan(makeDummyAdapter());
  mvec.set_relay_in_command(0, 1);

  auto future = mvec.send_relay_command();
  REQUIRE(future.wait_for(0ms) == std::future_status::timeout);

  mvec.parse(makeCommandReplyFrame());
  REQUIRE(future.wait_for(0ms) == std::future_status::ready);

  auto reply = future.get();
  REQUIRE(reply.is_valid());
}

TEST_CASE("Command abandon-and-resend: second call breaks first future", "[mvec_relay_socketcan][command]")
{
  auto mvec = polymath::sygnal::MvecRelaySocketcan(makeDummyAdapter());
  mvec.set_relay_in_command(0, 1);

  auto future1 = mvec.send_relay_command();
  mvec.set_relay_in_command(1, 1);
  auto future2 = mvec.send_relay_command();

  // future1's promise was destroyed — get() throws broken_promise
  REQUIRE_THROWS_AS(future1.get(), std::future_error);

  // future2 is still pending
  REQUIRE(future2.wait_for(0ms) == std::future_status::timeout);

  // Response fulfills the second (current) promise
  mvec.parse(makeCommandReplyFrame());
  REQUIRE(future2.wait_for(0ms) == std::future_status::ready);

  auto reply = future2.get();
  REQUIRE(reply.is_valid());
}

TEST_CASE("Command: response with no pending promise is silently dropped", "[mvec_relay_socketcan][command]")
{
  auto mvec = polymath::sygnal::MvecRelaySocketcan(makeDummyAdapter());

  auto msg_type = mvec.parse(makeCommandReplyFrame());
  REQUIRE(msg_type == polymath::sygnal::MvecMessageType::RELAY_COMMAND_RESPONSE);
}

// ============================================================================
// Population tests (get_relay_population)
// ============================================================================

TEST_CASE("Population returns future that is fulfilled by parse", "[mvec_relay_socketcan][population]")
{
  auto mvec = polymath::sygnal::MvecRelaySocketcan(makeDummyAdapter());

  auto future = mvec.get_relay_population();
  REQUIRE(future.wait_for(0ms) == std::future_status::timeout);

  mvec.parse(makePopulationReplyFrame());
  REQUIRE(future.wait_for(0ms) == std::future_status::ready);

  auto reply = future.get();
  REQUIRE(reply.is_valid());
}

TEST_CASE("Population abandon-and-resend: second call breaks first future", "[mvec_relay_socketcan][population]")
{
  auto mvec = polymath::sygnal::MvecRelaySocketcan(makeDummyAdapter());

  auto future1 = mvec.get_relay_population();
  auto future2 = mvec.get_relay_population();

  REQUIRE_THROWS_AS(future1.get(), std::future_error);

  mvec.parse(makePopulationReplyFrame());
  REQUIRE(future2.wait_for(0ms) == std::future_status::ready);

  auto reply = future2.get();
  REQUIRE(reply.is_valid());
}

// ============================================================================
// Status getter tests
// ============================================================================

TEST_CASE("Status getters return nullopt before any data", "[mvec_relay_socketcan][status]")
{
  auto mvec = polymath::sygnal::MvecRelaySocketcan(makeDummyAdapter());

  REQUIRE_FALSE(mvec.get_last_fuse_status().has_value());
  REQUIRE_FALSE(mvec.get_last_relay_status().has_value());
  REQUIRE_FALSE(mvec.get_last_error_status().has_value());
}

TEST_CASE("Fuse status getter returns value after parse", "[mvec_relay_socketcan][status]")
{
  auto mvec = polymath::sygnal::MvecRelaySocketcan(makeDummyAdapter());

  J1939_ID fuse_status_id(
    6, 0, polymath::sygnal::MvecProtocol::STATUS_PDU,
    0x00 + polymath::sygnal::MvecProtocol::DEFAULT_PGN_BASE_VALUE,
    polymath::sygnal::MvecProtocol::DEFAULT_SOURCE_ADDRESS);
  auto frame = createTestFrame(fuse_status_id.get_can_id(), {0x00, 0x55, 0xAA, 0x33, 0xCC, 0x0F, 0xF0, 0x99});

  auto msg_type = mvec.parse(frame);
  REQUIRE(msg_type == polymath::sygnal::MvecMessageType::FUSE_STATUS);

  auto fuse_status = mvec.get_last_fuse_status();
  REQUIRE(fuse_status.has_value());
  REQUIRE(fuse_status->is_valid());
}

TEST_CASE("Relay status getter returns value after parse", "[mvec_relay_socketcan][status]")
{
  auto mvec = polymath::sygnal::MvecRelaySocketcan(makeDummyAdapter());

  J1939_ID relay_status_id(
    6, 0, polymath::sygnal::MvecProtocol::STATUS_PDU,
    0x01 + polymath::sygnal::MvecProtocol::DEFAULT_PGN_BASE_VALUE,
    polymath::sygnal::MvecProtocol::DEFAULT_SOURCE_ADDRESS);
  auto frame = createTestFrame(relay_status_id.get_can_id(), {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77});

  auto msg_type = mvec.parse(frame);
  REQUIRE(msg_type == polymath::sygnal::MvecMessageType::RELAY_STATUS);

  auto relay_status = mvec.get_last_relay_status();
  REQUIRE(relay_status.has_value());
  REQUIRE(relay_status->is_valid());
}

TEST_CASE("Error status getter returns value after parse", "[mvec_relay_socketcan][status]")
{
  auto mvec = polymath::sygnal::MvecRelaySocketcan(makeDummyAdapter());

  J1939_ID error_status_id(
    6, 0, polymath::sygnal::MvecProtocol::STATUS_PDU,
    0x02 + polymath::sygnal::MvecProtocol::DEFAULT_PGN_BASE_VALUE,
    polymath::sygnal::MvecProtocol::DEFAULT_SOURCE_ADDRESS);
  auto frame = createTestFrame(
    error_status_id.get_can_id(),
    {polymath::sygnal::MvecProtocol::DEFAULT_SOURCE_ADDRESS, 0xFF, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00});

  auto msg_type = mvec.parse(frame);
  REQUIRE(msg_type == polymath::sygnal::MvecMessageType::ERROR_STATUS);

  auto error_status = mvec.get_last_error_status();
  REQUIRE(error_status.has_value());
  REQUIRE(error_status->is_valid());
}

// ============================================================================
// Cross-type isolation tests
// ============================================================================

TEST_CASE("Query response does not fulfill command promise", "[mvec_relay_socketcan][isolation]")
{
  auto mvec = polymath::sygnal::MvecRelaySocketcan(makeDummyAdapter());
  mvec.set_relay_in_command(0, 1);

  auto query_future = mvec.get_relay_state();
  auto command_future = mvec.send_relay_command();

  // Parse a query response — should only fulfill query, not command
  mvec.parse(makeQueryReplyFrame());
  REQUIRE(query_future.wait_for(0ms) == std::future_status::ready);
  REQUIRE(command_future.wait_for(0ms) == std::future_status::timeout);

  // Parse a command response — now command is fulfilled
  mvec.parse(makeCommandReplyFrame());
  REQUIRE(command_future.wait_for(0ms) == std::future_status::ready);
}

TEST_CASE("All three request types can be in-flight simultaneously", "[mvec_relay_socketcan][isolation]")
{
  auto mvec = polymath::sygnal::MvecRelaySocketcan(makeDummyAdapter());
  mvec.set_relay_in_command(0, 1);

  auto query_future = mvec.get_relay_state();
  auto command_future = mvec.send_relay_command();
  auto population_future = mvec.get_relay_population();

  // All pending
  REQUIRE(query_future.wait_for(0ms) == std::future_status::timeout);
  REQUIRE(command_future.wait_for(0ms) == std::future_status::timeout);
  REQUIRE(population_future.wait_for(0ms) == std::future_status::timeout);

  // Fulfill in reverse order
  mvec.parse(makePopulationReplyFrame());
  REQUIRE(population_future.wait_for(0ms) == std::future_status::ready);
  REQUIRE(query_future.wait_for(0ms) == std::future_status::timeout);
  REQUIRE(command_future.wait_for(0ms) == std::future_status::timeout);

  mvec.parse(makeCommandReplyFrame());
  REQUIRE(command_future.wait_for(0ms) == std::future_status::ready);
  REQUIRE(query_future.wait_for(0ms) == std::future_status::timeout);

  mvec.parse(makeQueryReplyFrame());
  REQUIRE(query_future.wait_for(0ms) == std::future_status::ready);
}

// ============================================================================
// Thread safety test
// ============================================================================

TEST_CASE("Concurrent parse and query calls don't crash", "[mvec_relay_socketcan][threading]")
{
  auto mvec = std::make_shared<polymath::sygnal::MvecRelaySocketcan>(makeDummyAdapter());
  constexpr int ITERATIONS = 100;

  std::thread query_thread([&]() {
    for (int i = 0; i < ITERATIONS; ++i) {
      auto future = mvec->get_relay_state();
      future.wait_for(1ms);
    }
  });

  std::thread parse_thread([&]() {
    for (int i = 0; i < ITERATIONS; ++i) {
      mvec->parse(makeQueryReplyFrame());
    }
  });

  query_thread.join();
  parse_thread.join();
}
