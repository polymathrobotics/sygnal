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

#include "sygnal_can_interface_lib/response_queues.hpp"

#include <chrono>
#include <future>
#include <vector>

#if __has_include(<catch2/catch_all.hpp>)
  #include <catch2/catch_all.hpp>
#elif __has_include(<catch2/catch.hpp>)
  #include <catch2/catch.hpp>
#else
  #error "Catch2 headers not found. Please install Catch2 (v2 or v3)."
#endif

using polymath::sygnal::MAX_PROMISE_QUEUE_LENGTH;
using polymath::sygnal::ResponseQueues;

namespace
{

constexpr std::chrono::seconds TIMEOUT{1};
constexpr std::chrono::steady_clock::time_point START{};

template <typename T>
bool is_ready(const std::future<T> & future)
{
  return std::future_status::ready == future.wait_for(std::chrono::seconds(0));
}

}  // namespace

TEST_CASE("ResponseQueues fulfills the oldest promise on a key first", "[response_queues]")
{
  ResponseQueues<int, int> queues(TIMEOUT);
  auto first = queues.push(1);
  auto second = queues.push(1);

  queues.fulfill(1, 10);
  REQUIRE(is_ready(first));
  CHECK(10 == first.get());  // NOLINT(readability/check)
  CHECK_FALSE(is_ready(second));

  queues.fulfill(1, 20);
  CHECK(20 == second.get());  // NOLINT(readability/check)
}

TEST_CASE("ResponseQueues keeps keys independent", "[response_queues]")
{
  ResponseQueues<int, int> queues(TIMEOUT);
  auto on_one = queues.push(1);
  auto on_two = queues.push(2);

  queues.fulfill(2, 5);
  CHECK_FALSE(is_ready(on_one));
  REQUIRE(is_ready(on_two));
  CHECK(5 == on_two.get());  // NOLINT(readability/check)
}

TEST_CASE("ResponseQueues ignores a response with nothing pending", "[response_queues]")
{
  ResponseQueues<int, int> queues(TIMEOUT);
  queues.fulfill(1, 5);

  auto future = queues.push(1);
  CHECK_FALSE(is_ready(future));
}

TEST_CASE("ResponseQueues withdraws the newest promise on a key", "[response_queues]")
{
  ResponseQueues<int, int> queues(TIMEOUT);
  auto first = queues.push(1);
  auto second = queues.push(1);

  queues.withdraw_newest(1);
  queues.withdraw_newest(2);  // no-op: nothing pending on key 2

  CHECK_THROWS_AS(second.get(), std::future_error);
  queues.fulfill(1, 3);
  CHECK(3 == first.get());  // NOLINT(readability/check)
}

TEST_CASE("ResponseQueues drops the oldest promise when full", "[response_queues]")
{
  ResponseQueues<int, int> queues(TIMEOUT);
  std::vector<std::future<int>> futures;
  for (size_t i = 0; i <= MAX_PROMISE_QUEUE_LENGTH; ++i) {
    futures.push_back(queues.push(1));
  }

  CHECK_THROWS_AS(futures.front().get(), std::future_error);
  queues.fulfill(1, 7);
  CHECK(7 == futures[1].get());  // NOLINT(readability/check)
}

TEST_CASE("ResponseQueues fulfill skips expired promises", "[response_queues]")
{
  ResponseQueues<int, int> queues(TIMEOUT);
  auto expired = queues.push(1, START);
  auto live = queues.push(1, START + TIMEOUT);

  queues.fulfill(1, 5, START + TIMEOUT + std::chrono::milliseconds(1));
  CHECK_THROWS_AS(expired.get(), std::future_error);
  REQUIRE(is_ready(live));
  CHECK(5 == live.get());  // NOLINT(readability/check)
}

TEST_CASE("ResponseQueues fulfill keeps a promise until its deadline", "[response_queues]")
{
  ResponseQueues<int, int> queues(TIMEOUT);
  auto future = queues.push(1, START);

  queues.fulfill(1, 5, START + TIMEOUT);
  REQUIRE(is_ready(future));
  CHECK(5 == future.get());  // NOLINT(readability/check)
}

TEST_CASE("ResponseQueues push drops expired promises", "[response_queues]")
{
  ResponseQueues<int, int> queues(TIMEOUT);
  auto expired = queues.push(1, START);
  auto live = queues.push(1, START + 2 * TIMEOUT);

  CHECK_THROWS_AS(expired.get(), std::future_error);
  CHECK_FALSE(is_ready(live));
}
