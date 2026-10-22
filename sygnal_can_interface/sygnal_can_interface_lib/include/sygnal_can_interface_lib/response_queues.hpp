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

#pragma once

#include <chrono>
#include <cstddef>
#include <deque>
#include <future>
#include <map>
#include <mutex>

namespace polymath::sygnal
{

constexpr size_t MAX_PROMISE_QUEUE_LENGTH = 1000;

/// @brief FIFO queues of pending response promises, one queue per key. Thread-safe.
///        The intended use if for each endpoint that sends a response has its own key in the map.
///        Each key in the map has its own FIFO queue of promises. This way responses for different
///        endpoints do not fulfill each other's promises.

template <typename Key, typename T>
class ResponseQueues
{
public:
  /// @param timeout How long a promise waits for its response.
  ///        push and fulfill drop expired promises on their key, whose futures then throw
  ///        std::future_error (broken_promise).
  explicit ResponseQueues(std::chrono::steady_clock::duration timeout)
  : timeout_(timeout)
  {}

  /// @brief Queue a promise for the next response on a specific key.
  ///        A full queue drops its oldest promise, whose future then throws std::future_error (broken_promise).
  std::future<T> push(const Key & key, std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now())
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::deque<Pending> & queue = queues_[key];
    drop_expired(queue, now);
    if (MAX_PROMISE_QUEUE_LENGTH <= queue.size()) {
      queue.pop_front();
    }
    queue.push_back({std::promise<T>{}, now + timeout_});
    return queue.back().promise.get_future();
  }

  /// @brief Fulfill the oldest unexpired promise on key with value. No-op when none is pending.
  void fulfill(
    const Key & key, const T & value, std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now())
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = queues_.find(key);
    if (queues_.end() == it) {
      return;
    }
    drop_expired(it->second, now);
    if (it->second.empty()) {
      return;
    }
    it->second.front().promise.set_value(value);
    it->second.pop_front();
  }

  /// @brief Drop the newest promise on key, whose future then throws std::future_error (broken_promise).
  ///        No-op when none is pending. Assumes one pusher at a time per key.
  void withdraw_newest(const Key & key)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = queues_.find(key);
    if (queues_.end() != it && !it->second.empty()) {
      it->second.pop_back();
    }
  }

private:
  struct Pending
  {
    std::promise<T> promise;
    std::chrono::steady_clock::time_point deadline;
  };

  static void drop_expired(std::deque<Pending> & queue, std::chrono::steady_clock::time_point now)
  {
    while (!queue.empty() && queue.front().deadline < now) {
      queue.pop_front();
    }
  }

  const std::chrono::steady_clock::duration timeout_;
  std::mutex mutex_;
  std::map<Key, std::deque<Pending>> queues_;
};

}  // namespace polymath::sygnal
