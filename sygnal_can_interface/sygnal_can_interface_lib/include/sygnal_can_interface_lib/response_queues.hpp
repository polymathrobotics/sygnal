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

#include <cstddef>
#include <deque>
#include <future>
#include <map>
#include <mutex>

namespace polymath::sygnal
{

constexpr size_t MAX_PROMISE_QUEUE_LENGTH = 100;

/// @brief FIFO queues of pending response promises, one queue per key. Thread-safe.
template <typename Key, typename T>
class ResponseQueues
{
public:
  /// @brief Queue a promise for the next response on key.
  ///        A full queue drops its oldest promise, whose future then throws std::future_error (broken_promise).
  std::future<T> push(const Key & key)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::deque<std::promise<T>> & queue = queues_[key];
    if (MAX_PROMISE_QUEUE_LENGTH <= queue.size()) {
      queue.pop_front();
    }
    queue.emplace_back();
    return queue.back().get_future();
  }

  /// @brief Fulfill the oldest promise on key with value. No-op when none is pending.
  void fulfill(const Key & key, const T & value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = queues_.find(key);
    if (queues_.end() == it || it->second.empty()) {
      return;
    }
    it->second.front().set_value(value);
    it->second.pop_front();
  }

private:
  std::mutex mutex_;
  std::map<Key, std::deque<std::promise<T>>> queues_;
};

}  // namespace polymath::sygnal
