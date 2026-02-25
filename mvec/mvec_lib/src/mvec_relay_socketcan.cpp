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

#include <mutex>
#include <utility>

namespace polymath::sygnal
{

MvecRelaySocketcan::MvecRelaySocketcan(std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter)
: MvecRelaySocketcan(socketcan_adapter, MVEC_DEFAULT_RESPONSE_TIMEOUT)
{}

MvecRelaySocketcan::MvecRelaySocketcan(
  std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter, std::chrono::milliseconds response_timeout)
: socketcan_adapter_(socketcan_adapter)
, relay_impl_()
, response_timeout_(response_timeout)
{}

MvecMessageType MvecRelaySocketcan::parse(const socketcan::CanFrame & frame)
{
  MvecMessageType message_type = relay_impl_.parseMessage(frame);

  switch (message_type) {
    case MvecMessageType::RELAY_QUERY_RESPONSE: {
      const auto & reply = relay_impl_.get_last_relay_query_reply();
      std::lock_guard<std::mutex> lock(query_mutex_);
      if (reply.is_valid() && query_reply_promise_.has_value()) {
        query_reply_promise_->set_value(std::make_optional(reply));
        query_reply_promise_.reset();
      }
      break;
    }
    case MvecMessageType::RELAY_COMMAND_RESPONSE: {
      const auto & reply = relay_impl_.get_last_relay_command_reply();
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (reply.is_valid() && command_reply_promise_.has_value()) {
        command_reply_promise_->set_value(std::make_optional(reply));
        command_reply_promise_.reset();
      }
      break;
    }
    case MvecMessageType::POPULATION_RESPONSE: {
      const auto & reply = relay_impl_.get_last_population_reply();
      std::lock_guard<std::mutex> lock(population_mutex_);
      if (reply.is_valid() && population_reply_promise_.has_value()) {
        population_reply_promise_->set_value(std::make_optional(reply));
        population_reply_promise_.reset();
      }
      break;
    }
    default:
      // Status messages and other types don't need promise handling
      break;
  }

  return message_type;
}

void MvecRelaySocketcan::set_relay_in_command(uint8_t relay_id, uint8_t relay_state)
{
  /// TODO: (zeerek) Where do we check against the population table? When do we initially query the population table?
  relay_impl_.set_relay_in_command(relay_id, relay_state);
}

void MvecRelaySocketcan::clear_relay()
{
  relay_impl_.clearRelayCommands();
}

std::future<std::optional<MvecRelayQueryReply>> MvecRelaySocketcan::get_relay_state()
{
  std::lock_guard<std::mutex> lock(query_mutex_);

  // If a request is already in-flight, check if it has timed out
  if (query_reply_promise_.has_value()) {
    auto elapsed = std::chrono::steady_clock::now() - query_send_time_;
    if (elapsed < response_timeout_) {
      // Still in-flight, reject this request
      std::promise<std::optional<MvecRelayQueryReply>> rejected;
      auto future = rejected.get_future();
      rejected.set_value(std::nullopt);
      return future;
    }
    // Timed out — fulfill the old promise with nullopt so any waiter gets a clean result
    query_reply_promise_->set_value(std::nullopt);
    query_reply_promise_.reset();
  }

  std::promise<std::optional<MvecRelayQueryReply>> promise;
  auto future = promise.get_future();
  query_reply_promise_.emplace(std::move(promise));
  query_send_time_ = std::chrono::steady_clock::now();

  auto query_frame = relay_impl_.getRelayQueryMessage();
  socketcan_adapter_->send(query_frame);

  return future;
}

std::future<std::optional<MvecRelayCommandReply>> MvecRelaySocketcan::send_relay_command()
{
  std::lock_guard<std::mutex> lock(command_mutex_);

  if (command_reply_promise_.has_value()) {
    auto elapsed = std::chrono::steady_clock::now() - command_send_time_;
    if (elapsed < response_timeout_) {
      std::promise<std::optional<MvecRelayCommandReply>> rejected;
      auto future = rejected.get_future();
      rejected.set_value(std::nullopt);
      return future;
    }
    command_reply_promise_->set_value(std::nullopt);
    command_reply_promise_.reset();
  }

  std::promise<std::optional<MvecRelayCommandReply>> promise;
  auto future = promise.get_future();
  command_reply_promise_.emplace(std::move(promise));
  command_send_time_ = std::chrono::steady_clock::now();

  auto command_frame = relay_impl_.getRelayCommandMessage();
  socketcan_adapter_->send(command_frame);

  return future;
}

std::future<std::optional<MvecPopulationReply>> MvecRelaySocketcan::get_relay_population()
{
  std::lock_guard<std::mutex> lock(population_mutex_);

  if (population_reply_promise_.has_value()) {
    auto elapsed = std::chrono::steady_clock::now() - population_send_time_;
    if (elapsed < response_timeout_) {
      std::promise<std::optional<MvecPopulationReply>> rejected;
      auto future = rejected.get_future();
      rejected.set_value(std::nullopt);
      return future;
    }
    population_reply_promise_->set_value(std::nullopt);
    population_reply_promise_.reset();
  }

  std::promise<std::optional<MvecPopulationReply>> promise;
  auto future = promise.get_future();
  population_reply_promise_.emplace(std::move(promise));
  population_send_time_ = std::chrono::steady_clock::now();

  auto population_frame = relay_impl_.getPopulationQueryMessage();
  socketcan_adapter_->send(population_frame);

  return future;
}

const std::optional<MvecFuseStatusMessage> MvecRelaySocketcan::get_last_fuse_status()
{
  static std::optional<MvecFuseStatusMessage> result;

  const auto & fuse_status = relay_impl_.get_fuse_status_message();
  if (fuse_status.is_valid()) {
    result = fuse_status;
  } else {
    result = std::nullopt;
  }

  return result;
}

const std::optional<MvecRelayStatusMessage> MvecRelaySocketcan::get_last_relay_status()
{
  static std::optional<MvecRelayStatusMessage> result;

  const auto & relay_status = relay_impl_.get_relay_status_message();
  if (relay_status.is_valid()) {
    result = relay_status;
  } else {
    result = std::nullopt;
  }

  return result;
}

const std::optional<MvecErrorStatusMessage> MvecRelaySocketcan::get_last_error_status()
{
  static std::optional<MvecErrorStatusMessage> result;

  const auto & error_status = relay_impl_.get_error_status_message();
  if (error_status.is_valid()) {
    result = error_status;
  } else {
    result = std::nullopt;
  }

  return result;
}

}  // namespace polymath::sygnal
