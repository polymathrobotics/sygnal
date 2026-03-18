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

#include <memory>
#include <mutex>
#include <utility>

namespace polymath::sygnal
{

MvecRelaySocketcan::MvecRelaySocketcan(std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter)
: socketcan_adapter_(socketcan_adapter)
, relay_impl_()
{}

MvecMessageType MvecRelaySocketcan::parse(const socketcan::CanFrame & frame)
{
  const MvecMessageType message_type = relay_impl_.parseMessage(frame);

  // Check if we received an expected response type and fulfill the waiting promise
  switch (message_type) {
    case MvecMessageType::RELAY_QUERY_RESPONSE: {
      const auto & reply = relay_impl_.get_last_relay_query_reply();
      std::lock_guard<std::mutex> lock(query_mutex_);
      if (reply.is_valid() && query_reply_promise_.has_value()) {
        // Fulfill the promise and clear the slot
        query_reply_promise_->set_value(reply);
        query_reply_promise_.reset();
      }
      break;
    }
    case MvecMessageType::RELAY_COMMAND_RESPONSE: {
      const auto & reply = relay_impl_.get_last_relay_command_reply();
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (reply.is_valid() && command_reply_promise_.has_value()) {
        command_reply_promise_->set_value(reply);
        command_reply_promise_.reset();
      }
      break;
    }
    case MvecMessageType::POPULATION_RESPONSE: {
      const auto & reply = relay_impl_.get_last_population_reply();
      std::lock_guard<std::mutex> lock(population_mutex_);
      if (reply.is_valid() && population_reply_promise_.has_value()) {
        population_reply_promise_->set_value(reply);
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

std::future<MvecRelayQueryReply> MvecRelaySocketcan::get_relay_state()
{
  std::lock_guard<std::mutex> lock(query_mutex_);

  // Abandon any in-flight query, caller's future becomes broken_promise if still waiting
  query_reply_promise_.reset();

  // Create a new promise and get its future
  std::promise<MvecRelayQueryReply> promise;
  auto future = promise.get_future();
  query_reply_promise_.emplace(std::move(promise));

  // Transmit the query message via socketcan adapter
  socketcan_adapter_->send(relay_impl_.getRelayQueryMessage());
  return future;
}

std::future<MvecRelayCommandReply> MvecRelaySocketcan::send_relay_command()
{
  std::lock_guard<std::mutex> lock(command_mutex_);

  // Abandon any in-flight command, caller's future becomes broken_promise if still waiting
  command_reply_promise_.reset();

  // Create a new promise and get its future
  std::promise<MvecRelayCommandReply> promise;
  auto future = promise.get_future();
  command_reply_promise_.emplace(std::move(promise));

  // Transmit the command message via socketcan adapter
  socketcan_adapter_->send(relay_impl_.getRelayCommandMessage());
  return future;
}

std::future<MvecPopulationReply> MvecRelaySocketcan::get_relay_population()
{
  std::lock_guard<std::mutex> lock(population_mutex_);

  // Abandon any in-flight query, caller's future becomes broken_promise if still waiting
  population_reply_promise_.reset();

  // Create a new promise and get its future
  std::promise<MvecPopulationReply> promise;
  auto future = promise.get_future();
  population_reply_promise_.emplace(std::move(promise));

  // Transmit the population query message via socketcan adapter
  socketcan_adapter_->send(relay_impl_.getPopulationQueryMessage());
  return future;
}

std::optional<MvecFuseStatusMessage> MvecRelaySocketcan::get_last_fuse_status() const
{
  const auto & fuse_status = relay_impl_.get_fuse_status_message();
  if (fuse_status.is_valid()) {
    return fuse_status;
  }
  return std::nullopt;
}

std::optional<MvecRelayStatusMessage> MvecRelaySocketcan::get_last_relay_status() const
{
  const auto & relay_status = relay_impl_.get_relay_status_message();
  if (relay_status.is_valid()) {
    return relay_status;
  }
  return std::nullopt;
}

std::optional<MvecErrorStatusMessage> MvecRelaySocketcan::get_last_error_status() const
{
  const auto & error_status = relay_impl_.get_error_status_message();
  if (error_status.is_valid()) {
    return error_status;
  }
  return std::nullopt;
}

}  // namespace polymath::sygnal
