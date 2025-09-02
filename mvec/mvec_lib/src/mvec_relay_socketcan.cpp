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

#include <iostream>
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
  MvecMessageType message_type = relay_impl_.parseMessage(frame);

  // Check if we received expected response types and fulfill promises
  std::lock_guard<std::mutex> lock(promises_mutex_);

  switch (message_type) {
    case MvecMessageType::RELAY_QUERY_RESPONSE: {
      const auto & reply = relay_impl_.get_last_relay_query_reply();
      if (reply.is_valid() && !query_reply_promises_.empty()) {
        // Get the oldest waiting promise
        auto promise = std::move(query_reply_promises_.front());
        query_reply_promises_.pop();

        // Fulfill the promise
        promise.set_value(reply);
      }
      break;
    }
    case MvecMessageType::RELAY_COMMAND_RESPONSE: {
      const auto & reply = relay_impl_.get_last_relay_command_reply();
      if (reply.is_valid() && !command_reply_promises_.empty()) {
        auto promise = std::move(command_reply_promises_.front());
        command_reply_promises_.pop();
        promise.set_value(reply);
      }
      break;
    }
    case MvecMessageType::POPULATION_RESPONSE: {
      const auto & reply = relay_impl_.get_last_population_reply();
      if (reply.is_valid() && !population_reply_promises_.empty()) {
        auto promise = std::move(population_reply_promises_.front());
        population_reply_promises_.pop();
        promise.set_value(reply);
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
  // Get the query message from the relay implementation
  /// TODO: (zeerek) Set invalid for received message
  auto query_frame = relay_impl_.getRelayQueryMessage();

  // Create a new promise and get its future
  std::promise<MvecRelayQueryReply> promise;
  auto future = promise.get_future();

  // Add promise to the queue with thread safety
  {
    std::lock_guard<std::mutex> lock(promises_mutex_);
    query_reply_promises_.push(std::move(promise));
  }

  // Transmit the query message via socketcan adapter
  socketcan_adapter_->send(query_frame);

  return future;
}

std::future<MvecRelayCommandReply> MvecRelaySocketcan::send_relay_command()
{
  // Get the command message from the relay implementation
  /// TODO: (zeerek) Set invalid for received message
  auto command_frame = relay_impl_.getRelayCommandMessage();

  // Create a new promise and get its future
  std::promise<MvecRelayCommandReply> promise;
  auto future = promise.get_future();

  // Add promise to the queue with thread safety
  {
    std::lock_guard<std::mutex> lock(promises_mutex_);
    command_reply_promises_.push(std::move(promise));
  }

  // Transmit the command message via socketcan adapter
  auto send_result = socketcan_adapter_->send(command_frame);
  if (send_result) {
    std::cout << send_result.value() << std::endl;
  }

  return future;
}

std::future<MvecPopulationReply> MvecRelaySocketcan::get_relay_population()
{
  // Get the population query message from the relay implementation
  /// TODO: (zeerek) Set invalid for received message
  auto population_frame = relay_impl_.getPopulationQueryMessage();

  // Create a new promise and get its future
  std::promise<MvecPopulationReply> promise;
  auto future = promise.get_future();

  // Add promise to the queue with thread safety
  {
    std::lock_guard<std::mutex> lock(promises_mutex_);
    population_reply_promises_.push(std::move(promise));
  }

  // Transmit the population query message via socketcan adapter
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
