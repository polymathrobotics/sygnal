// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#ifndef MVEC_LIB__MVEC_RELAY_SOCKETCAN_HPP_
#define MVEC_LIB__MVEC_RELAY_SOCKETCAN_HPP_

#include <future>
#include <memory>
#include <mutex>
#include <queue>

#include "mvec_lib/mvec_relay.hpp"
#include "socketcan_adapter/socketcan_adapter.hpp"

namespace polymath::sygnal
{

class MvecRelaySocketcan
{
public:
  explicit MvecRelaySocketcan(std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter);

  // passthrough to go into socketcan adpater callbacks
  MvecMessageType parse(const socketcan::CanFrame & frame);

  void set_relay_in_command(uint8_t relay_id, uint8_t relay_state);
  void clear_relay();

  // query response infrastructure
  std::future<MvecRelayQueryReply> get_relay_state();
  std::future<MvecRelayCommandReply> send_relay_command();
  std::future<MvecPopulationReply> get_relay_population();

  // automatically updated values
  const std::optional<MvecFuseStatusMessage> & get_last_fuse_status();
  const std::optional<MvecRelayStatusMessage> & get_last_relay_status();
  const std::optional<MvecErrorStatusMessage> & get_last_error_status();

private:
  std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter_;
  MvecRelay relay_impl_;

  std::queue<std::promise<MvecRelayQueryReply>> query_reply_promises_;
  std::queue<std::promise<MvecRelayCommandReply>> command_reply_promises_;
  std::queue<std::promise<MvecPopulationReply>> population_reply_promises_;

  std::mutex promises_mutex_;
};

}  // namespace polymath::sygnal
#endif  // MVEC_LIB__MVEC_RELAY_SOCKETCAN_HPP_
