// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#ifndef MVEC_LIB__MVEC_RELAY_SOCKETCAN_HPP_
#define MVEC_LIB__MVEC_RELAY_SOCKETCAN_HPP_

#include "mvec_lib/mvec_relay.hpp"
#include "socketcan_adapter/socketcan_adapter.hpp"
#include <future>
#include <queue>

namespace polymath::sygnal
{

class MvecRelaySocketcan
{
public:
  MvecRelaySocketcan(std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter);

  // passthrough to go into socketcan adpater callbacks
  MvecMessageType parse(const socketcan::CanFrame & frame);

  // query response infrastructure
  std::future<MvecRelayQueryReply> get_relay_state();
  std::future<MvecRelayCommandReply> send_relay_command();
  std::future<MvecPopulationReply> get_relay_population();

  // automatically updated values
  std::optional<MvecFuseStatusMessage> & const get_last_fuse_status();
  std::optional<MvecRelayStatusMessage> & const get_last_relay_status();
  std::optional<MvecErrorStatusMessage> & const get_last_error_status();
  
private:
  std::shared_ptr<socketcan::SocketcanAdapter> socketcan_adapter_;
  MvecRelay relay_impl_;

  std::promise<MvecRelayQueryReply> query_reply_promise_;
  std::promise<MvecRelayCommandReply> command_reply_promise_;
  std::promise<MvecPopulationReply> population_reply_promise_;
};

}
#endif // MVEC_LIB__MVEC_RELAY_SOCKETCAN_HPP_