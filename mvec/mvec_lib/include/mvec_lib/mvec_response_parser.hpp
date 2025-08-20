// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#ifndef MVEC_LIB__MVEC_RESPONSE_PARSER_HPP_
#define MVEC_LIB__MVEC_RESPONSE_PARSER_HPP_

#include <stdint.h>

#include <array>

#include "mvec_lib/can_bitwork.hpp"
#include "mvec_lib/j1939_id.hpp"
#include "mvec_lib/mvec_constants.hpp"
#include "mvec_lib/mvec_population_reply.hpp"
#include "mvec_lib/mvec_relay_command_reply.hpp"
#include "mvec_lib/mvec_relay_query_reply.hpp"
#include "socketcan_adapter/can_frame.hpp"

namespace polymath::sygnal
{

class MvecResponseParser
{
public:
  MvecResponseParser(
    uint8_t source_address = MvecProtocol::DEFAULT_SOURCE_ADDRESS,
    uint8_t my_address = MvecProtocol::DEFAULT_SELF_ADDRESS);

  bool parse(const socketcan::CanFrame & frame);

  uint8_t get_message_id() const
  {
    return message_id_;
  }

  const MvecRelayCommandReply & get_relay_command_reply() const
  {
    return relay_command_reply_;
  }

  const MvecRelayQueryReply & get_relay_query_reply() const
  {
    return relay_query_reply_;
  }

  const MvecPopulationReply & get_population_reply() const
  {
    return population_reply_;
  }

  bool is_valid() const
  {
    return is_valid_;
  }

private:
  J1939_ID expected_id_;
  uint8_t message_id_;
  MvecRelayCommandReply relay_command_reply_;
  MvecRelayQueryReply relay_query_reply_;
  MvecPopulationReply population_reply_;
  bool is_valid_;
};

}  // namespace polymath::sygnal

#endif  // MVEC_LIB__MVEC_RESPONSE_PARSER_HPP_
