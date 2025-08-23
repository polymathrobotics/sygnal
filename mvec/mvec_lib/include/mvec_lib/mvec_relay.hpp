// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#ifndef MVEC_LIB__MVEC_RELAY_HPP_
#define MVEC_LIB__MVEC_RELAY_HPP_

#include <stdint.h>

#include <array>
#include <chrono>

#include "mvec_lib/core/j1939_id.hpp"
#include "mvec_lib/core/mvec_constants.hpp"
#include "mvec_lib/core/mvec_status_messages.hpp"
#include "mvec_lib/responses/mvec_population_reply.hpp"
#include "mvec_lib/responses/mvec_relay_command_reply.hpp"
#include "mvec_lib/responses/mvec_relay_query_reply.hpp"
#include "socketcan_adapter/can_frame.hpp"
#include "socketcan_adapter/socketcan_adapter.hpp"

namespace polymath::sygnal
{

class MvecRelay
{
public:
  MvecRelay();

  virtual ~MvecRelay() = default;

  // return False if message is not relevant
  MvecMessageType parseMessage(const socketcan::CanFrame & can_frame);

  void set_relay_in_command(uint8_t relay_id, uint8_t relay_state);
  void set_high_side_output_in_command(uint8_t high_side_output_state);
  void clearRelayCommands();

  socketcan::CanFrame getRelayCommandMessage();

  socketcan::CanFrame getRelayQueryMessage();

  socketcan::CanFrame getPopulationQueryMessage();

  // Get message objects - all data access goes through these
  const MvecFuseStatusMessage & get_fuse_status_message() const
  {
    return fuse_status_message_;
  }

  const MvecRelayStatusMessage & get_relay_status_message() const
  {
    return relay_status_message_;
  }

  const MvecErrorStatusMessage & get_error_status_message() const
  {
    return error_status_message_;
  }

  const MvecRelayCommandReply & get_last_relay_command_reply() const
  {
    return relay_command_reply_;
  }

  const MvecRelayQueryReply & get_last_relay_query_reply() const
  {
    return relay_query_reply_;
  }

  const MvecPopulationReply & get_last_population_reply() const
  {
    return population_reply_;
  }

private:
  /// TODO: (zeerek) Make these queryable and configurable and update lookup
  // For now these are defaults
  const uint8_t mvec_source_address_ = MvecProtocol::DEFAULT_SOURCE_ADDRESS;
  const uint8_t pgn_base_value_ = MvecProtocol::DEFAULT_PGN_BASE_VALUE;
  const uint8_t self_address_ = MvecProtocol::DEFAULT_SELF_ADDRESS;

  // Command generation only - not used for parsing
  J1939_ID mvec_specific_command_id_;

  // Message objects - all data is stored here
  MvecFuseStatusMessage fuse_status_message_;
  MvecRelayStatusMessage relay_status_message_;
  MvecErrorStatusMessage error_status_message_;

  // Response message objects
  MvecRelayCommandReply relay_command_reply_;
  MvecRelayQueryReply relay_query_reply_;
  MvecPopulationReply population_reply_;

  std::array<unsigned char, CAN_MAX_DLC> relay_command_data_;
  std::array<unsigned char, CAN_MAX_DLC> query_data_;
};

}  // namespace polymath::sygnal

#endif  // MVEC_LIB__MVEC_RELAY_HPP_
