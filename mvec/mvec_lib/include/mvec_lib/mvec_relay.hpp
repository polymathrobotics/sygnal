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

/// @brief Core MVEC relay control and message parsing implementation
/// Handles J1939 message parsing, command generation, and status tracking for MVEC devices
class MvecRelay
{
public:
  /// @brief Default constructor
  MvecRelay();

  virtual ~MvecRelay() = default;

  /// @brief Parse incoming CAN frame for MVEC messages
  /// @param can_frame CAN frame to parse
  /// @return Message type that was parsed (UNSUPPORTED if not MVEC message)
  MvecMessageType parseMessage(const socketcan::CanFrame & can_frame);

  /// @brief Set relay command for transmission
  /// @param relay_id Relay ID (0-11)
  /// @param relay_state Relay state (0=off, 1=on)
  void set_relay_in_command(uint8_t relay_id, uint8_t relay_state);
  
  /// @brief Set high side output command
  /// @param high_side_output_state High side output state (0=off, 1=on)
  void set_high_side_output_in_command(uint8_t high_side_output_state);
  
  /// @brief Clear all pending relay commands
  void clearRelayCommands();

  /// @brief Generate CAN frame for relay command
  /// @return CAN frame containing relay command
  socketcan::CanFrame getRelayCommandMessage();

  /// @brief Generate CAN frame for relay state query
  /// @return CAN frame containing relay query
  socketcan::CanFrame getRelayQueryMessage();

  /// @brief Generate CAN frame for population query
  /// @return CAN frame containing population query
  socketcan::CanFrame getPopulationQueryMessage();

  /// @brief Get fuse status message object
  /// @return Reference to fuse status message (check is_valid() before use)
  const MvecFuseStatusMessage & get_fuse_status_message() const
  {
    return fuse_status_message_;
  }

  /// @brief Get relay status message object  
  /// @return Reference to relay status message (check is_valid() before use)
  const MvecRelayStatusMessage & get_relay_status_message() const
  {
    return relay_status_message_;
  }

  /// @brief Get error status message object
  /// @return Reference to error status message (check is_valid() before use)
  const MvecErrorStatusMessage & get_error_status_message() const
  {
    return error_status_message_;
  }

  /// @brief Get last received relay command reply
  /// @return Reference to relay command reply (check is_valid() before use)
  const MvecRelayCommandReply & get_last_relay_command_reply() const
  {
    return relay_command_reply_;
  }

  /// @brief Get last received relay query reply
  /// @return Reference to relay query reply (check is_valid() before use)
  const MvecRelayQueryReply & get_last_relay_query_reply() const
  {
    return relay_query_reply_;
  }

  /// @brief Get last received population reply
  /// @return Reference to population reply (check is_valid() before use)
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
