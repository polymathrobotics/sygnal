// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#include "mvec_lib/mvec_relay.hpp"

#include "mvec_lib/core/can_bitwork.hpp"

namespace polymath::sygnal
{

MvecRelay::MvecRelay()
: mvec_specific_command_id_(
    MvecProtocol::DEFAULT_PRIORITY,
    MvecProtocol::DEFAULT_DATA_PAGE,
    MvecProtocol::QUERY_PDU,
    mvec_source_address_,
    self_address_)
, fuse_status_message_(mvec_source_address_, pgn_base_value_)
, relay_status_message_(mvec_source_address_, pgn_base_value_)
, error_status_message_(mvec_source_address_, pgn_base_value_)
, relay_command_reply_(mvec_source_address_, self_address_)
, relay_query_reply_(mvec_source_address_, self_address_)
, population_reply_(mvec_source_address_, self_address_)
{
  relay_command_data_.fill(0xFF);
  query_data_.fill(0xFF);

  relay_command_data_[MvecMessageStructure::MSG_ID_BYTE] = MvecMessageIds::RELAY_COMMAND_WITH_FEEDBACK;
  relay_command_data_[MvecMessageStructure::GRID_ID_BYTE] = MvecProtocol::DEFAULT_GRID_ADDRESS;
}

void MvecRelay::set_relay_in_command(uint8_t relay_id, uint8_t relay_state)
{
  if (relay_id >= 12) {
    // invalid relay id
    return;
  }

  if (relay_state > MvecValueLimits::MAX_RELAY_STATE_VALUE) {
    // invalid relay state
    return;
  }

  uint8_t start_bit = MvecMessageStructure::RELAY_DATA_START_BIT;
  packData<uint8_t>(
    relay_state,
    relay_command_data_,
    start_bit + relay_id * MvecMessageStructure::BITS_PER_RELAY,
    MvecMessageStructure::BITS_PER_RELAY);
}

void MvecRelay::set_high_side_output_in_command(uint8_t high_side_output_state)
{
  if (high_side_output_state > MvecValueLimits::MAX_HIGH_SIDE_STATE_VALUE) {
    return;
  }

  uint8_t start_bit =
    MvecMessageStructure::RELAY_DATA_START_BIT + MvecHardware::MAX_NUMBER_RELAYS * MvecMessageStructure::BITS_PER_RELAY;
  packData<uint8_t>(high_side_output_state, relay_command_data_, start_bit, MvecMessageStructure::HIGH_SIDE_BITS);
}

void MvecRelay::clearRelayCommands()
{
  relay_command_data_.fill(0xFF);
}

socketcan::CanFrame MvecRelay::getRelayCommandMessage()
{
  socketcan::CanFrame frame;

  frame.set_can_id(mvec_specific_command_id_.get_can_id());
  frame.set_id_as_extended();
  frame.set_data(relay_command_data_);
  frame.set_len(CAN_MAX_DLC);

  return frame;
}

socketcan::CanFrame MvecRelay::getRelayQueryMessage()
{
  socketcan::CanFrame frame;

  query_data_[MvecMessageStructure::MSG_ID_BYTE] = MvecMessageIds::RELAY_STATE_QUERY;
  query_data_[MvecMessageStructure::GRID_ID_BYTE] = MvecProtocol::DEFAULT_GRID_ADDRESS;

  frame.set_can_id(mvec_specific_command_id_.get_can_id());
  frame.set_id_as_extended();
  frame.set_data(query_data_);
  frame.set_len(CAN_MAX_DLC);

  return frame;
}

socketcan::CanFrame MvecRelay::getPopoulationQueryMessage()
{
  socketcan::CanFrame frame;

  query_data_[MvecMessageStructure::MSG_ID_BYTE] = MvecMessageIds::POPULATION_QUERY;
  query_data_[MvecMessageStructure::GRID_ID_BYTE] = MvecProtocol::DEFAULT_GRID_ADDRESS;

  frame.set_can_id(mvec_specific_command_id_.get_can_id());
  frame.set_id_as_extended();
  frame.set_data(query_data_);
  frame.set_len(CAN_MAX_DLC);

  return frame;
}

// State methods (current actual state)

MvecMessageType MvecRelay::parseMessage(const socketcan::CanFrame & frame)
{
  // Evaluate the CAN_ID
  // We only operate on extended IDs
  if (frame.get_id_type() != socketcan::IdType::EXTENDED) {
    return MvecMessageType::UNSUPPORTED;
  }

  // We only operate on data IDs
  if (frame.get_frame_type() != socketcan::FrameType::DATA) {
    return MvecMessageType::UNSUPPORTED;
  }

  // Try each message parser - they will validate their own IDs internally
  if (fuse_status_message_.parse(frame)) {
    return MvecMessageType::FUSE_STATUS;
  }

  if (relay_status_message_.parse(frame)) {
    return MvecMessageType::RELAY_STATUS;
  }

  if (error_status_message_.parse(frame)) {
    return MvecMessageType::ERROR_STATUS;
  }

  if (relay_command_reply_.parse(frame)) {
    return MvecMessageType::RELAY_COMMAND_RESPONSE;
  }

  if (relay_query_reply_.parse(frame)) {
    return MvecMessageType::RELAY_QUERY_RESPONSE;
  }

  if (population_reply_.parse(frame)) {
    return MvecMessageType::POPULATION_RESPONSE;
  }

  return MvecMessageType::UNSUPPORTED;
}

}  // namespace polymath::sygnal
