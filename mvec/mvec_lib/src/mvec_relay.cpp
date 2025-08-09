#include "mvec_lib/mvec_relay.hpp"
#include "mvec_lib/can_bitwork.hpp"

namespace polymath::sygnal
{

MvecRelay::MvecRelay()
  : mvec_fuse_status_id_(6, 0, mvec_broadcast_pdu, 0x01 + pgn_base_value_, mvec_source_address_),
    mvec_relay_status_id_(6, 0, mvec_broadcast_pdu, 0x02 + pgn_base_value_, mvec_source_address_),
    mvec_error_status_id_(6, 0, mvec_broadcast_pdu, 0x03 + pgn_base_value_, mvec_source_address_),
    mvec_specific_command_id_(6, 0, mvec_specific_pdu, mvec_source_address_, my_address_),
    mvec_specific_response_id_(6, 0, mvec_specific_pdu, my_address_, mvec_source_address_)
{
  // Initialize relay and fuse states to false (not populated)
  relay_state_feedback_.fill(false);
  fuse_state_feedback_.fill(false);
  relay_population_state_.fill(false);
  fuse_population_state_.fill(false);

  relay_command_data_.fill(0xFF);
  relay_query_data_.fill(0xFF);

  // Set msg IDs and grid id and 0xFF for all others
  relay_command_data_[0] = mvec_msg_ids::relay_command_with_feedback;
  relay_command_data_[1] = 0x00;
  relay_query_data_[0] = mvec_msg_ids::relay_state_query;
  relay_query_data_[1] = 0x00;
}

void MvecRelay::set_relay_in_command(uint8_t relay_id, uint8_t relay_state)
{
  if (!is_relay_populated(relay_id))
  {
    return;
  }

  if(relay_id >= mvec_max_relays)
  {
    // invalid relay id
    return;
  }

  if(relay_state > 0x01)
  {
    // invalid relay state
    return;
  }

  uint8_t start_bit = 2 * sizeof(unsigned char);
  packData<uint8_t>(relay_state, relay_command_data_, start_bit + relay_id * 2, 2);
}

void MvecRelay::set_high_side_output_in_command(uint8_t high_side_output_state)
{

  if(high_side_output_state > 0x01)
  {
    return;
  }


  uint8_t start_bit = (2 * sizeof(unsigned char)) + mvec_max_relays * 2;
  packData<uint8_t>(high_side_output_state, relay_command_data_, start_bit, 2);
}

socketcan::CanFrame MvecRelay::getRelayCommandMessage()
{
  socketcan::CanFrame frame;

  frame.set_can_id(mvec_specific_command_id_.get_can_id());
  frame.set_id_as_extended();
  frame.set_data(relay_command_data_);

  return frame;
}

socketcan::CanFrame MvecRelay::getRelayQueryMessage()
{
  socketcan::CanFrame frame;

  frame.set_can_id(mvec_specific_command_id_.get_can_id());
  frame.set_id_as_extended();
  frame.set_data(relay_query_data_);

  return frame;
}

bool MvecRelay::get_relay_state(const uint8_t relay_id)
{
  return relay_state_feedback_[relay_id];
}

uint8_t MvecRelay::get_fuse_state(const uint8_t fuse_id)
{
  return fuse_state_feedback_[fuse_id];
}

bool MvecRelay::is_relay_populated(const uint8_t relay_id)
{
  return relay_population_state_[relay_id];
}

bool MvecRelay::is_fuse_populated(const uint8_t fuse_id)
{
  return fuse_population_state_[fuse_id];
}


bool MvecRelay::parseMessage(const socketcan::CanFrame & frame)
{
  // Evaluate the CAN_ID
  // We only operate on extended IDs
  if(frame.get_id_type() != socketcan::IdType::EXTENDED)
  {
    return false;
  }

  // We only operate on data IDs
  if(frame.get_frame_type() != socketcan::FrameType::DATA)
  {
    return false;
  }

  // We want to get the 29bit CAN ID
  auto j1939_id = J1939_ID(frame.get_id());

  // TODO: Send the timepoint, we want to record timepoint for each message def
  if(j1939_id == mvec_fuse_status_id_)
  {
    return parseFuseStatus(frame);
  }
  
  if (j1939_id == mvec_relay_status_id_)
  {
    return parseRelayStatus(frame);
  }
  
  if (j1939_id == mvec_error_status_id_)
  {
    return parseErrorStatus(frame);
  }
  
  if (j1939_id == mvec_specific_response_id_)
  {
    return parseSpecificResponse(frame);
  }
 
  return false;
}

bool MvecRelay::parseSpecificResponse(const socketcan::CanFrame & frame)
{
  // Handle specific response parsing logic
  // This is a placeholder for the actual implementation

  // Get message ID, uint8_t first 2 bytes of data
  std::array<unsigned char, CAN_MAX_DLC> raw_data = frame.get_data();
  uint8_t message_id = reinterpret_cast<uint8_t>(raw_data[0]);

  switch(message_id)
  {
    case mvec_msg_ids::response:
      return parseRelayCommandReply(frame);
    case mvec_msg_ids::population_response:
      return parsePopulationReply(frame);
    case mvec_msg_ids::relay_query_response:
      return parseRelayReply(frame);
    default:
      // Unknown message ID, log or handle as needed
      return false;
  }
}

bool MvecRelay::parseRelayCommandReply(const socketcan::CanFrame & frame)
{
  std::array<unsigned char, CAN_MAX_DLC> raw_data = frame.get_data();

  uint8_t command_msg_id = raw_data[1];
  // how do we want to use success?
  uint8_t success = raw_data[2];
  // Error, defaults to grid address otherwise 0xE0, 0xE1, handle later
  uint8_t error = raw_data[3];
  auto data = unpackData<uint16_t>(raw_data, 4*sizeof(unsigned char), mvec_max_relays + mvec_max_high_side_outputs);

  // Record the state (put into a struct in the future)
  for(size_t i = 0; i < mvec_max_relays; ++i)
  {
    relay_command_result_[i] = ((data >> i) & 0x01);
  }
  high_side_output_command_result_ = ((data >> mvec_max_relays) & 0x01);
  return true;
}

bool MvecRelay::parseRelayReply(const socketcan::CanFrame & frame)
{
  std::array<unsigned char, CAN_MAX_DLC> raw_data = frame.get_data();

  // Parse relays, Bytes 2&3
  auto data = unpackData<uint16_t>(raw_data, 2*sizeof(unsigned char), mvec_max_relays + mvec_max_high_side_outputs);
  for(size_t i = 0; i < mvec_max_relays; ++i)
  {
    relay_state_feedback_[i] = ((data >> i) & 0x01);
  }
  high_side_output_feedback_ = ((data >> mvec_max_relays) & 0x01);

  // Parse Defaults, Bytes 4&5
  data = unpackData<uint16_t>(raw_data, 4*sizeof(unsigned char), mvec_max_relays + mvec_max_high_side_outputs);
  for(size_t i = 0; i < mvec_max_relays; ++i)
  {
    relay_default_state_[i] = ((data >> i) & 0x01);
  }
  high_side_output_default_state_ = ((data >> mvec_max_relays) & 0x01);

  return true;
}

bool MvecRelay::parsePopulationReply(const socketcan::CanFrame & frame)
{
  std::array<unsigned char, CAN_MAX_DLC> raw_data = frame.get_data();

  // Parse fuses, Bytes 2-4
  auto data = unpackData<uint32_t>(raw_data, 2*sizeof(unsigned char), mvec_max_fuses);
  for(size_t i = 0; i < mvec_max_fuses; ++i)
  {
    fuse_population_state_[i] = ((data >> i) & 0x01);
  }
  // Parse Defaults, Bytes 5&6
  data = unpackData<uint32_t>(raw_data, 5*sizeof(unsigned char), mvec_max_relays + mvec_max_high_side_outputs);
  for(size_t i = 0; i < mvec_max_relays; ++i)
  {
    relay_population_state_[i] = ((data >> i) & 0x01);
  }
  high_side_output_population_state_ = ((data >> mvec_max_relays) & 0x01);

  return true;
}

bool MvecRelay::parseFuseStatus(const socketcan::CanFrame & frame)
{
  std::array<unsigned char, CAN_MAX_DLC> raw_data = frame.get_data();

  // 2 bits per fuse
  uint8_t len_data_bits = mvec_max_fuses * 2;

  // Parse bytes 1-7
  auto data = unpackData<uint64_t>(raw_data, 1*sizeof(unsigned char), len_data_bits);

  for(size_t i = 0; i< mvec_max_fuses; i++)
  {
    fuse_state_feedback_[i] = (data >> i*2) & 0x03;
  }

  return true;
}

bool MvecRelay::parseRelayStatus(const socketcan::CanFrame & frame)
{
  std::array<unsigned char, CAN_MAX_DLC> raw_data = frame.get_data();

  // 2 bits per fuse
  uint8_t len_data_bits = mvec_max_relays * 4;

  // Parse bytes 1-7
  auto data = unpackData<uint64_t>(raw_data, 1*sizeof(unsigned char), len_data_bits);

  for(size_t i = 0; i< mvec_max_fuses; i++)
  {
    relay_status_feedback_[i] = (data >> i*4) & 0x0F;
  }

  return true;
}

bool MvecRelay::parseErrorStatus(const socketcan::CanFrame & frame)
{
  std::array<unsigned char, CAN_MAX_DLC> raw_data = frame.get_data();

  error_bits_ = unpackData<uint64_t>(raw_data, 1*sizeof(unsigned char), num_error_bits);
  return true;
}

} // polymath::sygnal
