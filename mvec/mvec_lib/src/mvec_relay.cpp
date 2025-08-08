#include "mvec_lib/can_bitwork.hpp"
#include "mvec_lib/mvec_relay.hpp"
#include "mvec_lib/j1939_id.hpp"

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

  relay_command_data_.fill(0);
  relay_query_data_.fill(0);
}

bool MvecRelay::parseMessage(const socketcan::CanFrame & frame)
{
  // Evaluate the CAN_ID
  // We only operate on extended IDs
  if(frame.get_id_type() != socketcan::IdType::EXTENDED)
  {
    return;
  }

  // We only operate on data IDs
  if(frame.get_frame_type() != socketcan::FrameType::DATA)
  {
    return;
  }

  // We want to get the 29bit CAN ID
  auto j1939_id = J1939_ID(frame.get_id());

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
      return parseRelayCommmandReply(frame);
    case mvec_msg_ids::population_response:
      return parsePopulationReply(frame);
    case mvec_msg_ids::relay_query_response:
      return parseRelayReply(frame);
    default:
      // Unknown message ID, log or handle as needed
      return false;
  }
}

bool parseRelayCommandReply(const socketcan::CanFrame & frame)
{
  std::array<unsigned char, CAN_MAX_DLC> raw_data = frame.get_data();

  uint8_t command_msg_id = raw_data[1];
  // how do we want to use success?
  uint8_t success = raw_data[2];
  // Error, defaults to grid address otherwise 0xE0, 0xE1, handle later
  uint8_t error = raw_data[3];
  auto data = unpackData<uint16_t>(raw_data, 4*sizeof(unsigned char), mvec_max_relays + mvec_max_high_side_outputs);

  // Find the failure
  for(size_t i = 0; i < mvec_max_relays; ++i)
  {
    relay_command_result_[i] = ((data >> i) & 0x01);
  }

  high_side_output_result_ = ((data >> mvec_max_relays) & 0x01);
}

} // polymath::sygnal
