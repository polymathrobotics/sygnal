#include "mvec_lib/mvec_relay.hpp"
#include "mvec_lib/j1939_id.hpp"

namespace polymath::sygnal
{

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

  // Supported message receipts
  // broadcast, statuses PF: FF
  // fuse status, PS 01+pgn_base_value_, mvec_source_address
  // relay status, PS 02+pgn_base_value_, mvec_source_address
  // error status, PS 03+pgn_base_value_, mvec_source_address

  // specific/replies, PF: EF, PS: our "address" (00), mvec_source_address
  // Message IDs as defined in mvec_msg_ids, we should handle separately
}

} // polymath::sygnal
