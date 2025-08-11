#include "mvec_lib/mvec_relay.hpp"
#include "mvec_lib/can_bitwork.hpp"

namespace polymath::sygnal
{

MvecRelay::MvecRelay()
  : mvec_fuse_status_id_(MvecProtocol::DEFAULT_PRIORITY, MvecProtocol::DEFAULT_DATA_PAGE, MvecProtocol::BROADCAST_PDU,
                         0x01 + pgn_base_value_, mvec_source_address_),
    mvec_relay_status_id_(MvecProtocol::DEFAULT_PRIORITY, MvecProtocol::DEFAULT_DATA_PAGE, MvecProtocol::BROADCAST_PDU,
                          0x02 + pgn_base_value_, mvec_source_address_),
    mvec_error_status_id_(MvecProtocol::DEFAULT_PRIORITY, MvecProtocol::DEFAULT_DATA_PAGE, MvecProtocol::BROADCAST_PDU,
                          0x03 + pgn_base_value_, mvec_source_address_),
    mvec_specific_command_id_(MvecProtocol::DEFAULT_PRIORITY, MvecProtocol::DEFAULT_DATA_PAGE, MvecProtocol::SPECIFIC_PDU,
                              mvec_source_address_, my_address_),
    mvec_specific_response_id_(MvecProtocol::DEFAULT_PRIORITY, MvecProtocol::DEFAULT_DATA_PAGE, MvecProtocol::SPECIFIC_PDU,
                               my_address_, mvec_source_address_)
{
  // Initialize relay states to false (not populated)
  relay_state_feedback_.fill(false);
  relay_population_state_.fill(false);
  fuse_population_state_.fill(false);

  relay_command_data_.fill(0xFF);
  relay_query_data_.fill(0xFF);

  // Set msg IDs and grid id and 0xFF for all others
  relay_command_data_[MvecMessageStructure::MSG_ID_BYTE] = MvecMessageIds::RELAY_COMMAND_WITH_FEEDBACK;
  relay_command_data_[MvecMessageStructure::GRID_ID_BYTE] = MvecProtocol::DEFAULT_SELF_ADDRESS;
  relay_query_data_[MvecMessageStructure::MSG_ID_BYTE] = MvecMessageIds::RELAY_STATE_QUERY;
  relay_query_data_[MvecMessageStructure::GRID_ID_BYTE] = MvecProtocol::DEFAULT_SELF_ADDRESS;
}

void MvecRelay::set_relay_in_command(uint8_t relay_id, uint8_t relay_state)
{
  if (!is_relay_populated(relay_id))
  {
    return;
  }

  if(relay_id >= MvecHardware::MAX_RELAYS)
  {
    // invalid relay id
    return;
  }

  if(relay_state > MvecValueLimits::MAX_RELAY_STATE_VALUE)
  {
    // invalid relay state
    return;
  }

  uint8_t start_bit = MvecMessageStructure::RELAY_DATA_START_BIT;
  packData<uint8_t>(relay_state, relay_command_data_, start_bit + relay_id * MvecMessageStructure::BITS_PER_RELAY, MvecMessageStructure::BITS_PER_RELAY);
}

void MvecRelay::set_high_side_output_in_command(uint8_t high_side_output_state)
{

  if(high_side_output_state > MvecValueLimits::MAX_HIGH_SIDE_STATE_VALUE)
  {
    return;
  }


  uint8_t start_bit = MvecMessageStructure::RELAY_DATA_START_BIT + MvecHardware::MAX_RELAYS * MvecMessageStructure::BITS_PER_RELAY;
  packData<uint8_t>(high_side_output_state, relay_command_data_, start_bit, MvecMessageStructure::HIGH_SIDE_BITS);
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

// State methods (current actual state)
bool MvecRelay::get_relay_state(const uint8_t relay_id)
{
  if (relay_id >= MvecHardware::MAX_RELAYS) {
    return false;
  }
  return relay_state_feedback_[relay_id];
}

bool MvecRelay::get_high_side_output_state() const
{
  return high_side_output_feedback_;
}


// Status methods (detailed status information with error codes)
MvecFuseStatus MvecRelay::get_fuse_status(const uint8_t fuse_id)
{
  return fuse_status_message_.get_fuse_status(fuse_id);
}

MvecRelayStatus MvecRelay::get_relay_status(const uint8_t relay_id)
{
  return relay_status_message_.get_relay_status(relay_id);
}

uint16_t MvecRelay::get_error_bits()
{
  return error_status_message_.get_error_bits();
}

bool MvecRelay::has_error(MvecErrorType error_type)
{
  return error_status_message_.has_error(error_type);
}

uint8_t MvecRelay::get_error_grid_address()
{
  return error_status_message_.get_grid_address();
}

// Population methods
bool MvecRelay::is_relay_populated(const uint8_t relay_id)
{
  if (relay_id >= MvecHardware::MAX_RELAYS) {
    return false;
  }
  return relay_population_state_[relay_id];
}

bool MvecRelay::is_fuse_populated(const uint8_t fuse_id)
{
  if (fuse_id >= MvecHardware::MAX_FUSES) {
    return false;
  }
  return fuse_population_state_[fuse_id];
}

// Get status message objects
const MvecFuseStatusMessage& MvecRelay::get_fuse_status_message() const
{
  return fuse_status_message_;
}

const MvecRelayStatusMessage& MvecRelay::get_relay_status_message() const
{
  return relay_status_message_;
}

const MvecErrorStatusMessage& MvecRelay::get_error_status_message() const
{
  return error_status_message_;
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
    bool result = parseFuseStatus(frame);
    fuse_status_message_.parse(frame);
    return result;
  }

  if (j1939_id == mvec_relay_status_id_)
  {
    bool result = parseRelayStatus(frame);
    relay_status_message_.parse(frame);
    return result;
  }

  if (j1939_id == mvec_error_status_id_)
  {
    bool result = parseErrorStatus(frame);
    error_status_message_.parse(frame);
    return result;
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
  uint8_t message_id = raw_data[MvecMessageStructure::MSG_ID_BYTE];

  switch(message_id)
  {
    case MvecMessageIds::RESPONSE:
      return parseRelayCommandReply(frame);
    case MvecMessageIds::POPULATION_RESPONSE:
      return parsePopulationReply(frame);
    case MvecMessageIds::RELAY_QUERY_RESPONSE:
      return parseRelayReply(frame);
    default:
      // Unknown message ID, log or handle as needed
      return false;
  }
}

bool MvecRelay::parseRelayCommandReply(const socketcan::CanFrame & frame)
{
  std::array<unsigned char, CAN_MAX_DLC> raw_data = frame.get_data();

  uint8_t command_msg_id = raw_data[MvecMessageStructure::GRID_ID_BYTE];
  // how do we want to use success?
  uint8_t success = raw_data[MvecParsingPositions::COMMAND_REPLY_SUCCESS_BYTE];
  // Error, defaults to grid address otherwise 0xE0, 0xE1, handle later
  uint8_t error = raw_data[MvecParsingPositions::COMMAND_REPLY_ERROR_BYTE];
  auto data = unpackData<uint16_t>(raw_data, MvecParsingPositions::COMMAND_REPLY_DATA_START_BYTE * sizeof(unsigned char), MvecHardware::MAX_RELAYS + MvecHardware::MAX_HIGH_SIDE_OUTPUTS);

  // Record the state (put into a struct in the future)
  for(size_t i = 0; i < MvecHardware::MAX_RELAYS; ++i)
  {
    relay_command_result_[i] = ((data >> i) & 0x01);
  }
  high_side_output_command_result_ = ((data >> MvecHardware::MAX_RELAYS) & 0x01);
  return true;
}

bool MvecRelay::parseRelayReply(const socketcan::CanFrame & frame)
{
  std::array<unsigned char, CAN_MAX_DLC> raw_data = frame.get_data();

  // Parse relays, Bytes 2&3
  auto data = unpackData<uint16_t>(raw_data, MvecParsingPositions::RELAY_REPLY_STATE_START_BYTE * sizeof(unsigned char), MvecHardware::MAX_RELAYS + MvecHardware::MAX_HIGH_SIDE_OUTPUTS);
  for(size_t i = 0; i < MvecHardware::MAX_RELAYS; ++i)
  {
    relay_state_feedback_[i] = ((data >> i) & 0x01);
  }
  high_side_output_feedback_ = ((data >> MvecHardware::MAX_RELAYS) & 0x01);

  // Parse Defaults, Bytes 4&5
  data = unpackData<uint16_t>(raw_data, MvecParsingPositions::RELAY_REPLY_DEFAULT_START_BYTE * sizeof(unsigned char), MvecHardware::MAX_RELAYS + MvecHardware::MAX_HIGH_SIDE_OUTPUTS);
  for(size_t i = 0; i < MvecHardware::MAX_RELAYS; ++i)
  {
    relay_default_state_[i] = ((data >> i) & 0x01);
  }
  high_side_output_default_state_ = ((data >> MvecHardware::MAX_RELAYS) & 0x01);

  return true;
}

bool MvecRelay::parsePopulationReply(const socketcan::CanFrame & frame)
{
  std::array<unsigned char, CAN_MAX_DLC> raw_data = frame.get_data();

  // Parse fuses, Bytes 2-4
  auto data = unpackData<uint32_t>(raw_data, MvecParsingPositions::POPULATION_FUSE_START_BYTE * sizeof(unsigned char), MvecHardware::MAX_FUSES);
  for(size_t i = 0; i < MvecHardware::MAX_FUSES; ++i)
  {
    fuse_population_state_[i] = ((data >> i) & 0x01);
  }
  // Parse Defaults, Bytes 5&6
  data = unpackData<uint32_t>(raw_data, MvecParsingPositions::POPULATION_RELAY_START_BYTE * sizeof(unsigned char), MvecHardware::MAX_RELAYS + MvecHardware::MAX_HIGH_SIDE_OUTPUTS);
  for(size_t i = 0; i < MvecHardware::MAX_RELAYS; ++i)
  {
    relay_population_state_[i] = ((data >> i) & 0x01);
  }
  high_side_output_population_state_ = ((data >> MvecHardware::MAX_RELAYS) & 0x01);

  return true;
}

bool MvecRelay::parseFuseStatus(const socketcan::CanFrame & frame)
{
  // Let the status message object handle the parsing
  return fuse_status_message_.parse(frame);
}

bool MvecRelay::parseRelayStatus(const socketcan::CanFrame & frame)
{
  // Let the status message object handle the parsing
  return relay_status_message_.parse(frame);
}

bool MvecRelay::parseErrorStatus(const socketcan::CanFrame & frame)
{
  // Let the status message object handle the parsing
  return error_status_message_.parse(frame);
}

} // polymath::sygnal
