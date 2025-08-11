#ifndef MVEC_LIB__MVEC_RELAY_HPP_
#define MVEC_LIB__MVEC_RELAY_HPP_

#include <stdint.h>
#include <array>
#include <chrono>
#include "socketcan_adapter/can_frame.hpp"
#include "mvec_lib/j1939_id.hpp"
#include "mvec_lib/mvec_constants.hpp"
#include "mvec_lib/mvec_status_messages.hpp"

namespace polymath::sygnal
{

class MvecRelay
{
public:
  MvecRelay();

  virtual ~MvecRelay() = default;

  // return False if message is not relevant
  bool parseMessage(const socketcan::CanFrame & can_frame);

  void set_relay_in_command(uint8_t relay_id, uint8_t relay_state);
  void set_high_side_output_in_command(uint8_t high_side_output_state);

  socketcan::CanFrame getRelayCommandMessage();

  socketcan::CanFrame getRelayQueryMessage();

  // State methods (current actual state)
  bool get_relay_state(const uint8_t relay_id);
  bool get_high_side_output_state() const;

  // Status methods (detailed status information with error codes)
  MvecFuseStatus get_fuse_status(const uint8_t fuse_id);
  MvecRelayStatus get_relay_status(const uint8_t relay_id);
  uint16_t get_error_bits();
  bool has_error(MvecErrorType error_type);
  uint8_t get_error_grid_address();

  // Population methods
  bool is_relay_populated(const uint8_t relay_id);
  bool is_fuse_populated(const uint8_t fuse_id);

  // Get status message objects
  const MvecFuseStatusMessage& get_fuse_status_message() const;
  const MvecRelayStatusMessage& get_relay_status_message() const;
  const MvecErrorStatusMessage& get_error_status_message() const;

  // TODO: Add set_population and get_population_command
  // TODO: Add set_default and get_default_command

private:
  bool parseSpecificResponse(const socketcan::CanFrame & frame);
  bool parseRelayReply(const socketcan::CanFrame & frame);
  bool parsePopulationReply(const socketcan::CanFrame & frame);
  bool parseRelayCommandReply(const socketcan::CanFrame & frame);
  bool parseFuseStatus(const socketcan::CanFrame & frame);
  bool parseRelayStatus(const socketcan::CanFrame & frame);
  bool parseErrorStatus(const socketcan::CanFrame & frame);

  // TODO: Make these queryable and configurable and update lookup
  // For now these are defaults
  const uint8_t mvec_source_address_ = MvecProtocol::DEFAULT_SOURCE_ADDRESS;
  const uint8_t pgn_base_value_ = MvecProtocol::DEFAULT_PGN_BASE_VALUE;
  const uint8_t my_address_ = MvecProtocol::DEFAULT_SELF_ADDRESS;

  // Supported message receipts
  // broadcast, statuses PF: FF
  // fuse status, PS 01+pgn_base_value_, mvec_source_address
  // relay status, PS 02+pgn_base_value_, mvec_source_address
  // error status, PS 03+pgn_base_value_, mvec_source_address

  // specific/replies, PF: EF, PS: our "address" (00), mvec_source_address
  J1939_ID mvec_fuse_status_id_;
  J1939_ID mvec_relay_status_id_;
  J1939_ID mvec_error_status_id_;
  J1939_ID mvec_specific_command_id_;
  J1939_ID mvec_specific_response_id_;

  std::array<bool, MvecHardware::MAX_RELAYS> relay_command_result_;
  bool high_side_output_command_result_ = false;

  // State and population data (not duplicated in status messages)
  std::array<bool, MvecHardware::MAX_RELAYS> relay_state_feedback_;
  std::array<bool, MvecHardware::MAX_RELAYS> relay_default_state_;
  bool high_side_output_feedback_ = false;
  bool high_side_output_default_state_ = false;

  // Status message objects - these contain the structured status data
  MvecFuseStatusMessage fuse_status_message_;
  MvecRelayStatusMessage relay_status_message_;
  MvecErrorStatusMessage error_status_message_;

  std::array<bool, MvecHardware::MAX_RELAYS> relay_population_state_;
  std::array<bool, MvecHardware::MAX_FUSES> fuse_population_state_;
  bool high_side_output_population_state_ = false;

  std::array<unsigned char, CAN_MAX_DLC> relay_command_data_;
  std::array<unsigned char, CAN_MAX_DLC> relay_query_data_;

};

} // polymath::sygnal

#endif // MVEC_LIB__MVEC_RELAY_HPP_
