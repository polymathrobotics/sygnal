#ifndef MVEC_LIB__MVEC_RELAY_HPP_
#define MVEC_LIB__MVEC_RELAY_HPP_

#include <stdint.h>
#include <array>
#include <chrono>
#include "socketcan_adapter/can_frame.hpp"
#include "mvec_lib/j1939_id.hpp"

namespace polymath::sygnal
{

static const int mvec_max_relays=12;
static const int mvec_max_high_side_outputs=1;
static const int mvec_max_fuses=24;

static const uint8_t num_error_bits=13;

static const uint8_t mvec_broadcast_pdu = 0xFF;
static const uint8_t mvec_specific_pdu = 0xEF;

struct mvec_msg_ids {
  static const uint8_t relay_command_with_feedback = 0x88;
  static const uint8_t relay_commmand_no_feedback = 0x80;
  static const uint8_t relay_state_query = 0x96;
  static const uint8_t population_query = 0x92;
  static const uint8_t response = 0x01;
  static const uint8_t population_response = 0x94;

  // Not an enum because query response and command have the same id
  static const uint8_t relay_query_response = 0x96;
};

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

  bool get_relay_state(const uint8_t relay_id);

  uint8_t get_fuse_state(const uint8_t fuse_id);

  bool is_relay_populated(const uint8_t relay_id);

  bool is_fuse_populated(const uint8_t fuse_id);

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
  const uint8_t mvec_source_address_ = 0xB0;
  const uint8_t pgn_base_value_ = 0xA0;
  const uint8_t my_address_ = 0x00;

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

  std::array<bool, mvec_max_relays> relay_command_result_;
  bool high_side_output_command_result_ = false;

  std::array<bool, mvec_max_relays> relay_state_feedback_;
  std::array<uint8_t, mvec_max_relays> relay_status_feedback_;
  std::array<uint8_t, mvec_max_fuses> fuse_state_feedback_;
  std::array<bool, mvec_max_relays> relay_default_state_;
  bool high_side_output_feedback_ = false;
  bool high_side_output_default_state_ = false;
  uint64_t error_bits_;

  std::array<bool, mvec_max_relays> relay_population_state_;
  std::array<bool, mvec_max_fuses> fuse_population_state_;
  bool high_side_output_population_state_ = false;

  std::array<unsigned char, CAN_MAX_DLC> relay_command_data_;
  std::array<unsigned char, CAN_MAX_DLC> relay_query_data_;

};

} // polymath::sygnal

#endif // MVEC_LIB__MVEC_RELAY_HPP_
