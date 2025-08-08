#ifndef MVEC_LIB__MVEC_RELAY_HPP_
#define MVEC_LIB__MVEC_RELAY_HPP_

#include <stdint.h>
#include <array>
#include "socketcan_adapter/can_frame.hpp"

namespace polymath::sygnal
{

static const int mvec_max_relays=12;
static const int mvec_max_fuses=24;

static const uint8_t mvec_broadcast_pdu = 0xFF;
static const uint8_t mvec_specific_pdu = 0xEF;

struct mvec_msg_ids {
  static const uint8_t relay_command_with_feedback_ = 0x88;
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

  virtual ~MvecRelay();

  // return False if message is not relevant
  bool parseMessage(const socketcan::CanFrame & can_frame);

  void set_relay_in_command(uint8_t relay_id, uint8_t relay_state);

  socketcan::CanFrame getRelayCommandMessage();

  socketcan::CanFrame getRelayQueryMessage();

  bool get_relay_state(const uint8_t relay_id);

  bool get_fuse_state(const uint8_t fuse_id);

  bool is_relay_populated(const uint8_t relay_id);

  bool is_fuse_populated(const uint8_t fuse_id);

  // TODO: Add set_population and get_population_command
  // TODO: Add set_default and get_default_command

private:
  bool parseSpecificResponse(const socketcan::CanFrame & frame);
  bool parseRelayReply(const socketcan::CanFrame & frame);
  bool parsePopulationReply(const socketcan::CanFrame & frame);
  bool parseRelayCommmandReply(const socketcan::CanFrame & frame);
  bool parseFuseStatus(const socketcan::CanFrame & frame);
  bool parseRelayStatus(const socketcan::CanFrame & frame);
  bool parseErrorStatus(const socketcan::CanFrame & frame);

  // TODO: Make these queryable and configurable and update lookup
  // For now these are defaults
  const uint8_t mvec_source_address_ = 0xB0;
  const uint8_t pgn_base_value_ = 0xA0;
  const uint8_t my_address_ = 0x00;

  socketcan::CanFrame relay_command_message_;
  socketcan::CanFrame relay_query_messasge_;

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

  // Fun note, bool and uint8_t take up the same memory, but this is semantically more clear
  std::array<bool, mvec_max_relays> relay_state_feedback_;
  std::array<bool, mvec_max_fuses> fuse_state_feedback_;
  std::array<bool, mvec_max_relays> relay_default_state_;

  std::array<bool, mvec_max_relays> relay_population_state_;
  std::array<bool, mvec_max_fuses> fuse_population_state_;

  std::array<unsigned char, CAN_MAX_DLC> relay_command_data_;
  std::array<unsigned char, CAN_MAX_DLC> relay_query_data_;

};

} // polymath::sygnal

#endif // MVEC_LIB__MVEC_RELAY_HPP_
