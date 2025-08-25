// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#ifndef MVEC_LIB__MVEC_CONSTANTS_HPP_
#define MVEC_LIB__MVEC_CONSTANTS_HPP_

#include <stdint.h>

namespace polymath::sygnal
{

/// @brief Hardware limits for MVEC devices
namespace MvecHardware
{
/// @brief Maximum number of relays supported by MVEC device
inline constexpr int MAX_NUMBER_RELAYS = 12;
/// @brief Maximum number of fuses supported by MVEC device
inline constexpr int MAX_NUMBER_FUSES = 24;
/// @brief Maximum number of high-side outputs supported
inline constexpr int MAX_HIGH_SIDE_OUTPUTS = 1;
};  // namespace MvecHardware

/// @brief J1939 protocol constants for MVEC communication
namespace MvecProtocol
{
/// @brief PDU format for broadcast status messages
inline constexpr uint8_t STATUS_PDU = 0xFF;
/// @brief PDU format for specific query/response messages
inline constexpr uint8_t QUERY_PDU = 0xEF;
/// @brief Default J1939 message priority
inline constexpr uint8_t DEFAULT_PRIORITY = 6;
/// @brief Default J1939 data page
inline constexpr uint8_t DEFAULT_DATA_PAGE = 0;
/// @brief Reserved bit value for J1939
inline constexpr uint8_t RESERVED_BIT = 0;
/// @brief Default source address for MVEC devices
inline constexpr uint8_t DEFAULT_SOURCE_ADDRESS = 0xB0;
/// @brief Base value for PGN calculation
inline constexpr uint8_t DEFAULT_PGN_BASE_VALUE = 0xA0;
/// @brief Default self address for controller
inline constexpr uint8_t DEFAULT_SELF_ADDRESS = 0x00;
/// @brief Default grid address
inline constexpr uint8_t DEFAULT_GRID_ADDRESS = 0x00;
};  // namespace MvecProtocol

namespace MvecMessageIds
{
inline constexpr uint8_t RELAY_COMMAND_WITH_FEEDBACK = 0x88;
inline constexpr uint8_t RELAY_COMMAND_NO_FEEDBACK = 0x80;
inline constexpr uint8_t RELAY_STATE_QUERY = 0x96;
inline constexpr uint8_t POPULATION_QUERY = 0x92;
};  // namespace MvecMessageIds

namespace MvecMessageStructure
{
inline constexpr uint8_t MSG_ID_BYTE = 0;
inline constexpr uint8_t GRID_ID_BYTE = 1;
inline constexpr uint8_t RELAY_DATA_START_BIT = 16;
inline constexpr uint8_t BITS_PER_RELAY = 2;
inline constexpr uint8_t HIGH_SIDE_BITS = 2;
};  // namespace MvecMessageStructure

namespace MvecValueLimits
{
inline constexpr uint8_t MAX_RELAY_STATE_VALUE = 0x01;
inline constexpr uint8_t MAX_HIGH_SIDE_STATE_VALUE = 0x01;
};  // namespace MvecValueLimits

/// @brief Message types that can be parsed from MVEC devices
enum class MvecMessageType : int16_t
{
  /// Message not recognized as MVEC format
  UNSUPPORTED = -1,
  /// Fuse status broadcast message
  FUSE_STATUS,
  /// Error status broadcast message
  ERROR_STATUS,
  /// Relay status broadcast message
  RELAY_STATUS,            
  /// Response to relay command
  RELAY_COMMAND_RESPONSE,  
  /// Response to relay query
  RELAY_QUERY_RESPONSE,    
  /// Response to population query
  POPULATION_RESPONSE,     
};

}  // namespace polymath::sygnal

#endif  // MVEC_LIB__MVEC_CONSTANTS_HPP_
