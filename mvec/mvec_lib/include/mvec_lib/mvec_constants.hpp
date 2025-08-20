// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#ifndef MVEC_LIB__MVEC_CONSTANTS_HPP_
#define MVEC_LIB__MVEC_CONSTANTS_HPP_

#include <stdint.h>

namespace polymath::sygnal
{

namespace MvecHardware
{
inline constexpr int MAX_HIGH_SIDE_OUTPUTS = 1;
};

namespace MvecProtocol
{
inline constexpr uint8_t BROADCAST_PDU = 0xFF;
inline constexpr uint8_t SPECIFIC_PDU = 0xEF;
inline constexpr uint8_t DEFAULT_PRIORITY = 6;
inline constexpr uint8_t DEFAULT_DATA_PAGE = 0;
inline constexpr uint8_t RESERVED_BIT = 0;
inline constexpr uint8_t DEFAULT_SOURCE_ADDRESS = 0xB0;
inline constexpr uint8_t DEFAULT_PGN_BASE_VALUE = 0xA0;
inline constexpr uint8_t DEFAULT_SELF_ADDRESS = 0x00;
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

}  // namespace polymath::sygnal

#endif  // MVEC_LIB__MVEC_CONSTANTS_HPP_
