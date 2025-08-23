// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#ifndef MVEC_LIB__MVEC_ERROR_STATUS_MESSAGE_HPP_
#define MVEC_LIB__MVEC_ERROR_STATUS_MESSAGE_HPP_

#include <stdint.h>

#include "mvec_lib/core/can_bitwork.hpp"
#include "mvec_lib/core/j1939_id.hpp"
#include "mvec_lib/core/mvec_constants.hpp"
#include "socketcan_adapter/can_frame.hpp"

namespace polymath::sygnal
{

namespace MvecErrorStatusConstants
{
inline constexpr uint8_t NUM_ERROR_BITS = 13;
inline constexpr uint8_t START_BYTE = 1;
}  // namespace MvecErrorStatusConstants

namespace MvecErrorBits
{
inline constexpr uint16_t INVALID_CONFIG = 0x0001;
inline constexpr uint16_t GRID_ID_CHANGED = 0x0002;
inline constexpr uint16_t CAN_ADDRESS_CHANGED = 0x0004;
inline constexpr uint16_t CAN_RX_COMM_ERROR = 0x0008;
inline constexpr uint16_t CAN_TX_COMM_ERROR = 0x0010;
inline constexpr uint16_t UNEXPECTED_RESET = 0x0020;
inline constexpr uint16_t OVER_VOLTAGE = 0x0040;
inline constexpr uint16_t SPI_ERROR = 0x0080;
inline constexpr uint16_t SHORT_MESSAGE_RECEIVED = 0x0100;
inline constexpr uint16_t BAD_FLASH_ADDRESS = 0x0200;
inline constexpr uint16_t INVALID_LENGTH = 0x0400;
inline constexpr uint16_t CHECKSUM_FAILURE = 0x0800;
inline constexpr uint16_t FLASH_MISCOMPARE = 0x1000;
}  // namespace MvecErrorBits

enum class MvecErrorType : uint16_t
{
  INVALID_CONFIG = MvecErrorBits::INVALID_CONFIG,
  GRID_ID_CHANGED = MvecErrorBits::GRID_ID_CHANGED,
  CAN_ADDRESS_CHANGED = MvecErrorBits::CAN_ADDRESS_CHANGED,
  CAN_RX_COMM_ERROR = MvecErrorBits::CAN_RX_COMM_ERROR,
  CAN_TX_COMM_ERROR = MvecErrorBits::CAN_TX_COMM_ERROR,
  UNEXPECTED_RESET = MvecErrorBits::UNEXPECTED_RESET,
  OVER_VOLTAGE = MvecErrorBits::OVER_VOLTAGE,
  SPI_ERROR = MvecErrorBits::SPI_ERROR,
  SHORT_MESSAGE_RECEIVED = MvecErrorBits::SHORT_MESSAGE_RECEIVED,
  BAD_FLASH_ADDRESS = MvecErrorBits::BAD_FLASH_ADDRESS,
  INVALID_LENGTH = MvecErrorBits::INVALID_LENGTH,
  CHECKSUM_FAILURE = MvecErrorBits::CHECKSUM_FAILURE,
  FLASH_MISCOMPARE = MvecErrorBits::FLASH_MISCOMPARE
};

class MvecErrorStatusMessage
{
public:
  MvecErrorStatusMessage(
    uint8_t source_address = MvecProtocol::DEFAULT_SOURCE_ADDRESS,
    uint8_t pgn_base_value = MvecProtocol::DEFAULT_PGN_BASE_VALUE);

  bool parse(const socketcan::CanFrame & frame);

  uint16_t get_error_bits() const
  {
    return error_bits_;
  }

  uint8_t get_grid_address() const
  {
    return grid_address_;
  }

  bool has_error(MvecErrorType error_type) const;

  bool is_valid() const
  {
    return is_valid_;
  }

private:
  J1939_ID expected_id_;
  uint8_t grid_address_;
  uint16_t error_bits_;
  bool is_valid_;
};

}  // namespace polymath::sygnal

#endif  // MVEC_LIB__MVEC_ERROR_STATUS_MESSAGE_HPP_
