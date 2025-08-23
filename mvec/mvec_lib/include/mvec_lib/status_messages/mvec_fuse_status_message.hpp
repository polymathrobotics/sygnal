// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#ifndef MVEC_LIB__MVEC_FUSE_STATUS_MESSAGE_HPP_
#define MVEC_LIB__MVEC_FUSE_STATUS_MESSAGE_HPP_

#include <stdint.h>

#include <array>

#include "mvec_lib/core/can_bitwork.hpp"
#include "mvec_lib/core/j1939_id.hpp"
#include "mvec_lib/core/mvec_constants.hpp"
#include "socketcan_adapter/can_frame.hpp"

namespace polymath::sygnal
{

namespace MvecFuseStatusConstants
{
inline constexpr uint8_t START_BYTE = 1;
inline constexpr uint8_t BITS_PER_FUSE = 2;
}  // namespace MvecFuseStatusConstants

enum class MvecFuseStatus : uint8_t
{
  NO_FAULT = 0x0,
  BLOWN = 0x1,
  NOT_POWERED = 0x2,
  NOT_USED = 0x3
};

class MvecFuseStatusMessage
{
public:
  MvecFuseStatusMessage(
    uint8_t source_address = MvecProtocol::DEFAULT_SOURCE_ADDRESS,
    uint8_t pgn_base_value = MvecProtocol::DEFAULT_PGN_BASE_VALUE);

  bool parse(const socketcan::CanFrame & frame);

  MvecFuseStatus get_fuse_status(uint8_t fuse_id) const;

  bool is_valid() const
  {
    return is_valid_;
  }

private:
  J1939_ID expected_id_;
  std::array<MvecFuseStatus, MvecHardware::MAX_NUMBER_FUSES> fuse_statuses_;
  bool is_valid_;
};

}  // namespace polymath::sygnal

#endif  // MVEC_LIB__MVEC_FUSE_STATUS_MESSAGE_HPP_
