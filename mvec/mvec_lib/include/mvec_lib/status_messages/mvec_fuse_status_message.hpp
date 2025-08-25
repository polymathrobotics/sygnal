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

/// @brief Constants for parsing fuse status messages
namespace MvecFuseStatusConstants
{
/// @brief Starting byte position of fuse data in CAN frame
inline constexpr uint8_t START_BYTE = 1;
/// @brief Number of bits used to encode each fuse status
inline constexpr uint8_t BITS_PER_FUSE = 2;
}  // namespace MvecFuseStatusConstants

/// @brief Possible status values for MVEC fuses
enum class MvecFuseStatus : uint8_t
{
  NO_FAULT = 0x0,    ///< Fuse is functioning normally
  BLOWN = 0x1,       ///< Fuse has blown (overcurrent protection activated)
  NOT_POWERED = 0x2,  ///< Fuse is not receiving power
  NOT_USED = 0x3      ///< Fuse location is not populated or used
};

/// @brief Parser and container for MVEC fuse status broadcast messages
/// Provides status information for all fuses in the MVEC device
class MvecFuseStatusMessage
{
public:
  /// @brief Constructor
  /// @param source_address Expected source address for messages
  /// @param pgn_base_value Base PGN value for message identification
  MvecFuseStatusMessage(
    uint8_t source_address = MvecProtocol::DEFAULT_SOURCE_ADDRESS,
    uint8_t pgn_base_value = MvecProtocol::DEFAULT_PGN_BASE_VALUE);

  /// @brief Parse CAN frame for fuse status data
  /// @param frame CAN frame to parse
  /// @return true if frame was successfully parsed
  bool parse(const socketcan::CanFrame & frame);

  /// @brief Get status of specific fuse
  /// @param fuse_id Fuse ID (0-23)
  /// @return Current status of the specified fuse
  MvecFuseStatus get_fuse_status(uint8_t fuse_id) const;

  /// @brief Check if parsed data is valid
  /// @return true if message has been successfully parsed
  bool is_valid() const
  {
    return is_valid_;
  }

private:
  /// @brief Expected J1939 message ID for filtering
  J1939_ID expected_id_;
  /// @brief Status array for all fuses (0-23)
  std::array<MvecFuseStatus, MvecHardware::MAX_NUMBER_FUSES> fuse_statuses_;
  /// @brief Validity flag for parsed data
  bool is_valid_;
};

}  // namespace polymath::sygnal

#endif  // MVEC_LIB__MVEC_FUSE_STATUS_MESSAGE_HPP_
