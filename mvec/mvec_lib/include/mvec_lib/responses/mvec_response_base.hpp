// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#ifndef MVEC_LIB__RESPONSES__MVEC_RESPONSE_BASE_HPP_
#define MVEC_LIB__RESPONSES__MVEC_RESPONSE_BASE_HPP_

#include <linux/can.h>
#include <stdint.h>

#include <array>

#include "mvec_lib/core/j1939_id.hpp"
#include "socketcan_adapter/can_frame.hpp"

namespace polymath::sygnal
{

/// @brief Base class for all MVEC response parsers
/// @details Handles common J1939 parsing and validation logic
class MvecResponseBase
{
public:
  /// @brief Constructor
  /// @param expected_message_id The expected message ID for this response type
  /// @param source_address The source address of the MVEC controller
  /// @param self_address My address for filtering responses
  explicit MvecResponseBase(uint8_t expected_message_id, uint8_t source_address, uint8_t self_address);

  /// @brief Virtual destructor
  virtual ~MvecResponseBase() = default;

  /// @brief Parse a CAN frame and determine if it matches this response type
  /// @param frame The CAN frame to parse
  /// @return True if the frame was successfully parsed, false otherwise
  bool parse(const polymath::socketcan::CanFrame & frame);

  /// @brief Check if the last parsed message is valid
  /// @return True if valid, false otherwise
  bool is_valid() const
  {
    return is_valid_;
  }

  /// @brief Get the message ID from the last parsed frame
  /// @return The message ID
  uint8_t get_message_id() const
  {
    return expected_message_id_;
  }

protected:
  /// @brief Pure virtual function to be implemented by derived classes
  /// @details Called after J1939 validation passes to parse specific message content
  /// @param data CAN frame data array
  /// @return True if parsing was successful, false otherwise
  virtual bool parse_message_data(const std::array<unsigned char, CAN_MAX_DLC> & data) = 0;

  /// @brief Reset the validity state (called before each parse attempt)
  void reset_validity()
  {
    is_valid_ = false;
  }

  /// @brief Set the validity state (called by derived classes after successful parsing)
  void set_valid(bool valid)
  {
    is_valid_ = valid;
  }

private:
  J1939_ID expected_id_;  ///< Expected J1939 ID for this response type
  uint8_t expected_message_id_;  ///< Expected message ID
  bool is_valid_;  ///< Validity state of last parsed message
};

}  // namespace polymath::sygnal

#endif  // MVEC_LIB__RESPONSES__MVEC_RESPONSE_BASE_HPP_
