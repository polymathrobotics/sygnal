// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MVEC_LIB__MVEC_RELAY_COMMAND_REPLY_HPP_
#define MVEC_LIB__MVEC_RELAY_COMMAND_REPLY_HPP_

#include <linux/can.h>
#include <stdint.h>

#include <array>

#include "mvec_lib/core/can_bitwork.hpp"
#include "mvec_lib/responses/mvec_response_base.hpp"

namespace polymath::sygnal
{

/// @brief Constants for parsing relay command reply messages
namespace MvecRelayCommandConstants
{
/// @brief Byte position of success status in CAN frame
inline constexpr uint8_t COMMAND_REPLY_SUCCESS_BYTE = 2;
/// @brief Byte position of error status in CAN frame
inline constexpr uint8_t COMMAND_REPLY_ERROR_BYTE = 3;
/// @brief Starting byte position of relay result data
inline constexpr uint8_t COMMAND_REPLY_DATA_START_BYTE = 4;
/// @brief Message ID for relay command responses
inline constexpr uint8_t RESPONSE_MESSAGE_ID = 0x01;
}  // namespace MvecRelayCommandConstants

/// @brief Parser and container for MVEC relay command responses
/// Contains acknowledgment data and results of relay command execution
class MvecRelayCommandReply : public MvecResponseBase
{
public:
  /// @brief Constructor
  /// @param source_address Expected source address for messages
  /// @param my_address Self address for message filtering
  MvecRelayCommandReply(uint8_t source_address, uint8_t my_address);

public:
  /// @brief Get original command message ID
  /// @return Message ID of the command that was executed
  uint8_t get_command_msg_id() const
  {
    return command_msg_id_;
  }

  /// @brief Get command execution success status
  /// @return Success code from command execution
  uint8_t get_success() const
  {
    return success_;
  }

  /// @brief Get command execution error status
  /// @return Error code from command execution (0 = no error)
  uint8_t get_error() const
  {
    return error_;
  }

  /// @brief Get execution result for specific relay
  /// @param relay_id Relay ID (0-11)
  /// @return true if relay command was successfully executed
  bool get_relay_result(uint8_t relay_id) const;

  /// @brief Get execution result for high side output
  /// @return true if high side output command was successfully executed
  bool get_high_side_output_result() const
  {
    return high_side_output_result_;
  }

protected:
  /// @brief Parse CAN frame data for command reply information
  /// @param data CAN frame data array
  /// @return true if parsing was successful
  bool parse_message_data(const std::array<unsigned char, CAN_MAX_DLC> & data) override;

private:
  /// @brief Original command message ID
  uint8_t command_msg_id_;
  /// @brief Command execution success status
  uint8_t success_;
  /// @brief Command execution error status
  uint8_t error_;
  /// @brief Execution results for each relay (0-11)
  std::array<bool, 12> relay_results_;
  /// @brief Execution result for high side output
  bool high_side_output_result_;
};

}  // namespace polymath::sygnal

#endif  // MVEC_LIB__MVEC_RELAY_COMMAND_REPLY_HPP_
