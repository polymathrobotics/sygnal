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

#ifndef MVEC_LIB__MVEC_POPULATION_REPLY_HPP_
#define MVEC_LIB__MVEC_POPULATION_REPLY_HPP_

#include <linux/can.h>
#include <stdint.h>

#include <array>

#include "mvec_lib/core/can_bitwork.hpp"
#include "mvec_lib/responses/mvec_response_base.hpp"

namespace polymath::sygnal
{

namespace MvecPopulationConstants
{
inline constexpr uint8_t POPULATION_FUSE_START_BYTE = 2;
inline constexpr uint8_t POPULATION_RELAY_START_BYTE = 6;
inline constexpr uint8_t POPULATION_RESPONSE_MESSAGE_ID = 0x94;
}  // namespace MvecPopulationConstants

/// @brief Parser and container for MVEC population query responses
/// Contains information about which relays and fuses are physically installed
class MvecPopulationReply : public MvecResponseBase
{
public:
  /// @brief Constructor
  /// @param source_address Expected source address for messages
  /// @param my_address Self address for message filtering
  MvecPopulationReply(uint8_t source_address, uint8_t my_address);

  /// @brief Check if specific fuse is populated (installed)
  /// @param fuse_id Fuse ID (0-23)
  /// @return true if fuse is installed
  bool get_fuse_population(uint8_t fuse_id) const;

  /// @brief Check if specific relay is populated (installed)
  /// @param relay_id Relay ID (0-11)
  /// @return true if relay is installed
  bool get_relay_population(uint8_t relay_id) const;

  /// @brief Check if high side output is populated (installed)
  /// @return true if high side output is installed
  bool get_high_side_output_population() const
  {
    return high_side_output_population_;
  }

protected:
  bool parse_message_data(const std::array<unsigned char, CAN_MAX_DLC> & data) override;

private:
  std::array<bool, 24> fuse_population_;
  std::array<bool, 12> relay_population_;
  bool high_side_output_population_;
};

}  // namespace polymath::sygnal

#endif  // MVEC_LIB__MVEC_POPULATION_REPLY_HPP_
