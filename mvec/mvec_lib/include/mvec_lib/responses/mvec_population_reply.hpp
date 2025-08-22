// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

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
inline constexpr uint8_t POPULATION_RELAY_START_BYTE = 5;
inline constexpr uint8_t POPULATION_RESPONSE_MESSAGE_ID = 0x94;
}  // namespace MvecPopulationConstants

class MvecPopulationReply : public MvecResponseBase
{
public:
  MvecPopulationReply(
    uint8_t source_address,
    uint8_t my_address);

  bool get_fuse_population(uint8_t fuse_id) const;
  bool get_relay_population(uint8_t relay_id) const;

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
