// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#ifndef MVEC_LIB__MVEC_POPULATION_REPLY_HPP_
#define MVEC_LIB__MVEC_POPULATION_REPLY_HPP_

#include <stdint.h>

#include <array>

#include "mvec_lib/can_bitwork.hpp"
#include "socketcan_adapter/can_frame.hpp"

namespace polymath::sygnal
{

namespace MvecPopulationConstants
{
inline constexpr uint8_t POPULATION_FUSE_START_BYTE = 2;
inline constexpr uint8_t POPULATION_RELAY_START_BYTE = 5;
inline constexpr uint8_t POPULATION_RESPONSE_MESSAGE_ID = 0x94;
}  // namespace MvecPopulationConstants

class MvecPopulationReply
{
public:
  MvecPopulationReply();

  bool parse(const socketcan::CanFrame & frame);

  bool get_fuse_population(uint8_t fuse_id) const;
  bool get_relay_population(uint8_t relay_id) const;

  bool get_high_side_output_population() const
  {
    return high_side_output_population_;
  }

  bool is_valid() const
  {
    return is_valid_;
  }

private:
  std::array<bool, 24> fuse_population_;
  std::array<bool, 12> relay_population_;
  bool high_side_output_population_;
  bool is_valid_;
};

}  // namespace polymath::sygnal

#endif  // MVEC_LIB__MVEC_POPULATION_REPLY_HPP_
