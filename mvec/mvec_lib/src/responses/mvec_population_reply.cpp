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

#include "mvec_lib/responses/mvec_population_reply.hpp"

#include "mvec_lib/core/mvec_constants.hpp"

namespace polymath::sygnal
{

MvecPopulationReply::MvecPopulationReply(uint8_t source_address, uint8_t my_address)
: MvecResponseBase(MvecPopulationConstants::POPULATION_RESPONSE_MESSAGE_ID, source_address, my_address)
, high_side_output_population_(false)
{
  fuse_population_.fill(false);
  relay_population_.fill(false);
}

bool MvecPopulationReply::parse_message_data(const std::array<unsigned char, CAN_MAX_DLC> & data)
{
  // Parse fuse population from bytes 2-4
  auto fuse_data = unpackData<uint32_t>(
    data, MvecPopulationConstants::POPULATION_FUSE_START_BYTE * CHAR_BIT, MvecHardware::MAX_NUMBER_FUSES);

  for (size_t i = 0; i < MvecHardware::MAX_NUMBER_FUSES; ++i) {
    fuse_population_[i] = ((fuse_data >> i) & 0x01);
  }

  // Parse relay population from bytes 5&6
  auto relay_data = unpackData<uint32_t>(
    data,
    MvecPopulationConstants::POPULATION_RELAY_START_BYTE * CHAR_BIT,
    MvecHardware::MAX_NUMBER_RELAYS + MvecHardware::MAX_HIGH_SIDE_OUTPUTS);

  for (size_t i = 0; i < MvecHardware::MAX_NUMBER_RELAYS; ++i) {
    relay_population_[i] = ((relay_data >> i) & 0x01);
  }
  high_side_output_population_ = ((relay_data >> MvecHardware::MAX_NUMBER_RELAYS) & 0x01);

  return true;
}

bool MvecPopulationReply::get_fuse_population(uint8_t fuse_id) const
{
  if (fuse_id >= MvecHardware::MAX_NUMBER_FUSES) {
    return false;
  }
  return fuse_population_[fuse_id];
}

bool MvecPopulationReply::get_relay_population(uint8_t relay_id) const
{
  if (relay_id >= MvecHardware::MAX_NUMBER_RELAYS) {
    return false;
  }
  return relay_population_[relay_id];
}

}  // namespace polymath::sygnal
