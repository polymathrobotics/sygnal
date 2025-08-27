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

// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#include "mvec_lib/responses/mvec_response_base.hpp"

#include "mvec_lib/core/mvec_constants.hpp"

namespace polymath::sygnal
{

MvecResponseBase::MvecResponseBase(uint8_t expected_message_id, uint8_t source_address, uint8_t my_address)
: expected_id_(
    MvecProtocol::DEFAULT_PRIORITY,
    MvecProtocol::DEFAULT_DATA_PAGE,
    MvecProtocol::QUERY_PDU,
    my_address,
    source_address,
    MvecProtocol::RESERVED_BIT)
, expected_message_id_(expected_message_id)
, is_valid_(false)
{}

bool MvecResponseBase::parse(const polymath::socketcan::CanFrame & frame)
{
  // Parse J1939 ID from frame
  J1939_ID frame_id(frame.get_id());

  // Check if this frame matches our expected ID pattern
  if (!(frame_id == expected_id_)) {
    return false;
  }

  // Get frame data
  auto data = frame.get_data();
  if (data.empty()) {
    return false;
  }

  // Extract message ID from first byte
  auto message_id = data[MvecRelayCommandMessageStructure::MSG_ID_BYTE];

  // Check if message ID matches what we expect
  if (message_id != expected_message_id_) {
    return false;
  }

  // Let derived class parse the specific message content
  bool parse_success = parse_message_data(data);

  if (parse_success) {
    set_valid(true);
  }

  return parse_success;
}

}  // namespace polymath::sygnal
