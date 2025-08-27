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

#include <vector>

#include <catch2/catch.hpp>

#include "mvec_lib/core/mvec_constants.hpp"
#include "mvec_lib/status_messages/mvec_error_status_message.hpp"
#include "mvec_lib/status_messages/mvec_fuse_status_message.hpp"
#include "mvec_lib/status_messages/mvec_relay_status_message.hpp"
#include "socketcan_adapter/can_frame.hpp"

static polymath::socketcan::CanFrame createTestFrame(uint32_t can_id, const std::vector<uint8_t> & data)
{
  polymath::socketcan::CanFrame frame;
  frame.set_can_id(can_id);
  frame.set_id_as_extended();
  std::array<unsigned char, CAN_MAX_DLC> frame_data;
  frame_data.fill(0);
  for (size_t i = 0; i < data.size() && i < CAN_MAX_DLC; ++i) {
    frame_data[i] = data[i];
  }
  frame.set_data(frame_data);
  return frame;
}

TEST_CASE("FuseStatusMessage initialization", "[status_messages]")
{
  polymath::sygnal::MvecFuseStatusMessage msg;
  REQUIRE_FALSE(msg.is_valid());
  REQUIRE(msg.get_fuse_status(0) == polymath::sygnal::MvecFuseStatus::NOT_USED);
}

TEST_CASE("FuseStatusMessage parsing", "[status_messages]")
{
  polymath::sygnal::MvecFuseStatusMessage msg;
  J1939_ID fuse_status_id(6, 0, polymath::sygnal::MvecProtocol::STATUS_PDU, 0x00 + 0xA0, 0xB0);
  std::vector<uint8_t> data = {0x00, 0x55, 0xAA, 0x33, 0xCC, 0x0F, 0xF0, 0x99};

  auto frame = createTestFrame(fuse_status_id.get_can_id(), data);
  REQUIRE(msg.parse(frame));
  REQUIRE(msg.is_valid());
}

TEST_CASE("RelayStatusMessage initialization", "[status_messages]")
{
  polymath::sygnal::MvecRelayStatusMessage msg;
  REQUIRE_FALSE(msg.is_valid());
  REQUIRE(msg.get_relay_status(0) == polymath::sygnal::MvecRelayStatus::RELAY_LOCATION_NOT_USED);
}

TEST_CASE("RelayStatusMessage parsing", "[status_messages]")
{
  polymath::sygnal::MvecRelayStatusMessage msg;
  J1939_ID relay_status_id(6, 0, polymath::sygnal::MvecProtocol::STATUS_PDU, 0x01 + 0xA0, 0xB0);
  std::vector<uint8_t> data = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77};

  auto frame = createTestFrame(relay_status_id.get_can_id(), data);
  REQUIRE(msg.parse(frame));
  REQUIRE(msg.is_valid());
}

TEST_CASE("ErrorStatusMessage initialization", "[status_messages]")
{
  polymath::sygnal::MvecErrorStatusMessage msg;
  REQUIRE_FALSE(msg.is_valid());
  REQUIRE(msg.get_error_bits() == 0);
  REQUIRE_FALSE(msg.has_error(static_cast<polymath::sygnal::MvecErrorType>(0)));
}

TEST_CASE("ErrorStatusMessage parsing", "[status_messages]")
{
  polymath::sygnal::MvecErrorStatusMessage msg;
  std::vector<uint8_t> data = {0x00, 0xFF, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00};

  J1939_ID error_status_id(6, 0, polymath::sygnal::MvecProtocol::STATUS_PDU, 0x02 + 0xA0, 0xB0);
  auto frame = createTestFrame(error_status_id.get_can_id(), data);
  REQUIRE(msg.parse(frame));
  REQUIRE(msg.is_valid());
  REQUIRE(msg.has_error(polymath::sygnal::MvecErrorType::INVALID_CONFIG));
}
