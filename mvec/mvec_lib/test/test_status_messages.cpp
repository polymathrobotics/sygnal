// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#include <vector>

#include <catch2/catch.hpp>

#include "mvec_lib/mvec_constants.hpp"
#include "mvec_lib/mvec_status_messages.hpp"
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
  std::vector<uint8_t> data = {0x00, 0x55, 0xAA, 0x33, 0xCC, 0x0F, 0xF0, 0x99};

  auto frame = createTestFrame(0x18A01B0, data);  // Example CAN ID
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
  std::vector<uint8_t> data = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77};

  auto frame = createTestFrame(0x18A02B0, data);  // Example CAN ID
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

  auto frame = createTestFrame(0x18A03B0, data);  // Example CAN ID
  REQUIRE(msg.parse(frame));
  REQUIRE(msg.is_valid());
  REQUIRE(msg.has_error(polymath::sygnal::MvecErrorType::INVALID_CONFIG));
}
