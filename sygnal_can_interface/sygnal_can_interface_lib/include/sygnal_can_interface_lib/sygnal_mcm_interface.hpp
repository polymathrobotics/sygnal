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

#ifndef SYGNAL_CAN_INTERFACE_LIB__SYGNAL_MCM_INTERFACE_HPP_
#define SYGNAL_CAN_INTERFACE_LIB__SYGNAL_MCM_INTERFACE_HPP_

#include <array>
#include <optional>
#include <string>

#include "socketcan_adapter/can_frame.hpp"
#include "socketcan_adapter/socketcan_adapter.hpp"

namespace polymath::sygnal
{

constexpr uint8_t DEFAULT_MCM_BUS_ADDRESS = 1;
constexpr uint8_t SECONDARY_MCM_BUS_ADDRESS = 2;

/// @brief A major.minor.patch firmware version reported in an MCM Identify response.
struct SygnalVersion
{
  uint16_t major{0};
  uint16_t minor{0};
  uint16_t patch{0};
};

inline bool operator==(const SygnalVersion & a, const SygnalVersion & b)
{
  return a.major == b.major  // NOLINT(readability/check)
         && a.minor == b.minor  // NOLINT(readability/check)
         && a.patch == b.patch;
}

/// @brief One device's identity and firmware versions, assembled from its three IdentifyResponse frames
///        (Main, AppVersion, BLVersion) and keyed by (bus_id, subsystem_id). Device-type-agnostic: every
///        Sygnal device (MCM/HPO/CB/IO) answers an IdentifyCommand with the same layout. Each frame arrives
///        independently; the has_* flags mark which parts have been recorded. product_id is the
///        discriminator for classifying an entry into a typed device once a product_id -> device_type map
///        exists.
struct SygnalIdentity
{
  uint8_t bus_id{0};
  uint8_t subsystem_id{0};
  uint32_t module_serial_number{0};
  uint16_t product_id{0};
  uint8_t module_boot_state{0};
  SygnalVersion app_version{};
  SygnalVersion bl_version{};
  bool has_main{false};
  bool has_app_version{false};
  bool has_bl_version{false};
};

inline bool operator==(const SygnalIdentity & a, const SygnalIdentity & b)
{
  return a.bus_id == b.bus_id  // NOLINT(readability/check)
         && a.subsystem_id == b.subsystem_id  // NOLINT(readability/check)
         && a.module_serial_number == b.module_serial_number  // NOLINT(readability/check)
         && a.product_id == b.product_id  // NOLINT(readability/check)
         && a.module_boot_state == b.module_boot_state  // NOLINT(readability/check)
         && a.app_version == b.app_version  // NOLINT(readability/check)
         && a.bl_version == b.bl_version  // NOLINT(readability/check)
         && a.has_main == b.has_main  // NOLINT(readability/check)
         && a.has_app_version == b.has_app_version  // NOLINT(readability/check)
         && a.has_bl_version == b.has_bl_version;
}

/// @brief Decoded contents of a single IdentifyResponse frame, before endpoint filtering. Decoding is
///        device-type-agnostic because every Sygnal device answers with the same three-frame layout.
struct IdentifyResponseFrame
{
  enum class Kind : uint8_t
  {
    kMain,
    kAppVersion,
    kBlVersion
  };

  uint8_t bus_id{0};
  uint8_t subsystem_id{0};
  Kind kind{Kind::kMain};

  // Populated when kind == kMain.
  uint32_t module_serial_number{0};
  uint16_t product_id{0};
  uint8_t module_boot_state{0};

  // Populated when kind == kAppVersion or kBlVersion.
  SygnalVersion version{};
};

/// @brief Decode an IdentifyResponse CAN frame (Main / AppVersion / BLVersion) without endpoint
///        filtering. Identify frames carry no CRC. std::nullopt if the frame is not an IdentifyResponse.
std::optional<IdentifyResponseFrame> decodeIdentifyResponseFrame(const socketcan::CanFrame & frame);

/// @brief Merge one decoded IdentifyResponse frame into an identity record. Does not touch bus_id /
///        subsystem_id; the caller owns the (bus, subsystem) key.
inline void applyIdentifyFrame(SygnalIdentity & identity, const IdentifyResponseFrame & decoded)
{
  switch (decoded.kind) {
    case IdentifyResponseFrame::Kind::kMain:
      identity.module_serial_number = decoded.module_serial_number;
      identity.product_id = decoded.product_id;
      identity.module_boot_state = decoded.module_boot_state;
      identity.has_main = true;
      break;
    case IdentifyResponseFrame::Kind::kAppVersion:
      identity.app_version = decoded.version;
      identity.has_app_version = true;
      break;
    case IdentifyResponseFrame::Kind::kBlVersion:
      identity.bl_version = decoded.version;
      identity.has_bl_version = true;
      break;
  }
}

/// @brief Enum representing the overall state of the system
enum class SygnalSystemState : uint8_t
{
  FAIL_HARD = 254,
  HUMAN_OVERRIDE = 253,
  FAIL_OPERATIONAL_2 = 242,
  FAIL_OPERATIONAL_1 = 241,
  MCM_CONTROL = 1,
  HUMAN_CONTROL = 0
};

/// @brief Get the string of the Sygnal System State
/// @param state input SygnalSystemState
/// @return string of the state value
inline std::string sygnalSystemStateToString(SygnalSystemState state)
{
  switch (state) {
    case SygnalSystemState::FAIL_HARD:
      return "FAIL_HARD";
    case SygnalSystemState::HUMAN_OVERRIDE:
      return "HUMAN_OVERRIDE";
    case SygnalSystemState::FAIL_OPERATIONAL_2:
      return "FAIL_OPERATIONAL_2";
    case SygnalSystemState::FAIL_OPERATIONAL_1:
      return "FAIL_OPERATIONAL_1";
    case SygnalSystemState::MCM_CONTROL:
      return "MCM_CONTROL";
    case SygnalSystemState::HUMAN_CONTROL:
      return "HUMAN_CONTROL";
    default:
      return "UNKNOWN_STATE";
  }
}

class SygnalMcmInterface
{
public:
  SygnalMcmInterface();
  explicit SygnalMcmInterface(const uint8_t bus_address, const uint8_t subsystem_id);
  ~SygnalMcmInterface() = default;

  std::array<SygnalSystemState, 5> get_interface_states() const
  {
    return sygnal_interface_states_;
  }

  uint8_t get_bus_address() const
  {
    return bus_address_;
  }

  uint8_t get_subsystem_id() const
  {
    return subsystem_id_;
  }

  SygnalSystemState get_mcm_state() const
  {
    return sygnal_mcm_state_;
  }

  std::chrono::system_clock::time_point get_last_heartbeat_timestamp() const
  {
    return last_heartbeat_timestamp_;
  }

  /// @brief Get the identity assembled from this MCM's IdentifyResponse frames.
  /// @return std::nullopt until at least one IdentifyResponse frame has been recorded.
  std::optional<SygnalIdentity> get_identity() const
  {
    if (!identity_.has_main && !identity_.has_app_version && !identity_.has_bl_version) {
      return std::nullopt;
    }
    return identity_;
  }

  bool parseMcmHeartbeatFrame(const socketcan::CanFrame & frame);

  /// @brief Parse an MCM IdentifyResponse frame (Main / AppVersion / BLVersion)
  //         addressed to this MCM and record the result.
  //         Identify frames carry no CRC, so the filtering is done via bus + subsystem.
  /// @return true if the frame was a matching IdentifyResponse and was recorded.
  bool parseIdentifyResponseFrame(const socketcan::CanFrame & frame);

private:
  uint8_t subsystem_id_;
  uint8_t bus_address_;
  SygnalSystemState sygnal_mcm_state_;
  std::array<SygnalSystemState, 5> sygnal_interface_states_;
  std::chrono::system_clock::time_point last_heartbeat_timestamp_;
  SygnalIdentity identity_;
};

}  // namespace polymath::sygnal

#endif  // SYGNAL_CAN_INTERFACE_LIB__SYGNAL_MCM_INTERFACE_HPP_
