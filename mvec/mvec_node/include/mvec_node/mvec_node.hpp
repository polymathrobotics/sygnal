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

#ifndef MVEC_NODE__MVEC_NODE_HPP_
#define MVEC_NODE__MVEC_NODE_HPP_

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "mvec_lib/mvec_relay_socketcan.hpp"
#include "mvec_msgs/msg/mvec_feedback.hpp"
#include "mvec_msgs/msg/preset.hpp"
#include "mvec_msgs/msg/relay.hpp"
#include "mvec_msgs/srv/set_multi_relay.hpp"
#include "mvec_msgs/srv/set_single_relay.hpp"
#include "mvec_msgs/srv/trigger_preset.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "socketcan_adapter/socketcan_adapter.hpp"

namespace polymath::mvec
{

/// @brief ROS2 lifecycle node for MVEC relay control system
/// Provides diagnostics publishing and relay control service
class MvecNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /// @brief Constructor
  /// @param options Node options
  explicit MvecNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief Destructor
  ~MvecNode();

protected:
  // Lifecycle interface
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & state) override;

private:
  /// @brief Timer callback to publish diagnostics and status messages
  void timerCallback();

  /// @brief Service callback to set single relay
  /// @param request Service request containing relay
  /// @param response Service response with success flag
  void setSingleRelayCallback(
    const std::shared_ptr<mvec_msgs::srv::SetSingleRelay::Request> request,
    std::shared_ptr<mvec_msgs::srv::SetSingleRelay::Response> response);

  /// @brief Service callback to set multiple relays
  /// @param request Service request containing relay array
  /// @param response Service response with success flag
  void setMultiRelayCallback(
    const std::shared_ptr<mvec_msgs::srv::SetMultiRelay::Request> request,
    std::shared_ptr<mvec_msgs::srv::SetMultiRelay::Response> response);

  /// @brief Service callback to trigger preset
  /// @param request Service request containing preset name
  /// @param response Service response with success flag
  void triggerPresetCallback(
    const std::shared_ptr<mvec_msgs::srv::TriggerPreset::Request> request,
    std::shared_ptr<mvec_msgs::srv::TriggerPreset::Response> response);

  void addDefaultPresetIfNotPresent(const std::vector<mvec_msgs::msg::Relay> & default_relays);

  /// @brief Set a single relay, send it and wait for TIMEOUT amount of time
  /// @param relay
  /// @return error message if failed, nullopt if successful
  std::optional<std::string> set_single_relay(mvec_msgs::msg::Relay relay);

  /// @brief Set multiple relays via a vector, send it and wait for a TIMEOUT amount of time
  /// @param relays
  /// @return error message if failed, nullopt if successful
  std::optional<std::string> set_multi_relay(const std::vector<mvec_msgs::msg::Relay> & relays);

  /// @brief Create diagnostics array from MVEC status
  /// @return Diagnostic array message
  diagnostic_msgs::msg::DiagnosticArray createDiagnosticsMessage();

  /// @brief Parse preset parameters from ROS parameters
  /// Format: preset_N_name and preset_N_relays array with "relay_id:state" strings
  void parsePresetParams();

  // Parameters
  std::string can_interface_;
  double publish_rate_;

  std::chrono::milliseconds timeout_ms_;

  // SocketCAN and MVEC components
  std::shared_ptr<polymath::socketcan::SocketcanAdapter> socketcan_adapter_;
  std::unique_ptr<polymath::sygnal::MvecRelaySocketcan> mvec_socketcan_;

  // ROS2 components
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<mvec_msgs::srv::SetSingleRelay>::SharedPtr set_single_relay_service_;
  rclcpp::Service<mvec_msgs::srv::SetMultiRelay>::SharedPtr set_multi_relay_service_;
  rclcpp::Service<mvec_msgs::srv::TriggerPreset>::SharedPtr trigger_preset_service_;

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp_lifecycle::LifecyclePublisher<mvec_msgs::msg::MvecFeedback>::SharedPtr feedback_pub_;

  std::vector<mvec_msgs::msg::Preset> presets_;

  // Current relay states storage
  std::optional<mvec_msgs::msg::MvecFeedback> current_relay_states_;
};

}  // namespace polymath::mvec

RCLCPP_COMPONENTS_REGISTER_NODE(polymath::mvec::MvecNode)

#endif  // MVEC_NODE__MVEC_NODE_HPP_
