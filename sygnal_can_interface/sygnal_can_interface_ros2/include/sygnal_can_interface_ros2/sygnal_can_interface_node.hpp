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

#ifndef SYGNAL_CAN_INTERFACE_NODE__SYGNAL_CAN_INTERFACE_NODE_HPP_
#define SYGNAL_CAN_INTERFACE_NODE__SYGNAL_CAN_INTERFACE_NODE_HPP_

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "socketcan_adapter/socketcan_adapter.hpp"
#include "sygnal_can_interface_lib/sygnal_interface_socketcan.hpp"
#include "sygnal_can_msgs/msg/mcm_heartbeat.hpp"
#include "sygnal_can_msgs/srv/send_control_command.hpp"
#include "sygnal_can_msgs/srv/send_relay_command.hpp"
#include "sygnal_can_msgs/srv/set_control_state.hpp"

namespace polymath::sygnal
{

/// @brief ROS2 lifecycle node for Sygnal CAN interface control system
/// Provides MCM heartbeat publishing and command services
class SygnalCanInterfaceNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /// @brief Constructor
  /// @param options Node options
  explicit SygnalCanInterfaceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief Destructor
  ~SygnalCanInterfaceNode();

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
  /// @brief Timer callback to publish MCM heartbeat and diagnostics
  void timerCallback();

  /// @brief Service callback to set control state
  /// @param request Service request containing control state parameters
  /// @param response Service response with success flag
  void setControlStateCallback(
    const std::shared_ptr<sygnal_can_msgs::srv::SetControlState::Request> request,
    std::shared_ptr<sygnal_can_msgs::srv::SetControlState::Response> response);

  /// @brief Service callback to send control command
  /// @param request Service request containing control command parameters
  /// @param response Service response with success flag
  void sendControlCommandCallback(
    const std::shared_ptr<sygnal_can_msgs::srv::SendControlCommand::Request> request,
    std::shared_ptr<sygnal_can_msgs::srv::SendControlCommand::Response> response);

  /// @brief Service callback to send relay command
  /// @param request Service request containing relay command parameters
  /// @param response Service response with success flag
  void sendRelayCommandCallback(
    const std::shared_ptr<sygnal_can_msgs::srv::SendRelayCommand::Request> request,
    std::shared_ptr<sygnal_can_msgs::srv::SendRelayCommand::Response> response);

  /// @brief Subscription callback to handle incoming control commands
  /// @param msg Incoming control command message
  void controlCommandCallback(const sygnal_can_msgs::msg::ControlCommand::UniquePtr msg);

  /// @brief Create diagnostics array from Sygnal status
  /// @return Diagnostic array message
  diagnostic_msgs::msg::DiagnosticArray createDiagnosticsMessage();

  // Parameters
  std::string can_interface_;
  double publish_rate_;
  std::chrono::milliseconds timeout_ms_;

  // SocketCAN and Sygnal components
  std::shared_ptr<polymath::socketcan::SocketcanAdapter> socketcan_adapter_;
  std::unique_ptr<polymath::sygnal::SygnalInterfaceSocketcan> sygnal_interface_;

  // ROS2 components
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<sygnal_can_msgs::srv::SetControlState>::SharedPtr set_control_state_service_;
  rclcpp::Service<sygnal_can_msgs::srv::SendControlCommand>::SharedPtr send_control_command_service_;
  rclcpp::Service<sygnal_can_msgs::srv::SendRelayCommand>::SharedPtr send_relay_command_service_;

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sygnal_can_msgs::msg::McmHeartbeat>::SharedPtr mcm0_heartbeat_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sygnal_can_msgs::msg::McmHeartbeat>::SharedPtr mcm1_heartbeat_pub_;

  rclcpp::Subscription<sygnal_can_msgs::msg::ControlCommand>::SharedPtr control_command_sub_;

  // Current MCM state storage
  std::optional<sygnal_can_msgs::msg::McmHeartbeat> current_mcm0_state_;
  std::optional<sygnal_can_msgs::msg::McmHeartbeat> current_mcm1_state_;
};

}  // namespace polymath::sygnal

RCLCPP_COMPONENTS_REGISTER_NODE(polymath::sygnal::SygnalCanInterfaceNode)

#endif  // SYGNAL_CAN_INTERFACE_NODE__SYGNAL_CAN_INTERFACE_NODE_HPP_
