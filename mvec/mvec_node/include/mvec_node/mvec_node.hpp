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

#include <memory>
#include <string>
#include <thread>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "mvec_lib/mvec_relay_socketcan.hpp"
#include "mvec_msgs/msg/error_status.hpp"
#include "mvec_msgs/msg/fuse_status.hpp"
#include "mvec_msgs/msg/population_status.hpp"
#include "mvec_msgs/msg/relay_query.hpp"
#include "mvec_msgs/msg/relay_status.hpp"
#include "mvec_msgs/srv/set_relay_state.hpp"
#include "rclcpp/rclcpp.hpp"
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
  void timer_callback();

  /// @brief Service callback to set relay state
  /// @param request Service request containing relay ID and state
  /// @param response Service response with success flag
  void set_relay_state_callback(
    const std::shared_ptr<mvec_msgs::srv::SetRelayState::Request> request,
    std::shared_ptr<mvec_msgs::srv::SetRelayState::Response> response);

  /// @brief Create diagnostics array from MVEC status
  /// @return Diagnostic array message
  diagnostic_msgs::msg::DiagnosticArray create_diagnostics_message();

  // Parameters
  std::string can_interface_;
  double publish_rate_;

  // SocketCAN and MVEC components
  std::shared_ptr<polymath::socketcan::SocketcanAdapter> socketcan_adapter_;
  std::unique_ptr<polymath::sygnal::MvecRelaySocketcan> mvec_socketcan_;

  // ROS2 components
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<mvec_msgs::srv::SetRelayState>::SharedPtr set_relay_service_;

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
};

}  // namespace polymath::mvec

#endif  // MVEC_NODE__MVEC_NODE_HPP_
