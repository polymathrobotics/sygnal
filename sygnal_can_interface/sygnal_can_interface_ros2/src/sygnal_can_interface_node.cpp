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

#include "sygnal_can_interface_ros2/sygnal_can_interface_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <magic_enum.hpp>

namespace polymath::sygnal
{

SygnalCanInterfaceNode::SygnalCanInterfaceNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("sygnal_can_interface_node", "", options)
{
  // Declare parameters
  declare_parameter("can_interface", std::string("can0"));
  declare_parameter("publish_rate", 3.0);  // Hz
  declare_parameter("timeout_ms", 500);  // ms
}

SygnalCanInterfaceNode::~SygnalCanInterfaceNode()
{
  if (socketcan_adapter_) {
    socketcan_adapter_->joinReceptionThread();
    socketcan_adapter_->closeSocket();
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SygnalCanInterfaceNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  // Get parameters
  can_interface_ = get_parameter("can_interface").as_string();
  publish_rate_ = get_parameter("publish_rate").as_double();
  auto timeout_ms = get_parameter("timeout_ms").as_int();
  timeout_ms_ = std::chrono::milliseconds(timeout_ms);

  RCLCPP_INFO(get_logger(), "Configuring Sygnal CAN Interface node with CAN interface: %s", can_interface_.c_str());

  try {
    // Initialize SocketCAN adapter
    socketcan_adapter_ = std::make_shared<polymath::socketcan::SocketcanAdapter>(can_interface_);

    if (!socketcan_adapter_->openSocket()) {
      RCLCPP_ERROR(get_logger(), "Failed to open SocketCAN interface: %s", can_interface_.c_str());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    // Initialize Sygnal Interface SocketCAN controller
    sygnal_interface_ = std::make_unique<polymath::sygnal::SygnalInterfaceSocketcan>(socketcan_adapter_);

    // Set up callback to parse incoming messages
    socketcan_adapter_->setOnReceiveCallback(
      [this](std::unique_ptr<const polymath::socketcan::CanFrame> frame) { sygnal_interface_->parse(*frame); });

    // Start receiving messages
    if (!socketcan_adapter_->startReceptionThread()) {
      RCLCPP_ERROR(get_logger(), "Failed to start SocketCAN reception thread");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    // Create publishers
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(10));
    mcm_heartbeat_pub_ = create_publisher<sygnal_can_msgs::msg::McmHeartbeat>("~/mcm_heartbeat", rclcpp::QoS(10));

    // Create services
    set_control_state_service_ = create_service<sygnal_can_msgs::srv::SetControlState>(
      "~/set_control_state",
      std::bind(&SygnalCanInterfaceNode::setControlStateCallback, this, std::placeholders::_1, std::placeholders::_2));

    send_control_command_service_ = create_service<sygnal_can_msgs::srv::SendControlCommand>(
      "~/send_control_command",
      std::bind(
        &SygnalCanInterfaceNode::sendControlCommandCallback, this, std::placeholders::_1, std::placeholders::_2));

    send_relay_command_service_ = create_service<sygnal_can_msgs::srv::SendRelayCommand>(
      "~/send_relay_command",
      std::bind(&SygnalCanInterfaceNode::sendRelayCommandCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Create timer for publishing
    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&SygnalCanInterfaceNode::timerCallback, this));

    RCLCPP_INFO(get_logger(), "Sygnal CAN Interface node configured successfully");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during configuration: %s", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SygnalCanInterfaceNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  // Activate publishers
  diagnostics_pub_->on_activate();
  mcm_heartbeat_pub_->on_activate();

  RCLCPP_INFO(get_logger(), "Sygnal CAN Interface node activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SygnalCanInterfaceNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  // Deactivate publishers
  diagnostics_pub_->on_deactivate();
  mcm_heartbeat_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Sygnal CAN Interface node deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SygnalCanInterfaceNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  // Clean up resources
  timer_.reset();
  set_control_state_service_.reset();
  send_control_command_service_.reset();
  send_relay_command_service_.reset();
  diagnostics_pub_.reset();
  mcm_heartbeat_pub_.reset();

  if (socketcan_adapter_) {
    socketcan_adapter_->joinReceptionThread();
    socketcan_adapter_->closeSocket();
  }

  sygnal_interface_.reset();
  socketcan_adapter_.reset();

  RCLCPP_INFO(get_logger(), "Sygnal CAN Interface node cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void SygnalCanInterfaceNode::timerCallback()
{
  /// TODO: (Zeerek) Get MCM state from sygnal_interface_ and publish
  /// TODO: (Zeerek) Create and publish diagnostics message
  RCLCPP_DEBUG(get_logger(), "Timer callback - publishing MCM heartbeat and diagnostics");
}

void SygnalCanInterfaceNode::setControlStateCallback(
  const std::shared_ptr<sygnal_can_msgs::srv::SetControlState::Request> request,
  std::shared_ptr<sygnal_can_msgs::srv::SetControlState::Response> response)
{
  /// TODO: (Zeerek) Implement control state command
  RCLCPP_INFO(
    get_logger(),
    "Set control state service called - bus_id: %d, interface_id: %d, state: %d",
    request->bus_id,
    request->interface_id,
    request->control_state);
  response->success = false;
  response->message = "Not implemented yet";
}

void SygnalCanInterfaceNode::sendControlCommandCallback(
  const std::shared_ptr<sygnal_can_msgs::srv::SendControlCommand::Request> request,
  std::shared_ptr<sygnal_can_msgs::srv::SendControlCommand::Response> response)
{
  /// TODO: (Zeerek) Implement control command
  RCLCPP_INFO(
    get_logger(),
    "Send control command service called - bus_id: %d, interface_id: %d, value: %f",
    request->bus_id,
    request->interface_id,
    request->value);
  response->success = false;
  response->message = "Not implemented yet";
}

void SygnalCanInterfaceNode::sendRelayCommandCallback(
  const std::shared_ptr<sygnal_can_msgs::srv::SendRelayCommand::Request> request,
  std::shared_ptr<sygnal_can_msgs::srv::SendRelayCommand::Response> response)
{
  /// TODO: (Zeerek) Implement relay command
  RCLCPP_INFO(
    get_logger(),
    "Send relay command service called - bus_id: %d, subsystem_id: %d, state: %d",
    request->bus_id,
    request->subsystem_id,
    request->relay_state);
  response->success = false;
  response->message = "Not implemented yet";
}

diagnostic_msgs::msg::DiagnosticArray SygnalCanInterfaceNode::createDiagnosticsMessage()
{
  /// TODO: (Zeerek) Implement diagnostics message creation
  diagnostic_msgs::msg::DiagnosticArray diagnostics;
  diagnostics.header.stamp = now();
  return diagnostics;
}

}  // namespace polymath::sygnal
