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

#include "sygnal_can_interface_lib/sygnal_command_interface.hpp"
#include "sygnal_can_interface_lib/sygnal_mcm_interface.hpp"

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
  // Get MCM states from sygnal_interface_
  auto mcm0_state = sygnal_interface_->get_sygnal_mcm0_state();
  auto mcm1_state = sygnal_interface_->get_sygnal_mcm1_state();
  auto interface_states = sygnal_interface_->get_interface_states();

  // Create and populate MCM heartbeat message
  sygnal_can_msgs::msg::McmHeartbeat heartbeat_msg;
  heartbeat_msg.header.stamp = now();
  heartbeat_msg.mcm0_state = static_cast<uint8_t>(mcm0_state);
  heartbeat_msg.mcm1_state = static_cast<uint8_t>(mcm1_state);

  // Convert interface states array to vector
  heartbeat_msg.interface_states.clear();
  for (const auto & state : interface_states) {
    heartbeat_msg.interface_states.push_back(static_cast<uint8_t>(state));
  }

  // Store current state and publish
  current_mcm_state_ = heartbeat_msg;
  mcm_heartbeat_pub_->publish(heartbeat_msg);

  // Create and publish diagnostics
  auto diagnostics = createDiagnosticsMessage();
  diagnostics_pub_->publish(diagnostics);

  RCLCPP_DEBUG(get_logger(), "Published MCM heartbeat and diagnostics");
}

void SygnalCanInterfaceNode::setControlStateCallback(
  const std::shared_ptr<sygnal_can_msgs::srv::SetControlState::Request> request,
  std::shared_ptr<sygnal_can_msgs::srv::SetControlState::Response> response)
{
  RCLCPP_INFO(
    get_logger(),
    "Set control state service called - bus_id: %d, interface_id: %d, state: %d",
    request->bus_id,
    request->interface_id,
    request->control_state);

  // Convert uint8 to SygnalControlState enum
  SygnalControlState control_state =
    request->control_state == 1 ? SygnalControlState::MCM_CONTROL : SygnalControlState::HUMAN_CONTROL;

  // Send control state command
  std::string error_message;
  auto result = sygnal_interface_->sendControlStateCommand(
    request->bus_id, request->interface_id, control_state, request->expect_reply, error_message);

  if (!result.success) {
    response->success = false;
    response->message = "Failed to send control state command: " + error_message;
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  // If we expect a reply, wait for response with timeout
  if (request->expect_reply && result.response_future.has_value()) {
    auto future = std::move(result.response_future.value());
    auto status = future.wait_for(timeout_ms_);

    if (status == std::future_status::ready) {
      auto command_response = future.get();
      RCLCPP_INFO(
        get_logger(),
        "Control state command acknowledged - bus_id: %d, interface_id: %d",
        command_response.bus_id,
        command_response.interface_id);
      response->success = true;
      response->message = "Control state command successful";
    } else {
      response->success = false;
      response->message = "Timeout waiting for control state command response";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    }
  } else {
    // Fire-and-forget, command sent successfully
    response->success = true;
    response->message = "Control state command sent (no reply expected)";
  }
}

void SygnalCanInterfaceNode::sendControlCommandCallback(
  const std::shared_ptr<sygnal_can_msgs::srv::SendControlCommand::Request> request,
  std::shared_ptr<sygnal_can_msgs::srv::SendControlCommand::Response> response)
{
  RCLCPP_INFO(
    get_logger(),
    "Send control command service called - bus_id: %d, interface_id: %d, value: %f",
    request->bus_id,
    request->interface_id,
    request->value);

  // Send control command
  std::string error_message;
  auto result = sygnal_interface_->sendControlCommand(
    request->bus_id, request->interface_id, request->value, request->expect_reply, error_message);

  if (!result.success) {
    response->success = false;
    response->message = "Failed to send control command: " + error_message;
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  // If we expect a reply, wait for response with timeout
  if (request->expect_reply && result.response_future.has_value()) {
    auto future = std::move(result.response_future.value());
    auto status = future.wait_for(timeout_ms_);

    if (status == std::future_status::ready) {
      auto command_response = future.get();
      RCLCPP_INFO(
        get_logger(),
        "Control command acknowledged - bus_id: %d, interface_id: %d, value: %f",
        command_response.bus_id,
        command_response.interface_id,
        command_response.value);
      response->success = true;
      response->message = "Control command successful";
    } else {
      response->success = false;
      response->message = "Timeout waiting for control command response";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    }
  } else {
    // Fire-and-forget, command sent successfully
    response->success = true;
    response->message = "Control command sent (no reply expected)";
  }
}

void SygnalCanInterfaceNode::sendRelayCommandCallback(
  const std::shared_ptr<sygnal_can_msgs::srv::SendRelayCommand::Request> request,
  std::shared_ptr<sygnal_can_msgs::srv::SendRelayCommand::Response> response)
{
  RCLCPP_INFO(
    get_logger(),
    "Send relay command service called - bus_id: %d, subsystem_id: %d, state: %d",
    request->bus_id,
    request->subsystem_id,
    request->relay_state);

  // Convert uint8 to bool (1 = enable, 0 = disable)
  bool relay_state = (request->relay_state == 1);

  // Send relay command
  std::string error_message;
  auto result = sygnal_interface_->sendRelayCommand(
    request->bus_id, request->subsystem_id, relay_state, request->expect_reply, error_message);

  if (!result.success) {
    response->success = false;
    response->message = "Failed to send relay command: " + error_message;
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  // If we expect a reply, wait for response with timeout
  if (request->expect_reply && result.response_future.has_value()) {
    auto future = std::move(result.response_future.value());
    auto status = future.wait_for(timeout_ms_);

    if (status == std::future_status::ready) {
      auto command_response = future.get();
      RCLCPP_INFO(
        get_logger(),
        "Relay command acknowledged - bus_id: %d, subsystem_id: %d",
        command_response.bus_id,
        static_cast<uint8_t>(command_response.value));
      response->success = true;
      response->message = "Relay command successful";
    } else {
      response->success = false;
      response->message = "Timeout waiting for relay command response";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    }
  } else {
    // Fire-and-forget, command sent successfully
    response->success = true;
    response->message = "Relay command sent (no reply expected)";
  }
}

diagnostic_msgs::msg::DiagnosticArray SygnalCanInterfaceNode::createDiagnosticsMessage()
{
  diagnostic_msgs::msg::DiagnosticArray array_msg;

  // Set header
  array_msg.header.stamp = get_clock()->now();
  array_msg.header.frame_id = "";

  // Get current MCM states
  auto mcm0_state = sygnal_interface_->get_sygnal_mcm0_state();
  auto mcm1_state = sygnal_interface_->get_sygnal_mcm1_state();
  auto interface_states = sygnal_interface_->get_interface_states();

  // Create MCM 0 diagnostics
  {
    diagnostic_msgs::msg::DiagnosticStatus mcm0_diag;
    mcm0_diag.name = "sygnal_mcm0";
    mcm0_diag.hardware_id = "sygnal_can_interface";

    // Determine diagnostic level based on state
    if (mcm0_state == SygnalSystemState::MCM_CONTROL || mcm0_state == SygnalSystemState::HUMAN_CONTROL) {
      mcm0_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      mcm0_diag.message = "MCM0 operating normally: " + sygnalSystemStateToString(mcm0_state);
    } else if (
      mcm0_state == SygnalSystemState::FAIL_OPERATIONAL_1 || mcm0_state == SygnalSystemState::FAIL_OPERATIONAL_2)
    {
      mcm0_diag.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      mcm0_diag.message = "MCM0 in degraded mode: " + sygnalSystemStateToString(mcm0_state);
    } else {
      mcm0_diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      mcm0_diag.message = "MCM0 fault: " + sygnalSystemStateToString(mcm0_state);
    }

    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "state";
    kv.value = sygnalSystemStateToString(mcm0_state);
    mcm0_diag.values.push_back(kv);

    array_msg.status.push_back(mcm0_diag);
  }

  // Create MCM 1 diagnostics
  {
    diagnostic_msgs::msg::DiagnosticStatus mcm1_diag;
    mcm1_diag.name = "sygnal_mcm1";
    mcm1_diag.hardware_id = "sygnal_can_interface";

    // Determine diagnostic level based on state
    if (mcm1_state == SygnalSystemState::MCM_CONTROL || mcm1_state == SygnalSystemState::HUMAN_CONTROL) {
      mcm1_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      mcm1_diag.message = "MCM1 operating normally: " + sygnalSystemStateToString(mcm1_state);
    } else if (
      mcm1_state == SygnalSystemState::FAIL_OPERATIONAL_1 || mcm1_state == SygnalSystemState::FAIL_OPERATIONAL_2)
    {
      mcm1_diag.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      mcm1_diag.message = "MCM1 in degraded mode: " + sygnalSystemStateToString(mcm1_state);
    } else {
      mcm1_diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      mcm1_diag.message = "MCM1 fault: " + sygnalSystemStateToString(mcm1_state);
    }

    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "state";
    kv.value = sygnalSystemStateToString(mcm1_state);
    mcm1_diag.values.push_back(kv);

    array_msg.status.push_back(mcm1_diag);
  }

  // Create diagnostics for each interface (0-4)
  for (size_t i = 0; i < interface_states.size(); ++i) {
    diagnostic_msgs::msg::DiagnosticStatus interface_diag;
    interface_diag.name = "sygnal_interface_" + std::to_string(i);
    interface_diag.hardware_id = "sygnal_can_interface";

    auto state = interface_states[i];

    // Determine diagnostic level based on state
    if (state == SygnalSystemState::MCM_CONTROL || state == SygnalSystemState::HUMAN_CONTROL) {
      interface_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      interface_diag.message =
        "Interface " + std::to_string(i) + " operating normally: " + sygnalSystemStateToString(state);
    } else if (state == SygnalSystemState::FAIL_OPERATIONAL_1 || state == SygnalSystemState::FAIL_OPERATIONAL_2) {
      interface_diag.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      interface_diag.message =
        "Interface " + std::to_string(i) + " in degraded mode: " + sygnalSystemStateToString(state);
    } else {
      interface_diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      interface_diag.message = "Interface " + std::to_string(i) + " fault: " + sygnalSystemStateToString(state);
    }

    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "state";
    kv.value = sygnalSystemStateToString(state);
    interface_diag.values.push_back(kv);

    array_msg.status.push_back(interface_diag);
  }

  return array_msg;
}

}  // namespace polymath::sygnal
