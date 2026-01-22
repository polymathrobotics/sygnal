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
    mcm0_heartbeat_pub_ = create_publisher<sygnal_can_msgs::msg::McmHeartbeat>("~/mcm0_heartbeat", rclcpp::QoS(10));
    mcm1_heartbeat_pub_ = create_publisher<sygnal_can_msgs::msg::McmHeartbeat>("~/mcm1_heartbeat", rclcpp::QoS(10));

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
  mcm0_heartbeat_pub_->on_activate();
  mcm1_heartbeat_pub_->on_activate();

  create_subscription<sygnal_can_msgs::msg::ControlCommand>(
    "~/control_command",
    rclcpp::QoS(10),
    std::bind(&SygnalCanInterfaceNode::controlCommandCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Sygnal CAN Interface node activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SygnalCanInterfaceNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  // Deactivate publishers
  diagnostics_pub_->on_deactivate();
  mcm0_heartbeat_pub_->on_deactivate();
  mcm1_heartbeat_pub_->on_deactivate();

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
  mcm0_heartbeat_pub_.reset();
  mcm1_heartbeat_pub_.reset();

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
  auto interface_states_0 = sygnal_interface_->get_interface_states_0();
  auto interface_states_1 = sygnal_interface_->get_interface_states_1();

  // Create and populate MCM 0 heartbeat message
  sygnal_can_msgs::msg::McmHeartbeat mcm0_msg;
  mcm0_msg.header.stamp = now();
  mcm0_msg.state = static_cast<uint8_t>(mcm0_state);

  // Convert interface states array to vector for MCM 0
  mcm0_msg.interface_states.clear();
  for (const auto & state : interface_states_0) {
    mcm0_msg.interface_states.push_back(static_cast<uint8_t>(state));
  }

  // Store current state and publish MCM 0
  current_mcm0_state_ = mcm0_msg;
  mcm0_heartbeat_pub_->publish(mcm0_msg);

  // Create and populate MCM 1 heartbeat message
  sygnal_can_msgs::msg::McmHeartbeat mcm1_msg;
  mcm1_msg.header.stamp = now();
  mcm1_msg.state = static_cast<uint8_t>(mcm1_state);

  // Convert interface states array to vector for MCM 1
  mcm1_msg.interface_states.clear();
  for (const auto & state : interface_states_1) {
    mcm1_msg.interface_states.push_back(static_cast<uint8_t>(state));
  }

  // Store current state and publish MCM 1
  current_mcm1_state_ = mcm1_msg;
  mcm1_heartbeat_pub_->publish(mcm1_msg);

  // Create and publish diagnostics
  auto diagnostics = createDiagnosticsMessage();
  diagnostics_pub_->publish(diagnostics);

  RCLCPP_DEBUG(get_logger(), "Published MCM 0 and MCM 1 heartbeats and diagnostics");
}

void SygnalCanInterfaceNode::setControlStateCallback(
  const std::shared_ptr<sygnal_can_msgs::srv::SetControlState::Request> request,
  std::shared_ptr<sygnal_can_msgs::srv::SetControlState::Response> response)
{
  RCLCPP_INFO(
    get_logger(),
    "Set control state service called - bus_id: %d, interface_id: %d, subsystem_id: %d, state: %d",
    request->bus_id,
    request->interface_id,
    request->subsystem_id,
    request->control_state);

  // Convert uint8 to SygnalControlState enum
  SygnalControlState control_state =
    request->control_state == 1 ? SygnalControlState::MCM_CONTROL : SygnalControlState::HUMAN_CONTROL;

  // Send control state command
  std::string error_message;
  auto result = sygnal_interface_->sendControlStateCommand(
    request->bus_id, request->interface_id, request->subsystem_id, control_state, request->expect_reply, error_message);

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
    "Send control command service called - bus_id: %d, interface_id: %d, subsystem_id: %d, value: %f",
    request->command.bus_id,
    request->command.interface_id,
    request->command.subsystem_id,
    request->command.value);

  // Send control command
  std::string error_message;
  auto result = sygnal_interface_->sendControlCommand(
    request->command.bus_id,
    request->command.interface_id,
    request->command.subsystem_id,
    request->command.value,
    request->expect_reply,
    error_message);

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

void SygnalCanInterfaceNode::controlCommandCallback(const sygnal_can_msgs::msg::ControlCommand::UniquePtr msg)
{
  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    500,
    "Received control command - bus_id: %d, interface_id: %d, subsystem_id: %d, value: %f",
    msg->bus_id,
    msg->interface_id,
    msg->subsystem_id,
    msg->value);

  // Send control command without expecting a reply
  std::string error_message;
  auto result = sygnal_interface_->sendControlCommand(
    msg->bus_id, msg->interface_id, msg->subsystem_id, msg->value, false, error_message);

  if (!result.success) {
    RCLCPP_WARN(get_logger(), "Failed to send control command from subscription: %s", error_message.c_str());
  } else {
    RCLCPP_DEBUG(
      get_logger(),
      "Control command sent successfully from subscription - bus_id: %d, interface_id: %d",
      msg->bus_id,
      msg->interface_id);
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
  auto interface_states_0 = sygnal_interface_->get_interface_states_0();
  auto interface_states_1 = sygnal_interface_->get_interface_states_1();

  // Create MCM 0 diagnostics
  {
    diagnostic_msgs::msg::DiagnosticStatus mcm0_diag;
    mcm0_diag.name = "sygnal_mcm0";
    mcm0_diag.hardware_id = "sygnal_can_interface";

    // Determine diagnostic level based on state
    if (mcm0_state == SygnalSystemState::MCM_CONTROL || mcm0_state == SygnalSystemState::HUMAN_CONTROL) {
      mcm0_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    } else if (
      mcm0_state == SygnalSystemState::FAIL_OPERATIONAL_1 || mcm0_state == SygnalSystemState::FAIL_OPERATIONAL_2)
    {
      mcm0_diag.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    } else {
      mcm0_diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }

    mcm0_diag.message = "MCM0 state: " + sygnalSystemStateToString(mcm0_state);

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
    } else if (
      mcm1_state == SygnalSystemState::FAIL_OPERATIONAL_1 || mcm1_state == SygnalSystemState::FAIL_OPERATIONAL_2)
    {
      mcm1_diag.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    } else {
      mcm1_diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }

    mcm1_diag.message = "MCM1 state: " + sygnalSystemStateToString(mcm1_state);

    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "state";
    kv.value = sygnalSystemStateToString(mcm1_state);
    mcm1_diag.values.push_back(kv);

    array_msg.status.push_back(mcm1_diag);
  }

  // Create diagnostics for each interface (0-4) from both MCM subsystems
  for (size_t i = 0; i < interface_states_0.size(); ++i) {
    auto state_0 = interface_states_0[i];
    auto state_1 = interface_states_1[i];

    // Create diagnostic for this interface
    diagnostic_msgs::msg::DiagnosticStatus interface_diag;
    interface_diag.name = "sygnal_interface_" + std::to_string(i);
    interface_diag.hardware_id = "sygnal_can_interface";

    // Determine worst-case diagnostic level from both MCMs
    auto worst_state = state_0;
    if (static_cast<uint8_t>(state_1) > static_cast<uint8_t>(state_0)) {
      worst_state = state_1;
    }

    if (worst_state == SygnalSystemState::MCM_CONTROL || worst_state == SygnalSystemState::HUMAN_CONTROL) {
      interface_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    } else if (
      worst_state == SygnalSystemState::FAIL_OPERATIONAL_1 || worst_state == SygnalSystemState::FAIL_OPERATIONAL_2)
    {
      interface_diag.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    } else {
      interface_diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }

    // Build message showing states from both MCMs
    interface_diag.message = "Interface " + std::to_string(i) + " - MCM0: " + sygnalSystemStateToString(state_0) +
                             ", MCM1: " + sygnalSystemStateToString(state_1);

    // Add state values from both MCMs
    diagnostic_msgs::msg::KeyValue kv_mcm0;
    kv_mcm0.key = "mcm0_state";
    kv_mcm0.value = sygnalSystemStateToString(state_0);
    interface_diag.values.push_back(kv_mcm0);

    diagnostic_msgs::msg::KeyValue kv_mcm1;
    kv_mcm1.key = "mcm1_state";
    kv_mcm1.value = sygnalSystemStateToString(state_1);
    interface_diag.values.push_back(kv_mcm1);

    // Flag if states differ
    if (state_0 != state_1) {
      diagnostic_msgs::msg::KeyValue kv_mismatch;
      kv_mismatch.key = "state_mismatch";
      kv_mismatch.value = "true";
      interface_diag.values.push_back(kv_mismatch);
    }

    array_msg.status.push_back(interface_diag);
  }

  return array_msg;
}

}  // namespace polymath::sygnal
