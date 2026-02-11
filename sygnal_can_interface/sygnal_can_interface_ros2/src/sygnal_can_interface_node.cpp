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
  param_listener_ = std::make_shared<sygnal_can_interface_ros2::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();
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
  // Get parameters (refresh in case they changed)
  params_ = param_listener_->get_params();

  RCLCPP_INFO(
    get_logger(), "Configuring Sygnal CAN Interface node with CAN interface: %s", params_.can_interface.c_str());

  try {
    // Initialize SocketCAN adapter
    socketcan_adapter_ = std::make_shared<polymath::socketcan::SocketcanAdapter>(params_.can_interface);

    if (!socketcan_adapter_->openSocket()) {
      RCLCPP_ERROR(get_logger(), "Failed to open SocketCAN interface: %s", params_.can_interface.c_str());
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

    // Initialize MCM heartbeat publishers for all 4 devices
    mcm_heartbeat_entries_[0] = {DEFAULT_MCM_BUS_ADDRESS, 0, {}, {}};
    mcm_heartbeat_entries_[1] = {DEFAULT_MCM_BUS_ADDRESS, 1, {}, {}};
    mcm_heartbeat_entries_[2] = {SECONDARY_MCM_BUS_ADDRESS, 0, {}, {}};
    mcm_heartbeat_entries_[3] = {SECONDARY_MCM_BUS_ADDRESS, 1, {}, {}};

    for (auto & entry : mcm_heartbeat_entries_) {
      std::string topic =
        "~/mcm_heartbeat_bus" + std::to_string(entry.bus_id) + "_sub" + std::to_string(entry.subsystem_id);
      entry.publisher = create_publisher<sygnal_can_msgs::msg::McmHeartbeat>(topic, rclcpp::QoS(10));
    }

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
    auto period = std::chrono::duration<double>(1.0 / params_.publish_rate);
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
  for (auto & entry : mcm_heartbeat_entries_) {
    entry.publisher->on_activate();
  }

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
  for (auto & entry : mcm_heartbeat_entries_) {
    entry.publisher->on_deactivate();
  }

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
  for (auto & entry : mcm_heartbeat_entries_) {
    entry.publisher.reset();
    entry.current_state.reset();
  }

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
  for (auto & entry : mcm_heartbeat_entries_) {
    auto mcm_state_opt = sygnal_interface_->get_sygnal_mcm_state(entry.bus_id, entry.subsystem_id);
    auto interface_states_opt = sygnal_interface_->get_interface_state(entry.bus_id, entry.subsystem_id);

    if (!mcm_state_opt || !interface_states_opt) {
      continue;
    }

    sygnal_can_msgs::msg::McmHeartbeat msg;
    msg.header.stamp = now();
    msg.state = static_cast<uint8_t>(mcm_state_opt.value());

    for (const auto & state : interface_states_opt.value()) {
      msg.interface_states.push_back(static_cast<uint8_t>(state));
    }

    entry.current_state = msg;
    entry.publisher->publish(msg);
  }

  // Create and publish diagnostics
  auto diagnostics = createDiagnosticsMessage();
  diagnostics_pub_->publish(diagnostics);

  RCLCPP_DEBUG(get_logger(), "Published MCM heartbeats and diagnostics");
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
    auto status = future.wait_for(std::chrono::milliseconds(params_.timeout_ms));

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
    auto status = future.wait_for(std::chrono::milliseconds(params_.timeout_ms));

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
    auto status = future.wait_for(std::chrono::milliseconds(params_.timeout_ms));

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

  for (const auto & entry : mcm_heartbeat_entries_) {
    auto mcm_state_opt = sygnal_interface_->get_sygnal_mcm_state(entry.bus_id, entry.subsystem_id);
    auto interface_states_opt = sygnal_interface_->get_interface_state(entry.bus_id, entry.subsystem_id);

    if (!mcm_state_opt) {
      continue;
    }

    auto mcm_state = mcm_state_opt.value();
    std::string mcm_name =
      "sygnal_mcm_bus" + std::to_string(entry.bus_id) + "_sub" + std::to_string(entry.subsystem_id);

    // Create MCM diagnostic
    diagnostic_msgs::msg::DiagnosticStatus mcm_diag;
    mcm_diag.name = mcm_name;
    mcm_diag.hardware_id = "sygnal_can_interface";

    if (mcm_state == SygnalSystemState::MCM_CONTROL || mcm_state == SygnalSystemState::HUMAN_CONTROL) {
      mcm_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    } else if (mcm_state == SygnalSystemState::FAIL_OPERATIONAL_1 || mcm_state == SygnalSystemState::FAIL_OPERATIONAL_2)
    {
      mcm_diag.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    } else {
      mcm_diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }

    mcm_diag.message = mcm_name + " state: " + sygnalSystemStateToString(mcm_state);

    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "state";
    kv.value = sygnalSystemStateToString(mcm_state);
    mcm_diag.values.push_back(kv);

    array_msg.status.push_back(mcm_diag);

    // Create per-interface diagnostics if available
    if (interface_states_opt) {
      const auto & interface_states = interface_states_opt.value();
      for (size_t i = 0; i < interface_states.size(); ++i) {
        auto state = interface_states[i];

        diagnostic_msgs::msg::DiagnosticStatus interface_diag;
        interface_diag.name = mcm_name + "_interface_" + std::to_string(i);
        interface_diag.hardware_id = "sygnal_can_interface";

        if (state == SygnalSystemState::MCM_CONTROL || state == SygnalSystemState::HUMAN_CONTROL) {
          interface_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        } else if (state == SygnalSystemState::FAIL_OPERATIONAL_1 || state == SygnalSystemState::FAIL_OPERATIONAL_2) {
          interface_diag.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        } else {
          interface_diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        }

        interface_diag.message = "Interface " + std::to_string(i) + " state: " + sygnalSystemStateToString(state);

        diagnostic_msgs::msg::KeyValue interface_kv;
        interface_kv.key = "state";
        interface_kv.value = sygnalSystemStateToString(state);
        interface_diag.values.push_back(interface_kv);

        array_msg.status.push_back(interface_diag);
      }
    }
  }

  return array_msg;
}

}  // namespace polymath::sygnal
