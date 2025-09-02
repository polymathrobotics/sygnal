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

#include "mvec_node/mvec_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <magic_enum.hpp>

#include "mvec_lib/core/mvec_constants.hpp"

namespace polymath::mvec
{

MvecNode::MvecNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("mvec_node", "", options)
{
  // Declare parameters
  declare_parameter("can_interface", std::string("can0"));
  declare_parameter("publish_rate", 10.0);  // Hz
}

MvecNode::~MvecNode()
{
  if (socketcan_adapter_) {
    socketcan_adapter_->joinReceptionThread();
    socketcan_adapter_->closeSocket();
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MvecNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  // Get parameters
  can_interface_ = get_parameter("can_interface").as_string();
  publish_rate_ = get_parameter("publish_rate").as_double();

  RCLCPP_INFO(get_logger(), "Configuring MVEC node with CAN interface: %s", can_interface_.c_str());

  try {
    // Initialize SocketCAN adapter
    socketcan_adapter_ = std::make_shared<polymath::socketcan::SocketcanAdapter>(can_interface_);

    if (!socketcan_adapter_->openSocket()) {
      RCLCPP_ERROR(get_logger(), "Failed to open SocketCAN interface: %s", can_interface_.c_str());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    // Initialize MVEC SocketCAN controller
    mvec_socketcan_ = std::make_unique<polymath::sygnal::MvecRelaySocketcan>(socketcan_adapter_);

    // Set up callback to parse incoming messages
    socketcan_adapter_->setOnReceiveCallback(
      [this](std::unique_ptr<const polymath::socketcan::CanFrame> frame) { mvec_socketcan_->parse(*frame); });

    // Start receiving messages
    if (!socketcan_adapter_->startReceptionThread()) {
      RCLCPP_ERROR(get_logger(), "Failed to start SocketCAN reception thread");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    // Create publishers
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(10));

    // Create service
    set_relay_service_ = create_service<mvec_msgs::srv::SetRelayState>(
      "~/set_relay_state",
      std::bind(&MvecNode::set_relay_state_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Create timer for publishing
    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period), std::bind(&MvecNode::timer_callback, this));

    RCLCPP_INFO(get_logger(), "MVEC node configured successfully");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during configuration: %s", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MvecNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  // Activate publishers
  diagnostics_pub_->on_activate();

  RCLCPP_INFO(get_logger(), "MVEC node activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MvecNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  // Deactivate publishers
  diagnostics_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "MVEC node deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MvecNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  // Clean up resources
  timer_.reset();
  set_relay_service_.reset();
  diagnostics_pub_.reset();

  if (socketcan_adapter_) {
    socketcan_adapter_->joinReceptionThread();
    socketcan_adapter_->closeSocket();
    socketcan_adapter_.reset();
  }
  mvec_socketcan_.reset();

  RCLCPP_INFO(get_logger(), "MVEC node cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void MvecNode::timer_callback()
{
  // Publish diagnostics array
  auto diagnostics_msg = create_diagnostics_message();
  diagnostics_pub_->publish(diagnostics_msg);
}

void MvecNode::set_relay_state_callback(
  const std::shared_ptr<mvec_msgs::srv::SetRelayState::Request> request,
  std::shared_ptr<mvec_msgs::srv::SetRelayState::Response> response)
{
  RCLCPP_INFO(get_logger(), "Setting relay %d to state %d", request->relay_id, request->relay_state);

  try {
    // Validate relay ID
    if (request->relay_id >= polymath::sygnal::MvecHardware::MAX_NUMBER_RELAYS) {
      response->success = false;
      response->message = "Invalid relay ID: " + std::to_string(request->relay_id);
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }

    // Validate relay state
    if (request->relay_state > 1) {
      response->success = false;
      response->message = "Invalid relay state: " + std::to_string(request->relay_state);
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }

    // Clear relay command before setting new state
    mvec_socketcan_->clear_relay();

    // Set the specific relay state
    mvec_socketcan_->set_relay_in_command(request->relay_id, request->relay_state);

    // Send command and wait for response
    auto future = mvec_socketcan_->send_relay_command();
    auto status = future.wait_for(std::chrono::seconds(5));

    if (status == std::future_status::ready) {
      auto command_reply = future.get();
      if (command_reply.get_success() == 1) {
        response->success = true;
        response->message = "Relay command successful";
        RCLCPP_INFO(get_logger(), "Successfully set relay %d to state %d", request->relay_id, request->relay_state);
      } else {
        response->success = false;
        response->message = "MVEC device rejected relay command";
        RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      }
    } else {
      response->success = false;
      response->message = "Timeout waiting for relay command response";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    }
  } catch (const std::exception & e) {
    response->success = false;
    response->message = "Exception during relay command: " + std::string(e.what());
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
  }
}

diagnostic_msgs::msg::DiagnosticArray MvecNode::create_diagnostics_message()
{
  diagnostic_msgs::msg::DiagnosticArray array_msg;

  // Set header
  array_msg.header.stamp = get_clock()->now();
  array_msg.header.frame_id = "";

  // Get status messages
  auto relay_status = mvec_socketcan_->get_last_relay_status();
  auto fuse_status = mvec_socketcan_->get_last_fuse_status();
  auto error_status = mvec_socketcan_->get_last_error_status();

  // Create relay diagnostics status
  if (relay_status.has_value() && relay_status->is_valid()) {
    for (int i = 0; i < polymath::sygnal::MvecHardware::MAX_NUMBER_RELAYS; ++i) {
      auto status = relay_status->get_relay_status(i);
      if (status != polymath::sygnal::MvecRelayStatus::RELAY_LOCATION_NOT_USED) {
        diagnostic_msgs::msg::DiagnosticStatus relay_diag;
        relay_diag.name = "mvec_relay_" + std::to_string(i);
        relay_diag.hardware_id = "mvec";

        if (status == polymath::sygnal::MvecRelayStatus::OKAY) {
          relay_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
          relay_diag.message = "Relay operating normally";
        } else {
          relay_diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
          relay_diag.message = "Relay fault: " + std::string(magic_enum::enum_name(status));
        }

        // Add status value
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = "status";
        kv.value = std::string(magic_enum::enum_name(status));
        relay_diag.values.push_back(kv);

        array_msg.status.push_back(relay_diag);
      }
    }
  }

  // Create fuse diagnostics status
  if (fuse_status.has_value() && fuse_status->is_valid()) {
    for (int i = 0; i < polymath::sygnal::MvecHardware::MAX_NUMBER_FUSES; ++i) {
      auto status = fuse_status->get_fuse_status(i);
      if (status != polymath::sygnal::MvecFuseStatus::NOT_USED) {
        diagnostic_msgs::msg::DiagnosticStatus fuse_diag;
        fuse_diag.name = "mvec_fuse_" + std::to_string(i);
        fuse_diag.hardware_id = "mvec";

        if (status == polymath::sygnal::MvecFuseStatus::NO_FAULT) {
          fuse_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
          fuse_diag.message = "Fuse operating normally";
        } else if (status == polymath::sygnal::MvecFuseStatus::BLOWN) {
          fuse_diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
          fuse_diag.message = "Fuse blown";
        } else {
          fuse_diag.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
          fuse_diag.message = "Fuse issue: " + std::string(magic_enum::enum_name(status));
        }

        // Add status value
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = "status";
        kv.value = std::string(magic_enum::enum_name(status));
        fuse_diag.values.push_back(kv);

        array_msg.status.push_back(fuse_diag);
      }
    }
  }

  // Create general MVEC system status
  diagnostic_msgs::msg::DiagnosticStatus system_diag;
  system_diag.name = "mvec_system";
  system_diag.hardware_id = "mvec";

  bool has_valid_data = (relay_status.has_value() && relay_status->is_valid()) ||
                        (fuse_status.has_value() && fuse_status->is_valid()) ||
                        (error_status.has_value() && error_status->is_valid());

  if (!has_valid_data) {
    system_diag.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    system_diag.message = "No valid MVEC status data received";
  } else if (error_status.has_value() && error_status->is_valid() && error_status->get_error_bits() != 0) {
    system_diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    system_diag.message = "System errors detected";

    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "error_bits";
    kv.value = "0x" + std::to_string(error_status->get_error_bits());
    system_diag.values.push_back(kv);
  } else {
    system_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    system_diag.message = "MVEC system operating normally";
  }

  array_msg.status.push_back(system_diag);

  return array_msg;
}

}  // namespace polymath::mvec
