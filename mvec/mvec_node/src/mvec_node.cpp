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

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <magic_enum.hpp>

#include "mvec_lib/core/mvec_constants.hpp"

namespace polymath::mvec
{

MvecNode::MvecNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("mvec_node", "", options)
{
  // Declare parameters
  declare_parameter("can_interface", std::string("can0"));
  declare_parameter("publish_rate", 3.0);  // Hz
  declare_parameter("timetout_ms", 500.0);  // ms

  // Declare and parse parameters for dynamic loading
  parsePresetParams();
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
  auto timeout_ms = get_parameter("timeout_ms").as_int();
  timeout_ms_ = std::chrono::milliseconds(timeout_ms);

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
    feedback_pub_ = create_publisher<mvec_msgs::msg::MvecFeedback>("~/feedback", rclcpp::QoS(10));

    // Create services
    set_single_relay_service_ = create_service<mvec_msgs::srv::SetSingleRelay>(
      "~/set_single_relay",
      std::bind(&MvecNode::setSingleRelayCallback, this, std::placeholders::_1, std::placeholders::_2));

    set_multi_relay_service_ = create_service<mvec_msgs::srv::SetMultiRelay>(
      "~/set_multi_relay",
      std::bind(&MvecNode::setMultiRelayCallback, this, std::placeholders::_1, std::placeholders::_2));

    trigger_preset_service_ = create_service<mvec_msgs::srv::TriggerPreset>(
      "~/trigger_preset",
      std::bind(&MvecNode::triggerPresetCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Create timer for publishing
    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period), std::bind(&MvecNode::timerCallback, this));

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
  feedback_pub_->on_activate();

  RCLCPP_INFO(get_logger(), "MVEC node activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MvecNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  // Deactivate publishers
  diagnostics_pub_->on_deactivate();
  feedback_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "MVEC node deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MvecNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  // Clean up resources
  timer_.reset();
  set_single_relay_service_.reset();
  set_multi_relay_service_.reset();
  trigger_preset_service_.reset();
  diagnostics_pub_.reset();
  feedback_pub_.reset();

  if (socketcan_adapter_) {
    socketcan_adapter_->joinReceptionThread();
    socketcan_adapter_->closeSocket();
    socketcan_adapter_.reset();
  }
  mvec_socketcan_.reset();

  RCLCPP_INFO(get_logger(), "MVEC node cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void MvecNode::timerCallback()
{
  // Query relay states asynchronously
  auto relay_state_future = mvec_socketcan_->get_relay_state();
  auto status = relay_state_future.wait_for(std::chrono::milliseconds(100));

  if (status == std::future_status::ready) {
    auto relay_query_reply = relay_state_future.get();
    if (relay_query_reply.is_valid()) {
      // Store current relay states
      mvec_msgs::msg::MvecFeedback feedback_msg;
      feedback_msg.header.stamp = get_clock()->now();
      feedback_msg.header.frame_id = "";
      feedback_msg.high_side_output_state = relay_query_reply.get_high_side_output_state();

      // Populate relay states
      for (int i = 0; i < polymath::sygnal::MvecHardware::MAX_NUMBER_RELAYS; ++i) {
        mvec_msgs::msg::Relay relay;
        relay.relay_id = static_cast<uint8_t>(i);
        relay.state = relay_query_reply.get_relay_state(i);
        feedback_msg.relay_states.push_back(relay);
      }

      current_relay_states_ = feedback_msg;
      RCLCPP_DEBUG(get_logger(), "Updated relay states from hardware");
    } else {
      RCLCPP_DEBUG(get_logger(), "Invalid relay query response received");
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Relay state query timed out");
  }

  // Publish diagnostics array
  auto diagnostics_msg = createDiagnosticsMessage();
  diagnostics_pub_->publish(diagnostics_msg);

  // Publish relay states feedback at end of timer
  if (current_relay_states_.has_value()) {
    feedback_pub_->publish(current_relay_states_.value());
  }
}

std::optional<std::string> MvecNode::set_single_relay(mvec_msgs::msg::Relay relay)
{
  try {
    // Validate relay ID
    if (relay.relay_id >= polymath::sygnal::MvecHardware::MAX_NUMBER_RELAYS) {
      std::string error_msg = "Invalid relay ID: " + std::to_string(relay.relay_id);
      RCLCPP_WARN(get_logger(), "%s", error_msg.c_str());
      return error_msg;
    }

    // Clear relay command before setting new state
    mvec_socketcan_->clear_relay();

    // Set the specific relay state
    mvec_socketcan_->set_relay_in_command(relay.relay_id, relay.state ? 1 : 0);

    // Send command and wait for response
    auto future = mvec_socketcan_->send_relay_command();
    auto status = future.wait_for(timeout_ms_);

    if (status == std::future_status::ready) {
      auto command_reply = future.get();
      if (command_reply.get_success() == 1) {
        RCLCPP_INFO(get_logger(), "Successfully set relay %d to state %s", relay.relay_id, relay.state ? "ON" : "OFF");
        return std::nullopt;  // Success
      } else {
        std::string error_msg = "MVEC device rejected relay command";
        RCLCPP_WARN(get_logger(), "%s", error_msg.c_str());
        return error_msg;
      }
    } else {
      std::string error_msg = "Timeout waiting for relay command response";
      RCLCPP_WARN(get_logger(), "%s", error_msg.c_str());
      return error_msg;
    }
  } catch (const std::exception & e) {
    std::string error_msg = "Exception during relay command: " + std::string(e.what());
    RCLCPP_ERROR(get_logger(), "%s", error_msg.c_str());
    return error_msg;
  }
}

std::optional<std::string> MvecNode::set_multi_relay(const std::vector<mvec_msgs::msg::Relay> & relays)
{
  try {
    // Validate all relay IDs first
    for (const auto & relay : relays) {
      if (relay.relay_id >= polymath::sygnal::MvecHardware::MAX_NUMBER_RELAYS) {
        std::string error_msg = "Invalid relay ID: " + std::to_string(relay.relay_id);
        RCLCPP_WARN(get_logger(), "%s", error_msg.c_str());
        return error_msg;
      }
    }

    // Clear relay command before setting new states
    mvec_socketcan_->clear_relay();

    // Set all relay states
    for (const auto & relay : relays) {
      mvec_socketcan_->set_relay_in_command(relay.relay_id, relay.state ? 1 : 0);
      RCLCPP_DEBUG(get_logger(), "Set relay %d to %s", relay.relay_id, relay.state ? "ON" : "OFF");
    }

    // Send command and wait for response
    auto future = mvec_socketcan_->send_relay_command();
    auto status = future.wait_for(timeout_ms_);

    if (status == std::future_status::ready) {
      auto command_reply = future.get();
      if (command_reply.get_success() == 1) {
        RCLCPP_INFO(get_logger(), "Successfully set %zu relays", relays.size());
        return std::nullopt;  // Success
      } else {
        std::string error_msg = "MVEC device rejected multi-relay command";
        RCLCPP_WARN(get_logger(), "%s", error_msg.c_str());
        return error_msg;
      }
    } else {
      std::string error_msg = "Timeout waiting for multi-relay command response";
      RCLCPP_WARN(get_logger(), "%s", error_msg.c_str());
      return error_msg;
    }
  } catch (const std::exception & e) {
    std::string error_msg = "Exception during multi-relay command: " + std::string(e.what());
    RCLCPP_ERROR(get_logger(), "%s", error_msg.c_str());
    return error_msg;
  }
}

void MvecNode::setSingleRelayCallback(
  const std::shared_ptr<mvec_msgs::srv::SetSingleRelay::Request> request,
  std::shared_ptr<mvec_msgs::srv::SetSingleRelay::Response> response)
{
  RCLCPP_INFO(
    get_logger(), "Setting single relay %d to state %s", request->relay.relay_id, request->relay.state ? "ON" : "OFF");

  auto result = set_single_relay(request->relay);
  if (result.has_value()) {
    // Error occurred
    response->success = false;
    response->message = result.value();
  } else {
    // Success
    response->success = true;
    response->message = "Single relay command successful";
  }
}

void MvecNode::setMultiRelayCallback(
  const std::shared_ptr<mvec_msgs::srv::SetMultiRelay::Request> request,
  std::shared_ptr<mvec_msgs::srv::SetMultiRelay::Response> response)
{
  RCLCPP_INFO(get_logger(), "Setting %zu relays", request->relays.size());

  auto result = set_multi_relay(request->relays);
  if (result.has_value()) {
    // Error occurred
    response->success = false;
    response->message = result.value();
  } else {
    // Success
    response->success = true;
    response->message = "Multi-relay command successful";
  }
}

void MvecNode::triggerPresetCallback(
  const std::shared_ptr<mvec_msgs::srv::TriggerPreset::Request> request,
  std::shared_ptr<mvec_msgs::srv::TriggerPreset::Response> response)
{
  RCLCPP_INFO(get_logger(), "Triggering preset: %s", request->name.c_str());

  // Find the preset by name
  auto it = std::find_if(presets_.begin(), presets_.end(), [&request](const mvec_msgs::msg::Preset & preset) {
    return preset.name == request->name;
  });

  if (it == presets_.end()) {
    response->success = false;
    response->message = "Preset not found: " + request->name;
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  auto result = set_multi_relay(it->relays);
  if (result.has_value()) {
    // Error occurred
    response->success = false;
    response->message = "Failed to trigger preset '" + request->name + "': " + result.value();
  } else {
    // Success
    response->success = true;
    response->message = "Preset '" + request->name + "' triggered successfully";
  }
}

diagnostic_msgs::msg::DiagnosticArray MvecNode::createDiagnosticsMessage()
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
        diagnostic_msgs::msg::KeyValue kv_status;
        kv_status.key = "status";
        kv_status.value = std::string(magic_enum::enum_name(status));
        relay_diag.values.push_back(kv_status);

        // Add current relay state if available
        if (current_relay_states_.has_value()) {
          for (const auto & relay : current_relay_states_.value().relay_states) {
            if (relay.relay_id == i) {
              diagnostic_msgs::msg::KeyValue kv_state;
              kv_state.key = "current_state";
              kv_state.value = relay.state ? "ON" : "OFF";
              relay_diag.values.push_back(kv_state);
              break;
            }
          }
        }

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

void MvecNode::parsePresetParams()
{
  presets_.clear();
  int preset_index = 0;
  while (true) {
    auto name = declare_parameter<std::string>("preset_" + std::to_string(preset_index) + "_name", "");
    auto relays = declare_parameter<std::vector<std::string>>("preset_" + std::to_string(preset_index) + "_relays", {});

    mvec_msgs::msg::Preset preset;

    if (!name.empty()) {
      preset.name = name;
    } else {
      RCLCPP_WARN(get_logger(), "empty param name given for preset_%d_name, aborting parse", preset_index);
      break;
    }

    for (auto & relay_string : relays) {
      relay_string.erase(std::remove_if(relay_string.begin(), relay_string.end(), ::isspace), relay_string.end());

      size_t colon_pos = relay_string.find(':');

      if (colon_pos == std::string::npos) {
        RCLCPP_WARN(
          get_logger(),
          "Invalid relay format in preset '%s': '%s' (expected 'id:state')",
          name.c_str(),
          relay_string.c_str());
        continue;
      }

      std::string id_str = relay_string.substr(0, colon_pos);
      std::string state_str = relay_string.substr(colon_pos + 1);
      mvec_msgs::msg::Relay relay;

      relay.relay_id = static_cast<uint8_t>(std::stoi(id_str));
      relay.state = (std::stoi(state_str) != 0);
      if (relay.relay_id >= polymath::sygnal::MvecHardware::MAX_NUMBER_RELAYS) {
        RCLCPP_WARN(
          get_logger(),
          "Invalid relay ID %d in preset '%s' (max: %d)",
          relay.relay_id,
          name.c_str(),
          polymath::sygnal::MvecHardware::MAX_NUMBER_RELAYS - 1);
        continue;
      }

      preset.relays.push_back(relay);

      RCLCPP_DEBUG(
        get_logger(),
        "Added relay %d (state: %s) to preset '%s'",
        relay.relay_id,
        relay.state ? "ON" : "OFF",
        name.c_str());
    }

    if (!preset.relays.empty()) {
      presets_.push_back(preset);
      RCLCPP_INFO(get_logger(), "Loaded preset '%s' with %zu relays", preset.name.c_str(), preset.relays.size());
    } else {
      RCLCPP_WARN(get_logger(), "Preset '%s' has no valid relays, skipping", name.c_str());
    }
    ++preset_index;
  }

  RCLCPP_INFO(get_logger(), "Loaded %zu presets total", presets_.size());
}

}  // namespace polymath::mvec
