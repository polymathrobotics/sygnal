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

#include "sygnal_fault_clear/fault_clear_node.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <poll.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <iostream>
#include <string>

#include "rclcpp/time.hpp"

using std::chrono_literals::operator""s;

namespace polymath
{
namespace sygnal_interface
{
FaultClearNode::FaultClearNode(const rclcpp::NodeOptions & options)
: LifecycleNode("fault_clear_node", "", options)
, seed_frame_receive_time_(rclcpp::Time(static_cast<int64_t>(0), RCL_ROS_TIME))
, mcm_0_receive_time_(rclcpp::Time(static_cast<int64_t>(0), RCL_ROS_TIME))
, mcm_1_receive_time_(rclcpp::Time(static_cast<int64_t>(0), RCL_ROS_TIME))
{
  this->declare_parameter<std::string>("interface_name", "can0");
  this->declare_parameter<int>("bus_address", 1);  // Assuming bus address is in range 0-255
  this->declare_parameter<double>("timeout_s", 5);  // Assuming bus address is in range 0-255
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn FaultClearNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring...");
  interface_name_ = this->get_parameter("interface_name").as_string();
  bus_address_ = static_cast<uint8_t>(this->get_parameter("bus_address").as_int());
  timeout_s_ = this->get_parameter("timeout_s").as_double();

  generateCrc8Table();

  // Start subscription to CAN data
  run_receive_ = true;
  socket_ = openSocket(interface_name_);
  if (socket_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open CAN socket");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  can_receive_thread_ = std::thread([this]() {
    struct pollfd fds[1];
    struct can_frame frame;

    fds[0].fd = this->socket_;
    fds[0].events = POLLIN;

    int timeout_msecs = 10;

    while (this->run_receive_ && rclcpp::ok()) {
      int ret = poll(fds, 1, timeout_msecs);
      if (ret <= 0) {
        continue;
      }

      int nbytes = recv(this->socket_, &frame, sizeof(struct can_frame), MSG_DONTWAIT);
      if (nbytes < 0) {
        // Is this MISRA compliant?
        continue;
      }

      if (frame.can_id == 0x170) {
        if ((frame.data[0] & 0x7F) == this->bus_address_) {
          if (frame.data[0] & 0x80) {
            RCLCPP_DEBUG(this->get_logger(), "MCM 0 Received");
            this->mcm_0_receive_time_ = this->get_clock()->now();
            this->mcm_0_received_ = true;
            this->mcm_state_0_ = MCMSystemState(frame.data[2]);
          } else {
            RCLCPP_DEBUG(this->get_logger(), "MCM 1 Received");
            this->mcm_1_receive_time_ = this->get_clock()->now();
            this->mcm_1_received_ = true;
            this->mcm_state_1_ = MCMSystemState(frame.data[2]);
          }
        }
      }

      if (frame.can_id == 0x70) {
        if ((frame.data[0] & 0x7F) == this->bus_address_) {
          RCLCPP_DEBUG(this->get_logger(), "Seed Received");
          this->seed_frame_receive_time_ = this->get_clock()->now();
          this->seed_received_ = true;
          this->seed_frame_ = frame;

          if (this->send_clears_) {
            // Send fault clear request
            uint32_t seed = this->seed_frame_.data[2];
            seed |= static_cast<uint32_t>(this->seed_frame_.data[3]) << 8;
            seed |= static_cast<uint32_t>(this->seed_frame_.data[4]) << 16;
            seed |= static_cast<uint32_t>(this->seed_frame_.data[5]) << 24;
            uint32_t key = seed ^ 0xA5A5A5A5;
            struct can_frame keyMessage;
            keyMessage.can_id = 0x7F;
            keyMessage.can_dlc = 8;
            keyMessage.data[0] = this->seed_frame_.data[0];
            keyMessage.data[1] = this->seed_frame_.data[1];
            *reinterpret_cast<uint32_t *>(&(keyMessage.data[2])) = key;
            keyMessage.data[6] = 0;
            keyMessage.data[7] = calculateCrc8(0, keyMessage.data, 7);
            send(this->socket_, &keyMessage, sizeof(struct can_frame), 0);
          }
        }
      }
    }
  });

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn FaultClearNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating...");

  trigger_service_ = this->create_service<std_srvs::srv::Trigger>(
    "clear_sygnal_faults",
    std::bind(&FaultClearNode::faultClearServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn FaultClearNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating...");

  timer_.reset();
  trigger_service_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn FaultClearNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up...");

  if (socket_ != -1) {
    close(socket_);
    socket_ = -1;
  }

  run_receive_ = false;
  can_receive_thread_.join();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn FaultClearNode::on_shutdown(
  const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down...");
  return on_cleanup(state);  // Reuse the cleanup logic for shutdown
}

int FaultClearNode::openSocket(const std::string & interface)
{
  struct sockaddr_can addr;
  int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (s < 0) {
    perror("Socket");
    return -1;
  }

  auto if_index = if_nametoindex(interface.c_str());

  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = if_index;
  if (bind(s, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    perror("Bind");
    close(s);
    return -1;
  }

  return s;
}

void FaultClearNode::generateCrc8Table()
{
  uint8_t crc;
  for (uint16_t i = 0; i < 256; ++i) {
    crc = i;
    for (uint8_t j = 0; j < 8; ++j) {
      crc = (crc << 1) ^ ((crc & 0x80) ? 0x07 : 0);
    }
    crc8_table_[i] = crc;
  }
}

uint8_t FaultClearNode::calculateCrc8(uint8_t input_crc8_accum, const uint8_t * msg, uint8_t rx_len)
{
  uint8_t crc = input_crc8_accum;
  for (uint8_t i = 0; i < rx_len; ++i) {
    crc = crc8_table_[crc ^ msg[i]];
  }
  return crc;
}

void FaultClearNode::faultClearServiceCallback(
  const std_srvs::srv::Trigger_Request::SharedPtr, std_srvs::srv::Trigger_Response::SharedPtr trigger_response)
{
  RCLCPP_INFO(get_logger(), "Attempting fault clear...");
  auto fault_clear_response = clearFaults();
  send_clears_ = false;
  RCLCPP_INFO(get_logger(), "Fault clear result: %s", std_srvs::srv::to_yaml(fault_clear_response).c_str());
  *trigger_response = fault_clear_response;
}

std_srvs::srv::Trigger_Response FaultClearNode::clearFaults()
{
  std_srvs::srv::Trigger_Response fault_clear_response;
  auto timeout_duration = rclcpp::Duration(timeout_s_, 0);

  // Make sure we got state and seeds recently
  auto now = this->get_clock()->now();
  auto time_since_mcm_0 = now - mcm_0_receive_time_;
  auto time_since_mcm_1 = now - mcm_1_receive_time_;

  // We need to have an updated state in order to do this job
  if ((time_since_mcm_0 > timeout_duration) || (time_since_mcm_1 > timeout_duration)) {
    RCLCPP_DEBUG(
      get_logger(),
      "Time since MCM 0: %ld, Time since MCM 1 %ld",
      time_since_mcm_0.nanoseconds(),
      time_since_mcm_1.nanoseconds());

    fault_clear_response.success = false;
    fault_clear_response.message = "MCM Messages not received";
    return fault_clear_response;
  }

  if (((uint8_t)mcm_state_0_ < uint8_t(2)) || ((uint8_t)mcm_state_1_ < uint8_t(2))) {
    // Device not faulted
    // Not handling state mismatch currently
    fault_clear_response.success = true;
    fault_clear_response.message = "Device not faulted";
    return fault_clear_response;
  }

  if (mcm_state_0_ != mcm_state_1_) {
    // Failed, state mismatch
    fault_clear_response.success = false;
    fault_clear_response.message = "MCM State mismatch";
    return fault_clear_response;
  }

  auto time_since_seed = this->get_clock()->now() - seed_frame_receive_time_;

  if (time_since_seed > timeout_duration) {
    // No seed received in time
    fault_clear_response.success = false;
    fault_clear_response.message = "No seed received in time";
    return fault_clear_response;
  }

  // Tell the can thread to start sending clears
  send_clears_ = true;

  now = this->get_clock()->now();
  // Wait for faults to clear
  while ((mcm_state_0_ != MCMSystemState::HumanControl) || (mcm_state_1_ != MCMSystemState::HumanControl)) {
    auto time_waiting = this->get_clock()->now() - now;
    if (time_waiting > timeout_duration) {
      // We did not clear in time
      fault_clear_response.success = false;
      fault_clear_response.message = "MCM did not clear in time";
      send_clears_ = false;
      return fault_clear_response;
    }
  }

  send_clears_ = false;
  fault_clear_response.success = true;
  fault_clear_response.message = "MCM cleared successfully";
  return fault_clear_response;
}

}  // namespace sygnal_interface
}  // namespace polymath

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(polymath::sygnal_interface::FaultClearNode)
