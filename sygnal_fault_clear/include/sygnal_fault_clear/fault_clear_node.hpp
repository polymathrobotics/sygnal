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

#ifndef FAULT_CLEAR_NODE_HPP_
#define FAULT_CLEAR_NODE_HPP_

#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>  // for close()

#include <cstdint>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <std_srvs/srv/trigger.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace polymath
{
namespace sygnal_interface
{

enum class MCMSystemState : uint8_t
{
  HumanControl = 0x00,
  MCMControl = 0x01,
  FailOp1 = 0xF1,
  FailOp2 = 0xF2,
  HumanOverride = 0xFD,
  FailHard = 0xFE,
  WarrantyReturn = 0xFF,
};

class FaultClearNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit FaultClearNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  // Lifecycle stages
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State &);

private:
  std_srvs::srv::Trigger_Response clearFaults();
  int openSocket(const std::string & interface);
  void generateCrc8Table();
  uint8_t calculateCrc8(uint8_t input_crc8_accum, const uint8_t * msg, uint8_t rx_len);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;
  void faultClearServiceCallback(
    const std_srvs::srv::Trigger_Request::SharedPtr trigger_request,
    std_srvs::srv::Trigger_Response::SharedPtr trigger_response);

  int socket_{-1};
  double timeout_s_;
  std::string interface_name_;
  uint8_t bus_address_;
  uint8_t crc8_table_[256];
  bool run_receive_ = false;
  std::thread can_receive_thread_;
  std::mutex can_thread_mutex_;
  struct can_frame seed_frame_;
  MCMSystemState mcm_state_0_;
  MCMSystemState mcm_state_1_;
  rclcpp::Time seed_frame_receive_time_;
  rclcpp::Time mcm_0_receive_time_;
  rclcpp::Time mcm_1_receive_time_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool mcm_0_received_;
  bool mcm_1_received_;
  bool seed_received_;

  bool send_clears_;
};

}  // namespace sygnal_interface
}  // namespace polymath

#endif  // FAULT_CLEAR_NODE_HPP_
