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

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <memory>

#if __has_include(<catch2/catch_all.hpp>)
  #include <catch2/catch_all.hpp>
#elif __has_include(<catch2/catch.hpp>)
  #include <catch2/catch.hpp>
#else
  #error "Catch2 headers not found. Please install Catch2 (v2 or v3)."
#endif

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "sygnal_can_interface_ros2/sygnal_can_interface_node.hpp"
#include "sygnal_can_msgs/msg/control_command.hpp"
#include "sygnal_dbc/mcm_control.h"

namespace
{
constexpr const char * VCAN_INTERFACE = "vcan0";

/// Raw CAN socket that only receives control command frames
class ControlFrameListener
{
public:
  ControlFrameListener()
  {
    fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    REQUIRE(fd_ >= 0);

    can_filter filter{};
    filter.can_id = MCM_CONTROL_CONTROL_COMMAND_FRAME_ID;
    filter.can_mask = CAN_SFF_MASK;
    setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));

    timeval timeout{0, 10000};
    setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    ifreq ifr{};
    std::strncpy(ifr.ifr_name, VCAN_INTERFACE, IFNAMSIZ - 1);
    REQUIRE(ioctl(fd_, SIOCGIFINDEX, &ifr) == 0);

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    REQUIRE(bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == 0);
  }

  ~ControlFrameListener()
  {
    close(fd_);
  }

  /// @brief Read every pending frame, returns how many arrived
  int drain()
  {
    int count = 0;
    can_frame frame{};
    while (read(fd_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame))) {
      ++count;
    }
    return count;
  }

private:
  int fd_{-1};
};

/// @brief Spin for the given duration and return the number of control frames seen on the bus
int spinAndCount(rclcpp::Executor & executor, ControlFrameListener & listener, std::chrono::milliseconds duration)
{
  int count = 0;
  auto end = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < end) {
    executor.spin_some(std::chrono::milliseconds(10));
    count += listener.drain();
  }
  return count;
}

sygnal_can_msgs::msg::ControlCommand makeCommand(double value)
{
  sygnal_can_msgs::msg::ControlCommand msg;
  msg.bus_id = 1;
  msg.interface_id = 0;
  msg.subsystem_id = 0;
  msg.value = value;
  return msg;
}

/// @brief Initializes rclcpp for the lifetime of a test case
struct RclcppContext
{
  RclcppContext()
  {
    rclcpp::init(0, nullptr);
  }

  ~RclcppContext()
  {
    rclcpp::shutdown();
  }
};

/// @brief Node under test, a publisher on its control command topic and a listener on the bus
class LifecycleFixture
{
public:
  LifecycleFixture()
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"can_interface", VCAN_INTERFACE}});
    node = std::make_shared<polymath::sygnal::SygnalCanInterfaceNode>(options);
    pub_node = std::make_shared<rclcpp::Node>("control_command_test_publisher");
    publisher = pub_node->create_publisher<sygnal_can_msgs::msg::ControlCommand>(
      "/sygnal_can_interface_node/control_command", rclcpp::QoS(10));
    executor.add_node(node->get_node_base_interface());
    executor.add_node(pub_node);
  }

  ~LifecycleFixture()
  {
    executor.remove_node(pub_node);
    executor.remove_node(node->get_node_base_interface());
  }

  size_t waitForSubscribers(size_t expected)
  {
    auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (publisher->get_subscription_count() != expected && std::chrono::steady_clock::now() < end) {
      executor.spin_some(std::chrono::milliseconds(10));
    }
    return publisher->get_subscription_count();
  }

  /// @brief Configure and activate, then check that one command reaches the bus
  void activateAndSendOne()
  {
    REQUIRE(node->configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    REQUIRE(node->activate().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    REQUIRE(waitForSubscribers(1) == 1);
    listener.drain();
    publisher->publish(makeCommand(0.11));
    REQUIRE(spinAndCount(executor, listener, std::chrono::milliseconds(1000)) == 1);
  }

  RclcppContext context;
  std::shared_ptr<polymath::sygnal::SygnalCanInterfaceNode> node;
  std::shared_ptr<rclcpp::Node> pub_node;
  rclcpp::Publisher<sygnal_can_msgs::msg::ControlCommand>::SharedPtr publisher;
  rclcpp::executors::SingleThreadedExecutor executor;
  ControlFrameListener listener;
};
}  // namespace

TEST_CASE("Inactive node does not send control commands", "[sygnal_can_interface_node]")
{
  LifecycleFixture fixture;
  fixture.activateAndSendOne();

  REQUIRE(fixture.node->deactivate().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  fixture.waitForSubscribers(0);
  for (double value : {0.22, 0.23, 0.24}) {
    fixture.publisher->publish(makeCommand(value));
  }
  REQUIRE(spinAndCount(fixture.executor, fixture.listener, std::chrono::milliseconds(1000)) == 0);
}

TEST_CASE("Control command after cleanup is ignored", "[sygnal_can_interface_node]")
{
  LifecycleFixture fixture;
  fixture.activateAndSendOne();

  REQUIRE(fixture.node->deactivate().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  REQUIRE(fixture.node->cleanup().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  fixture.publisher->publish(makeCommand(0.33));
  REQUIRE(spinAndCount(fixture.executor, fixture.listener, std::chrono::milliseconds(1000)) == 0);
}
