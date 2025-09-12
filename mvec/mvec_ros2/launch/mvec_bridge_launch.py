# Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.conditions import IfCondition
from lifecycle_msgs.msg import Transition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Default config file path using find-pkg-share
    default_config_file = PathJoinSubstitution(
        [FindPackageShare("mvec_ros2"), "config", "relay_presets.yaml"]
    )

    # Define launch arguments
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config_file,
        description="Path to the MVEC configuration file with relay presets",
    )

    can_interface_arg = DeclareLaunchArgument(
        "can_interface",
        default_value="can0",
        description="CAN interface to use for MVEC communication",
    )

    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate",
        default_value="3.0",
        description="Rate in Hz to publish MVEC status and diagnostics",
    )

    timeout_ms_arg = DeclareLaunchArgument(
        "timeout_ms",
        default_value="500",
        description="Timeout in milliseconds for MVEC communication",
    )

    auto_configure_arg = DeclareLaunchArgument(
        "auto_configure",
        default_value="true",
        description="Automatically configure the lifecycle node",
    )

    auto_activate_arg = DeclareLaunchArgument(
        "auto_activate",
        default_value="true",
        description="Automatically activate the lifecycle node after configuration",
    )

    # Create the MVEC node (executable renamed from mvec_bridge to mvec_node)
    mvec_node = LifecycleNode(
        package="mvec_ros2",
        executable="mvec_node",
        name="mvec_node",
        namespace="",
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "can_interface": LaunchConfiguration("can_interface"),
                "publish_rate": LaunchConfiguration("publish_rate"),
                "timeout_ms": LaunchConfiguration("timeout_ms"),
            },
        ],
        output="screen",
    )

    # Event handler to automatically configure the node
    mvec_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=mvec_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(mvec_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration("auto_configure")),
    )

    # Event handler to automatically activate the node after configuration
    mvec_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=mvec_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(mvec_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration("auto_activate")),
    )

    return LaunchDescription(
        [
            config_file_arg,
            can_interface_arg,
            publish_rate_arg,
            timeout_ms_arg,
            auto_configure_arg,
            auto_activate_arg,
            mvec_node,
            mvec_configure_event_handler,
            mvec_activate_event_handler,
        ]
    )

