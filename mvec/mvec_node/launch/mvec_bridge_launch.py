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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.conditions import IfCondition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    # Define launch arguments
    can_interface_arg = DeclareLaunchArgument(
        "can_interface",
        default_value="can0",
        description="CAN interface to use for MVEC communication",
    )

    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate",
        default_value="10.0",
        description="Rate in Hz to publish MVEC status and diagnostics",
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

    # Create the MVEC bridge node
    mvec_bridge_node = LifecycleNode(
        package="mvec_node",
        executable="mvec_bridge",
        name="mvec_bridge",
        namespace="",
        parameters=[
            {
                "can_interface": LaunchConfiguration("can_interface"),
                "publish_rate": LaunchConfiguration("publish_rate"),
            }
        ],
        output="screen",
    )

    # Event handler to automatically configure the node
    mvec_bridge_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=mvec_bridge_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(mvec_bridge_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration("auto_configure")),
    )

    # Event handler to automatically activate the node after configuration
    mvec_bridge_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=mvec_bridge_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(mvec_bridge_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration("auto_activate")),
    )

    return LaunchDescription(
        [
            can_interface_arg,
            publish_rate_arg,
            auto_configure_arg,
            auto_activate_arg,
            mvec_bridge_node,
            mvec_bridge_configure_event_handler,
            mvec_bridge_activate_event_handler,
        ]
    )
