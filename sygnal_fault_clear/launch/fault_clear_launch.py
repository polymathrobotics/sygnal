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
from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get the launch directory

    print("Launching Sygnal Fault Clear....")

    log_level = LaunchConfiguration("log_level")
    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    interface_name = LaunchConfiguration("interface_name")
    interface_name_arg = DeclareLaunchArgument(
        "interface_name",
        default_value="can0",
        description="CAN interface name (i.e can0)",
    )

    namespace = LaunchConfiguration("namespace")
    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    sygnal_fault_clear_launch_node = Node(
        namespace=namespace,
        package="sygnal_fault_clear",
        executable="sygnal_fault_clear",
        name="sygnal_fault_clear",
        output="screen",
        parameters=[{"interface_name": interface_name}],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription(
        [
            log_level_arg,
            namespace_arg,
            interface_name_arg,
            sygnal_fault_clear_launch_node,
        ]
    )
