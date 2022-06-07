#!/usr/bin/env python3
"""
Roomba joystick teleoperation launch file
https://github.com/process1183/roomba-rpi
Copyright (C) 2022  Josh Gadeken

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
            get_package_share_directory("roomba_bringup"),
            "config",
            "ds4_teleop.yaml"
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=config,
            description="Controller configuration file"
        ),
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            parameters=[LaunchConfiguration("config_file")]
        ),
        Node(
            package="joy_teleop",
            executable="joy_teleop",
            name="joy_teleop",
            parameters=[LaunchConfiguration("config_file")]
        ),
    ])
