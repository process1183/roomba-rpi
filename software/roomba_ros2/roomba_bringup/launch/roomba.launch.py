#!/usr/bin/env python3
"""
Roomba base launch file
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
    roomba_config = os.path.join(
            get_package_share_directory("roomba_bringup"),
            "config",
            "roomba.yaml"
        )
    bno055_config = os.path.join(
            get_package_share_directory("roomba_bringup"),
            "config",
            "bno055.yaml"
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            "roomba_config_file",
            default_value=roomba_config,
            description="Roomba configuration file"
        ),
        DeclareLaunchArgument(
            "bno055_config_file",
            default_value=bno055_config,
            description="BNO055 configuration file"
        ),
        Node(
            package="create_driver",
            executable="create_driver",
            name="create_driver",
            parameters=[LaunchConfiguration("roomba_config_file")]
        ),
        Node(
            package="ros2_bno055",
            executable="bno055",
            name="bno055",
            parameters=[LaunchConfiguration("bno055_config_file")]
        ),
    ])
