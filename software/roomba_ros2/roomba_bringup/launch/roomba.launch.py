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
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="create_driver",
            executable="create_driver",
            name="create_driver",
            parameters=[{
                "oi_mode_workaround": True,
            }]
        ),
        Node(
            package="ros2_bno055",
            executable="bno055",
            name="bno055"
        ),
    ])
