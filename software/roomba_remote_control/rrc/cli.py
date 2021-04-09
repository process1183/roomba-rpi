"""
Roomba Remote Control
https://github.com/process1183/roomba-rpi
Copyright (C) 2021  Josh Gadeken

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
import argparse
import logging
import os.path as osp

import rrc.roomba

DEFAULT_SERIAL = "/dev/ttyUSB0"
DEFAULT_SPEED_LIMIT = 0.70
DEFAULT_CONTROLLER_CONFIG = osp.abspath(osp.join(osp.dirname(osp.abspath(__file__)), "../", "controller.json"))



def speed_limiter(arg: str) -> float:
    """Convert `arg` to float and check that it is a valid PWM limit value."""
    value = float(arg)
    if not rrc.roomba.AxisToPWM.is_valid_pwm_limit(value):
        raise ValueError("Value must be greater than 0 and less than or equal to 1.")

    return value


def parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description="Roomba Remote Control")

    parser.add_argument(
        "-v", "--verbose", action="store_const", const=logging.INFO, dest="log_level",
        default=logging.WARNING, help="Enable verbose output."
    )

    parser.add_argument(
        "--debug", action="store_const", const=logging.DEBUG, dest="log_level",
        default=logging.WARNING, help="Enable debugging output."
    )

    parser.add_argument(
        "-s", "--serial-port", action="store", type=str, default=DEFAULT_SERIAL,
        help=f"Path of the serial port connected to the Roomba. Default is '{DEFAULT_SERIAL}'."
    )

    parser.add_argument(
        "-c", "--controller-config", action="store", type=str, default=DEFAULT_CONTROLLER_CONFIG,
        help=f"Path to the controller configuration JSON file. Default is '{DEFAULT_CONTROLLER_CONFIG}'"
    )

    parser.add_argument(
        "-d", "--controller-device", action="store", type=str, default=None,
        help="Path of the controller event device file. E.g. '/dev/input/event27'. If omitted and there is only one controller detected, it will be used. If omitted and there is more than one controller detected, then the user will be asked to select a controller to use."
    )

    parser.add_argument(
        "-l", "--limit-speed", action="store", type=speed_limiter, default=DEFAULT_SPEED_LIMIT,
        help=f"Normal speed limit percent for the drive wheels. Default is '{DEFAULT_SPEED_LIMIT}'. Full speed can be temporarily enabled with the 'full_speed' button on the controller."
    )

    parser.add_argument(
        "--unsafe", action="store_true",
        help="Use Roomba Open Interface 'Full Mode'. WARNING: This mode disables cliff, wheel-drop, and internal charger safety features!"
    )

    return parser.parse_args()
