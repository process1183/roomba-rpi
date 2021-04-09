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
import evdev.ecodes # https://pypi.org/project/evdev/


def clamped_map(x: float,
                in_min: float,
                in_max: float,
                out_min: float,
                out_max: float) -> float:
    """Re-maps a number from one range to another.

    This is a modified copy of Arduino's map() function:
    https://www.arduino.cc/reference/en/language/functions/math/map/

    This version clamps (constrains) the output to `out_min` or `out_max`
    if `x` is less than `in_min` or greater than `in_max`.

    Args:
        x: The number to map.
        in_min: The lower bound of the value's current range.
        in_max: The upper bound of the value's current range.
        out_min: The lower bound of the value's target range.
        out_max: The upper bound of the value's target range.

    Returns:
        The mapped value.
    """
    if x < in_min:
        return out_min

    if x > in_max:
        return out_max

    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def ecode_decode(controller_config: dict) -> dict:
    """Convert evdev ecodes in the controller config from strings to ints.

    Args:
        controller_config: The raw controller configuration dict

    Returns:
        The decoded controller configuration dict
    """
    control_map = controller_config.pop("control_map")
    invert_axis = controller_config.pop("invert_axis_list")

    controller_config["control_map"] = {k: getattr(evdev.ecodes, v) for k, v in control_map.items()}
    controller_config["invert_axis_list"] = [getattr(evdev.ecodes, i) for i in invert_axis]

    return controller_config
