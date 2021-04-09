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
import logging

import rrc.util

# These values are from pages 14 and 15 of the iRobot Create 2 Open Interface Spec
# https://www.irobotweb.com/~/media/MainSite/PDFs/About/STEM/Create/iRobot_Roomba_600_Open_Interface_Spec.pdf
WHEEL_PWM_MIN = -255
WHEEL_PWM_MAX = 255
BRUSH_PWM_MIN = -127
BRUSH_PWM_MAX = 127
VACUUM_PWM_MIN = 0
VACUUM_PWM_MAX = 127



class AxisToPWM:
    """Class for converting controller axis values to Roomba PWM values."""
    def __init__(self,
                 axis_min: int,
                 axis_max: int,
                 pwm_min: int,
                 pwm_max: int,
                 invert_axis: bool = False,
                 center_deadband: int = 0,
                 pwm_limit: float = 1.0) -> None:
        """
        Args:
            axis_min: Minimum value for the given axis.
            axis_max: Maximum value for the given axis.
            pwm_min: Minimum value of the PWM output range.
            pwm_max: Maximum value of the PWM output range.
            invert_axis: Flip the axis range? E.g. if an analog stick's value when fully
                         pushed up is 0, and its value when fully pushed down is 255, then
                         the PWM output would be -255 (backwards) for up and 255 (forwards)
                         for down.
            center_deadband: Deadband range to apply around the center value of the axis. A
                             value of 0 will disable the deadband.
            pwm_limit: Default limit percent (expressed as a decimal) for the output PWM
                       range. This value must be greater than 0 and less than or equal to 1.

        Returns:
            None
        """
        self.axis_min = axis_min
        self.axis_max = axis_max
        self.pwm_min = pwm_min
        self.pwm_max = pwm_max
        self.invert_axis = invert_axis
        self.deadband = center_deadband

        if not self.is_valid_pwm_limit(pwm_limit):
            raise ValueError("PWM limit must be greater than 0 and less than or equal to 1!")
        self.pwm_limit = pwm_limit

        self.axis_center = (axis_min + axis_max) / 2.0
        self.axis_value = self.axis_center

    def deadband_filter(self, axis_value: int) -> float:
        """Change axis values within the deadband to the axis center value. Axis values
        outside the deadband will be unchanged.

        Args:
            axis_value: The raw axis value.

        Returns:
            The new axis value.
        """
        deadband_lower_bound = self.axis_center - (self.deadband / 2)
        deadband_upper_bound = self.axis_center + (self.deadband / 2)

        value = float(axis_value)
        if deadband_lower_bound <= axis_value <= deadband_upper_bound:
            value = self.axis_center

        return value

    def update(self, value: int) -> bool:
        """Apply a deadband filter (if enabled) to the axis and store the value.

        Args:
            value: The axis value

        Returns:
            True if the new axis value differs from the last one seen, False otherwise.
        """
        if self.deadband != 0:
            value = self.deadband_filter(value)

        changed = self.axis_value != value
        self.axis_value = value

        return changed

    def to_pwm(self, full_speed: bool = True) -> int:
        """Convert the stored axis value to its corresponding PWM value.

        Args:
            full_speed: If True, the PWM limit is ignored. If False, then the output PWM
                        range is reduced by the amount given in `pwm_limit`.

        Returns:
            The PWM value
        """
        if full_speed:
            pwm_min = self.pwm_min
            pwm_max = self.pwm_max
        else:
            pwm_min = round(self.pwm_min * self.pwm_limit, 0)
            pwm_max = round(self.pwm_max * self.pwm_limit, 0)

        if self.invert_axis:
            axis_value = (self.axis_min + self.axis_max) - self.axis_value
        else:
            axis_value = self.axis_value

        pwm_value = rrc.util.clamped_map(axis_value, self.axis_min, self.axis_max, pwm_min, pwm_max)

        return round(pwm_value)

    @staticmethod
    def is_valid_pwm_limit(value: float) -> bool:
        """Check if a value is greater than 0 and less than or equal to 1.

        Args:
            value: The number to check.

        Returns:
            True if the number is within range, False otherwise.
        """
        return 0.0 < value <= 1.0


class CleaningMotors:
    """Wrapper around the Roomba's cleaning motors that enables toggling while retaining
    the cleaning motors' last speed."""
    def __init__(self,
                 roomba: "pyroombaadapter.PyRoombaAdapter",
                 axis_min: int,
                 axis_max: int) -> None:
        """
        Args:
            roomba: An instance of PyRoombaAdapter.
            axis_min: Minimum value for the given axis.
            axis_max: Maximum value for the given axis.

        Returns:
            None
        """
        self._roomba = roomba
        self.axis = AxisToPWM(axis_min, axis_max, 0, VACUUM_PWM_MAX)
        _ = self.axis.update(axis_min)
        self.pwm = VACUUM_PWM_MAX
        self.running = False

    def activate(self) -> None:
        """Turn on the cleaning motors.

        Args:
            None

        Returns:
            None
        """
        logging.info("Activating cleaning motors (PWM: %s)...", self.pwm)
        self._roomba.send_pwm_moters(self.pwm, self.pwm, self.pwm)
        self.running = True

    def deactivate(self) -> None:
        """Turn off the cleaning motors.

        Args:
            None

        Returns:
            None
        """
        logging.info("Deactivating cleaning motors...")
        self._roomba.send_pwm_moters(0, 0, 0)
        self.running = False

    def toggle(self) -> None:
        """Toggle the cleaning motors on or off, depending on their current state.

        Args:
            None

        Returns:
            None
        """
        if self.running:
            self.deactivate()
        else:
            self.activate()

    def store_axis(self, value: int) -> bool:
        """Update the stored axis value. Note: this method does not change the cleaning
        motor speed when called.

        Args:
            value: The axis value to store.

        Returns:
            True if the new axis value differs from the last one seen, False otherwise.
        """
        return self.axis.update(value)

    def apply_stored_speed(self) -> None:
        """Toggle the cleaning motor state or set a new cleaning motor speed.

        If the stored axis value is equal to `axis_min`, then toggle the cleaning motors
        on or off. If the stored axis value is not `axis_min`, then change the cleaning
        motor speeds using the stored axis value. This was designed for a controller
        trigger that has a minimum value, e.g. 0, when unpressed and a maximum value, e.g.
        255, when fully pressed. This way, the cleaning motor speed is only changed if the
        user is pressing the trigger and presses the cleaning motor set/toggle button.

        Args:
            None

        Returns:
            None
        """
        if self.axis.axis_value == self.axis.axis_min:
            self.toggle()
        else:
            self.pwm = self.axis.to_pwm()
            self.activate()
