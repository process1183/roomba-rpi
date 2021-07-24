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
import json
import logging
import time

import evdev # https://pypi.org/project/evdev/

import rrc.roomba
import rrc.util



def load_config(config_path: str) -> dict:
    """Load controller configuration json file and decode evdev ecode strings.

    Args:
        config_path: Filesystem path of the controller configuration json file.

    Returns:
        The controller configuration dict
    """
    with open(config_path, 'r') as infd:
        raw_config = json.load(infd)

    return rrc.util.ecode_decode(raw_config)


def select_controller(options: dict) -> str:
    """Select the event device of the controller to use.

    If the -d/--controller-device option is given, its arg will returned. If the
    -d/--controller-device option is not given and there is only one controller
    detected, its event device path will be returned. If the -d/--controller-device
    option and there is more than one controller detected, then the user will be
    asked to select a controller to use.

    This function was inspired by evdev's `select_devices()` in evtest.py

    Args:
        options: The dict containing command line options and controller config

    Returns:
        The path of the controller event device
    """
    if options["cli_args"]["controller_device"]:
        return options["cli_args"]["controller_device"]

    dev_paths = evdev.list_devices()

    if not dev_paths:
        return None

    # If there's only one controller detected, return its event device path.
    if len(dev_paths) == 1:
        return dev_paths[0]

    # More than one controller detected- ask the user which one to use
    fmt_str = "{:<3} {:<18} {:<17} {:<17} {:<20}"
    print("Available controllers:")
    print(fmt_str.format("ID", "Device", "Uniq", "Phys", "Name"))

    for num, dev_path in enumerate(dev_paths):
        try:
            dev = evdev.InputDevice(dev_path)
        except Exception as err:
            logging.warning("Error trying to open '%s'... skipping.", dev_path)
            logging.debug(err)
            continue

        print(fmt_str.format(num, dev.path, dev.uniq, dev.phys, dev.name))

    while True:
        selection = input(f"Select device [0-{len(dev_paths)-1}]: ")
        try:
            dev_num = int(selection)
        except ValueError:
            dev_num = -1

        if 0 <= dev_num < len(dev_paths):
            return dev_paths[dev_num]

        print("Invalid selection!")


def process_events(options: dict,
                   controller: evdev.InputDevice,
                   roomba: "pyroombaadapter.PyRoombaAdapter",
                   shutdown_event: "threading.Event") -> None:
    """Translate controller events to Roomba commands.

    Args:
        options: The dict containing command line options and controller config
        controller: An instance of evdev's InputDevice
        roomba: An instance of PyRoombaAdapter
        shutdown_event: Event that signals shutdown. This function will block until
                        this Event is set.

    Returns:
        None
    """
    wheel_full_speed = False
    left_wheel_axis = rrc.roomba.AxisToPWM(
        controller.absinfo(options["control_map"]["wheel_left"]).min,
        controller.absinfo(options["control_map"]["wheel_left"]).max,
        rrc.roomba.WHEEL_PWM_MIN,
        rrc.roomba.WHEEL_PWM_MAX,
        invert_axis=(options["control_map"]["wheel_left"] in options["invert_axis_list"]),
        center_deadband=options["drive_axis_deadband"],
        pwm_limit=options["cli_args"]["limit_speed"]
    )
    right_wheel_axis = rrc.roomba.AxisToPWM(
        controller.absinfo(options["control_map"]["wheel_right"]).min,
        controller.absinfo(options["control_map"]["wheel_right"]).max,
        rrc.roomba.WHEEL_PWM_MIN,
        rrc.roomba.WHEEL_PWM_MAX,
        invert_axis=(options["control_map"]["wheel_right"] in options["invert_axis_list"]),
        center_deadband=options["drive_axis_deadband"],
        pwm_limit=options["cli_args"]["limit_speed"]
    )

    cleaning_motors = rrc.roomba.CleaningMotors(
        roomba,
        controller.absinfo(options["control_map"]["cleaning_motor_speed"]).min,
        controller.absinfo(options["control_map"]["cleaning_motor_speed"]).max
    )

    while not shutdown_event.is_set():
        ev = controller.read_one()
        if not ev:
            time.sleep(0.01)
            continue

        # `drive_update` is used to determine if there's a change to the drive wheel speed
        # With my DS4, there's a large amount of jitter on the analog stick centers
        # that causes a lot of extra events, so the Roomba's wheel speeds will only
        # be updated if there is a change outside of the deadband.
        drive_update = False

        if ev.code == options["control_map"]["wheel_left"]:
            drive_update = left_wheel_axis.update(ev.value)

        elif ev.code == options["control_map"]["wheel_right"]:
            drive_update = right_wheel_axis.update(ev.value)

        elif ev.code == options["control_map"]["wheel_full_speed"]:
            logging.debug(evdev.categorize(ev))
            wheel_full_speed = bool(ev.value)
            logging.info(f"Full speed {'enabled' if wheel_full_speed else 'disabled'}.")
            drive_update = True

        elif ev.code == options["control_map"]["cleaning_motor_speed"]:
            logging.debug("%s, %s", evdev.categorize(ev), ev.value)
            _ = cleaning_motors.store_axis(ev.value)

        elif ev.code == options["control_map"]["cleaning_motor_set_toggle"] and ev.value == 1:
            logging.debug(evdev.categorize(ev))
            cleaning_motors.apply_stored_speed()

        elif ev.code == options["control_map"]["auto_clean"] and ev.value == 1:
            logging.debug(evdev.categorize(ev))
            logging.info("Auto cleaning mode...")
            roomba.start_cleaning()

        elif ev.code == options["control_map"]["seek_dock"] and ev.value == 1:
            logging.debug(evdev.categorize(ev))
            logging.info("Seek dock...")
            roomba.start_seek_dock()

        elif ev.code == options["control_map"]["restart_oi"] and ev.value == 1:
            logging.debug(evdev.categorize(ev))
            logging.info("Restarting ROI...")

            # Roomba automatically turns off cleaning motors when ROI is restarted,
            # so update the cleaning_motors.running state to reflect this.
            cleaning_motors.running = False

            roomba.change_mode_to_passive()
            time.sleep(1)
            if options["cli_args"]["unsafe"]:
                roomba.change_mode_to_full()
            else:
                roomba.change_mode_to_safe()

            logging.info("ROI restarted.")

        else:
            continue

        if drive_update:
            left_wheel_pwm = left_wheel_axis.to_pwm(wheel_full_speed)
            right_wheel_pwm = right_wheel_axis.to_pwm(wheel_full_speed)
            logging.debug("left_wheel_pwm=%s, right_wheel_pwm=%s", left_wheel_pwm, right_wheel_pwm)
            roomba.send_drive_pwm(right_wheel_pwm, left_wheel_pwm)
