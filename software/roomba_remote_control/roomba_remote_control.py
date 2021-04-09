#!/usr/bin/env python3
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
import signal
import threading

import evdev # https://pypi.org/project/evdev/
import pyroombaadapter # https://pypi.org/project/pyroombaadapter/

import rrc



def main(args: "argparse.Namespace") -> None:
    """Connect to controller and Roomba, then process controller input.

    Args:
        args: Parsed command line args from argparse.

    Returns:
        None
    """
    logging.basicConfig(
        format="%(levelname)s: %(message)s (%(filename)s:%(lineno)s)",
        level=args.log_level
    )

    try:
        controller_config = rrc.controller.load_config(args.controller_config)
    except Exception as err:
        logging.critical("Could not load config file! (%s)", err)
        return
    else:
        logging.info("Loaded controller config: %s", args.controller_config)

    # Combine command line args and controller config into one dict.
    # Convert args into a dict for consistency.
    options = {"cli_args": vars(args)}
    options.update(controller_config)

    logging.debug("options = %s", options)

    controller_path = rrc.controller.select_controller(options)
    if not controller_path:
        logging.critical("No controller found!")
        return

    controller = evdev.InputDevice(controller_path)
    logging.debug(
        "Controller opened: %s, %s, %s",
        controller.name,
        controller.path,
        controller.uniq
    )

    roomba = pyroombaadapter.PyRoombaAdapter(args.serial_port)

    if options["cli_args"]["unsafe"]:
        roomba.change_mode_to_full()
        logging.warning("Cliff, wheel-drop, and internal charger safety features have been disabled! Drive carefully!")

    # Set shutdown_event when CTRL+c is pressed
    shutdown_event = threading.Event()
    signal.signal(signal.SIGINT, lambda signum, frame: shutdown_event.set())

    try:
        rrc.controller.process_events(options, controller, roomba, shutdown_event)
    except OSError as err:
        logging.critical("Controller error! (%s)", err)
        shutdown_event.set()

    shutdown_event.wait()
    print("\nbye!")

    # Stop the Roomba's wheels and cleaning motors before exiting
    roomba.send_drive_pwm(0, 0)
    roomba.send_pwm_moters(0, 0, 0)
    logging.debug("Roomba drive and cleaning motors stopped.")

    controller.close()
    logging.debug(
        "Closed controller: %s, %s, %s",
        controller.name,
        controller.path,
        controller.uniq
    )


if __name__ == "__main__":
    main(rrc.cli.parse_args())
