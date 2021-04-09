# Roomba Remote Control #

This program allows the Roomba to be teleoperated using a controller connected via Bluetooth. It was designed to use the DualShock 4, but other controllers should work, however you may need to change the button map in `controller.json`.

(Located in: `roomba-rpi/software/roomba_remote_control/`)

## Usage ##

* `cd ~/roomba-rpi/software/roomba_remote_control/` (Assuming this repo was cloned to your home directory)
* Install the required Python 3 dependencies: `sudo pip3 install -r requirements.txt`
* Connect Bluetooth controller (see below instructions)
* Run `./roomba_remote_control.py`


## `roomba_remote_control.py` Help Message ##

```
usage: roomba_remote_control.py [-h] [-v] [--debug] [-s SERIAL_PORT]
                                [-c CONTROLLER_CONFIG] [-d CONTROLLER_DEVICE]
                                [-l LIMIT_SPEED] [--unsafe]

Roomba Remote Control

optional arguments:
  -h, --help            show this help message and exit
  -v, --verbose         Enable verbose output.
  --debug               Enable debugging output.
  -s SERIAL_PORT, --serial-port SERIAL_PORT
                        Path of the serial port connected to the Roomba.
                        Default is '/dev/ttyUSB0'.
  -c CONTROLLER_CONFIG, --controller-config CONTROLLER_CONFIG
                        Path to the controller configuration JSON file.
                        Default is 'controller.json'
  -d CONTROLLER_DEVICE, --controller-device CONTROLLER_DEVICE
                        Path of the controller event device file. E.g.
                        '/dev/input/event27'. If omitted and there is only one
                        controller detected, it will be used. If omitted and
                        there is more than one controller detected, then the
                        user will be asked to select a controller to use.
  -l LIMIT_SPEED, --limit-speed LIMIT_SPEED
                        Normal speed limit percent for the drive wheels.
                        Default is '0.7'. Full speed can be temporarily
                        enabled with the 'full_speed' button on the
                        controller.
  --unsafe              Use Roomba Open Interface 'Full Mode'. WARNING: This
                        mode disables cliff, wheel-drop, and internal charger
                        safety features!
```


## Bluetooth Controller Connection ##

* Ensure the Bluetooth service is running with `sudo systemctl status bluetooth.service`
* If the Bluetooth service is not running, enable and start it with `sudo systemctl enable bluetooth.service && sudo systemctl start bluetooth.service`
* Run `bluetoothctl`
* Power on Bluetooth: `power on`
* Start scanning: `scan on`
* Wait until your controller is listed, E.g. `[NEW] Device 01:23:45:67:89:AB 01-23-45-67-89-AB`
* Once your controller is found, stop scanning: `scan off`
* (Replace `01:23:45:67:89:AB` with the MAC address of your controller in the following instructions).
* Pair with your controller: `pair 01:23:45:67:89:AB`
* Trust your controller: `trust 01:23:45:67:89:AB`
* Connect to your controller (if necessary): `connect 01:23:45:67:89:AB`


## Controller Layout Configuration ##

The controller button and axis map configuration is stored in the `controller.json` file. The values of the members must be evdev event code strings. You can find out which event codes correspond to the buttons and axis on your controller by running: `python3 -m evdev.evtest`.

The default `controller.json` file:

```json
{
    "control_map": {
        "auto_clean": "BTN_EAST",
        "cleaning_motor_set_toggle": "BTN_SOUTH",
        "cleaning_motor_speed": "ABS_RZ",
        "restart_oi": "BTN_WEST",
        "seek_dock": "BTN_NORTH",
        "wheel_full_speed": "BTN_TL",
        "wheel_left": "ABS_Y",
        "wheel_right": "ABS_RY"
    },
    "drive_axis_deadband": 20,
    "invert_axis_list": [
        "ABS_Y",
        "ABS_RY"
    ]
}
```

| Object Name | Description |
| ----------- | ----------- |
| `auto_clean` | Button to start/stop the Roomba's normal automatic cleaning routine. |
| `cleaning_motor_set_toggle` | Button to set or toggle the Roomba's cleaning motors (brushes and vacuum). If the `cleaning_motor_speed` axis value is 0, then the cleaning motors are toggled on or off, while retaining the last speed they were set at. If the `cleaning_motor_speed` is not 0, then the cleaning motors are set to the speed determined by the axis value. |
| `cleaning_motor_speed` | Axis used to set the cleaning motor speed. Note: This was designed around the left and right triggers that have a value of 0 when fully unpressed, and their maximum value when fully pressed. |
| `restart_oi` | Button to restart the iRobot Roomba Open Interface. Useful if a safety condition is tripped (such as a wheel drop) during driving. |
| `seek_dock` | Button to tell the Roomba to return to its dock. |
| `wheel_full_speed` | Button that enables full driving speed while pressed. |
| `wheel_left` | Axis that controls the left wheel. |
| `wheel_right` | Axis that controls the right wheel. |
| `drive_axis_deadband` | Center deadband for the wheel drive axis. Center (at rest) axis values within the band will be ignored. For example, if an analog stick has a center value of 127, then a deadband value of 20 will ignore all axis values from 117 to 137 |
| `invert_axis_list` | List of axis to flip. For example, the DualShock 4's analog sticks have their fully 'up' values at 0, while fully 'down' is 255. |


## DualShock 4 Default Control Layout ##

![DS4 RRC Layout](https://cdn.hackaday.io/images/8486461617866169857.png "DS4 RRC Layout")

This work, "DS4 RRC Layout", is a derivative of "[Dualshock 4 Layout](https://commons.wikimedia.org/wiki/File:Dualshock_4_Layout.svg)" by [Tokyoship](https://commons.wikimedia.org/wiki/User:Tokyoship), used under [CC BY 3.0](https://creativecommons.org/licenses/by/3.0/). "DS4 RRC Layout" is licensed under [CC BY 3.0](https://creativecommons.org/licenses/by/3.0/) by Josh Gadeken.

![DS4 RRC Layout 2](https://cdn.hackaday.io/images/7791471617866339850.png "DS4 RRC Layout 2")

This work, "DS4 RRC Layout 2", is a derivative of "[Dualshock 4 Layout 2](https://commons.wikimedia.org/wiki/File:Dualshock_4_Layout_2.svg)" by [Tokyoship](https://commons.wikimedia.org/wiki/User:Tokyoship), used under [CC BY 3.0](https://creativecommons.org/licenses/by/3.0/). "DS4 RRC Layout 2" is licensed under [CC BY 3.0](https://creativecommons.org/licenses/by/3.0/) by Josh Gadeken.
