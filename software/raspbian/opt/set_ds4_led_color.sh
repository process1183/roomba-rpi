#!/usr/bin/env bash
# Set DualShock 4 LED bar color
# **The udev 99-ds4.rules file expects this script to be in /opt/**
#
# https://github.com/process1183/roomba-rpi
# Copyright (C) 2021  Josh Gadeken
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

# This script requires two args: device path and color;
# Exit if either are missing.
if [ "$#" -ne 2 ]; then
    exit 2
fi

# Extract the ID from the device path
ID=$(echo "$1" | grep -P -o '(([[:alnum:]]{4}:){2}[[:alnum:]]{4}\.[[:alnum:]]{4})')

# Set paths for each of the LED brightness control files
LED_BASE_STR="/sys/class/leds/${ID}:%s/brightness"
LED_RED=$(printf "${LED_BASE_STR}" red)
LED_GREEN=$(printf "${LED_BASE_STR}" green)
LED_BLUE=$(printf "${LED_BASE_STR}" blue)
LED_GLOBAL=$(printf "${LED_BASE_STR}" global)

# Exit if any of the LED paths are missing
if [ ! -e "${LED_RED}" ] || \
   [ ! -e "${LED_GREEN}" ] || \
   [ ! -e "${LED_BLUE}" ] || \
   [ ! -e "${LED_GLOBAL}" ]
then
    exit 3
fi

# Remove preceding '0x' from color (if present)
RGB=${2#0x}

# Convert hex color into individual decimal colors
RED=$((16#${RGB:0:2}))
GREEN=$((16#${RGB:2:2}))
BLUE=$((16#${RGB:4:2}))

# Set LED colors
echo ${RED} > "${LED_RED}"
echo ${GREEN} > "${LED_GREEN}"
echo ${BLUE} > "${LED_BLUE}"
echo 1 > "${LED_GLOBAL}"
