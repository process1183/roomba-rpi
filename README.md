# Roomba RPi #

Project to enable enhanced capabilities, such as remote control and improved navigation, of my iRobot Roomba 690 with the addition of a Raspberry Pi Zero W and IMU.

This repo contains the technical documentation, schematic, configuration files, and software for the Roomba RPi project.

Project logs, additional photos, and goals are available on the [Hackaday.io Roomba RPi Project Page](https://hackaday.io/project/178565-roomba-rpi).

---

![Roomba RPi](https://cdn.hackaday.io/images/3599631617161680999.jpg "Roomba RPi")

This is the Roomba after the hardware modifications were completed. (As intended, it looks no different than before the alterations).

---

![Roomba RPi Internals 1](https://cdn.hackaday.io/images/8201331617161814883.jpg "Roomba RPi Internals 1")

The outer ring top cover is trivial to remove and replace, providing access to the Roomba serial connector, RPi serial port, and RPi shutdown button. The center top cover can be popped off to access the serial board, RTC battery, and Micro SD card (although the Roomba's clear button shroud needs to be unscrewed to get to the Micro SD card).

---

![Roomba RPi Internals 2](https://cdn.hackaday.io/images/435191617161771995.jpg "Roomba RPi Internals 2")

With the Roomba mostly disassembled, you can see the voltage regulator, USB to serial adapter board, RTC, IMU, and Raspberry Pi Zero W. The unconnected cable near the top left is for the Roomba serial port connector. The unconnected cable near the bottom right plugs into the RPi serial and shutdown button board, which is attached to the Roomba's upper chassis cover.
