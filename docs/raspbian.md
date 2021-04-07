# Roomba RPi Raspbian #

Roomba RPi Raspbian configuration.


## First Steps ##

*Note*: It is suggested to perform these steps before installing the RPi ZW in the Roomba.

* Flash Micro SD card with Raspbian Lite
* Run `raspi-config` (Roomba RPi specific details are below)
* Update Raspbian
* Configure firewall (nftables)
* Harden SSH


## `raspi-config` ##

Run `sudo raspi-config` to:

* Configure WiFi
* Enable I2C
* Enable login shell over serial
* Set GPU memory to `16`
* Change other options as desired


## General Dependencies ##

Install the required software:

```
sudo apt-get update
sudo apt-get install git i2c-tools python3-pip
```


## `roomba-rpi` Git Repo ##

Clone this Git repo to your home directory on the RPi:

```
cd ~/
git clone https://github.com/process1183/roomba-rpi.git
```

## DS3231 RTC ##

(See the [Adding a Real Time Clock to Raspberry Pi](https://learn.adafruit.com/adding-a-real-time-clock-to-raspberry-pi?view=all) page for a complete RPi RTC tutorial.)

To enable the DS3231 real time clock, add the following lines to `/boot/config.txt`:

```
# Adafruit DS3231 Precision RTC Breakout
# https://learn.adafruit.com/adafruit-ds3231-precision-rtc-breakout?view=all
dtoverlay=i2c-rtc,ds3231
```

A copy of the complete Roomba RPi's `config.txt` is here: `roomba-rpi/software/raspbian/boot/config.txt`

Next, purge the `fake-hwclock` package:

```
sudo apt-get purge fake-hwclock
```

Next, the `/lib/udev/hwclock-set` script needs several modifications:

```diff
--- /lib/udev/hwclock-set.orig	2021-04-01 01:30:43.000000000 -0700
+++ /lib/udev/hwclock-set	2021-04-01 04:24:11.785994254 -0700
@@ -4,9 +4,9 @@

 dev=$1

-if [ -e /run/systemd/system ] ; then
-    exit 0
-fi
+#if [ -e /run/systemd/system ] ; then
+#    exit 0
+#fi

 if [ -e /run/udev/hwclock-set ]; then
     exit 0
@@ -26,10 +26,10 @@
 fi

 if [ yes = "$BADYEAR" ] ; then
-    /sbin/hwclock --rtc=$dev --systz --badyear
+#    /sbin/hwclock --rtc=$dev --systz --badyear
     /sbin/hwclock --rtc=$dev --hctosys --badyear
 else
-    /sbin/hwclock --rtc=$dev --systz
+#    /sbin/hwclock --rtc=$dev --systz
     /sbin/hwclock --rtc=$dev --hctosys
 fi

```

A copy of the modified `hwclock-set` script is here: `roomba-rpi/software/raspbian/lib/udev/hwclock-set`

Finally, set the RTC time from the RPi's time:

```
sudo hwclock -w
```


## BNO055 IMU ##

As noted in [Adafruit BNO055 Absolute Orientation Sensor Guide](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor?view=all), the Raspberry Pi's I2C clock needs to be slowed down to properly work with he BNO055. This is done by adding the following lines to `/boot/config.txt`:

```
# The RPi I2C clock needs to be slowed down (stretched) to work with the BNO055
# https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/i2c-clock-stretching
dtparam=i2c_arm_baudrate=10000
```

A copy of the complete Roomba RPi's `config.txt` is here: `roomba-rpi/software/raspbian/boot/config.txt`.


## Shutdown Button ##

Safely shutdown the RPi with a button on GPIO 22 and a Device Tree overlay. Add the following to `/boot/config.txt`:

```
# Shutdown when GPIO22 is driven high
dtoverlay=gpio-shutdown,gpio_pin=22,active_low=0,gpio_pull=down
```

A copy of the complete Roomba RPi's `config.txt` is here: `roomba-rpi/software/raspbian/boot/config.txt`.


## Disable HDMI ##

The RPi's HDMI can be disabled to save power. Add the following to `/etc/rc.local`:

```
# Disable HDMI to save power
# https://www.jeffgeerling.com/blogs/jeff-geerling/raspberry-pi-zero-conserve-energy
/usr/bin/tvservice -o
```

Additionally, since HDMI is disabled, remove or comment out the IP address printing lines in `/etc/rc.local`.

A copy of the `rc.local` file with these changes is here: `roomba-rpi/software/raspbian/etc/rc.local`.


## DualShock 4 LED Bar Color ##

*(Optional)* Udev rule and script to set the DS4 LED bar color after it connects.

First, edit `roomba-rpi/software/raspbian/etc/udev/rules.d/99-ds4.rules` according to the instructions at the top of the file. Then copy the modified Udev rule and LED bar script into place:

```
sudo cp ~/roomba-rpi/software/raspbian/etc/udev/rules.d/99-ds4.rules /etc/udev/rules.d/99-ds4.rules
sudo cp ~/roomba-rpi/software/raspbian/opt/set_ds4_led_color.sh /opt/set_ds4_led_color.sh
sudo chown root:root /opt/set_ds4_led_color.sh
sudo chmod 0544 /opt/set_ds4_led_color.sh
```
