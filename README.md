## Introduction

This package allows to control a Lewansoul xArm robot from a Linux PC with ROS.  It uses the HID interface between your PC and the Arduino board imbedded in the xArm.  Said Arduino board then communicates with the servos (6 of them) using a serial TTL UART connection.

## Requirements

### Software

Code tested on Linux Ubuntu 18.04 using Python 2.  The following packages are required:
```sh
sudo apt install libhidapi-hidraw0 libhidapi-libusb0 python-hid
pip install --user easyhid
```

### Permissions

The HID device is created with the permissions `root.root`, `0600` by default which means that users can communicate with the robot unless they escalate to root privileges (`sudo`).

To avoid this you should create a `udev` rule.  To get the proper attributes, you must first identify the `hidraw` device created for your device.  You can use `dmesg -w` in a shell and then plug/power on the arm.  Let's assume it's `hidraw3` and launch `sudo udevadm info --attribute-walk /dev/hidraw3 | less`.

You can create a rule using the product name since it seems specific enough to identify the xArm Arduino board.  Create the file
`/etc/udev/rules.d/80-usb-xarm.rules` with the content `SUBSYSTEM=="hidraw", ATTRS{product}=="LOBOT", GROUP="dialout", MODE="0660"` (note that all users will need to be members of the group `dialout`).  To apply the rules without rebooting: `sudo udevadm control --reload-rules` and `sudo udevadm trigger` (this is a one time setup).

### `hidraw` device not created properly

The last issue is a lot more annoying and we don't have a good fix.  From time to time, the `hidraw` device is not properly created.  I found an error message in ` /var/log/syslog` related to FuEngine.  This seems to be a feature to automatically check/upgrade a device's firmware.  The only way we found to avoid this is to disable the service `fwupd` using `sudo service fwupd stop` (has to be done after every reboot) or ` sudo apt remove fwupd ` (completely remove fwupd).  If anyone has a better solution, please raise your hand.
